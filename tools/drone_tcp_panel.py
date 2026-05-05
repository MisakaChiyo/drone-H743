#!/usr/bin/env python3
"""TCP control panel for the drone-H743 Ai-WB2 transparent link."""

from __future__ import annotations

import queue
import re
import socket
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 6666


def parse_kv(line: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for match in re.finditer(r"([A-Za-z0-9_]+)=([^ \r\n]+)", line):
        result[match.group(1)] = match.group(2)
    return result


class TcpServer:
    def __init__(self, rx_queue: "queue.Queue[str]") -> None:
        self.rx_queue = rx_queue
        self.sock: socket.socket | None = None
        self.client: socket.socket | None = None
        self.thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

    def start(self, host: str, port: int) -> None:
        self.stop()
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, args=(host, port), daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        with self.lock:
            sockets = [self.client, self.sock]
            self.client = None
            self.sock = None
        for item in sockets:
            if item is not None:
                try:
                    item.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    item.close()
                except OSError:
                    pass

    def send_line(self, line: str) -> bool:
        payload = (line.rstrip("\r\n") + "\r\n").encode("utf-8")
        with self.lock:
            client = self.client
        if client is None:
            return False
        try:
            client.sendall(payload)
            return True
        except OSError as exc:
            self.rx_queue.put(f"[上位机] 发送失败: {exc}")
            return False

    def _run(self, host: str, port: int) -> None:
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((host, port))
            server.listen(1)
            server.settimeout(0.5)
            with self.lock:
                self.sock = server
            self.rx_queue.put(f"[上位机] 正在监听 {host}:{port}")
        except OSError as exc:
            self.rx_queue.put(f"[上位机] 监听失败: {exc}")
            return

        while not self.stop_event.is_set():
            try:
                client, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            with self.lock:
                if self.client is not None:
                    try:
                        self.client.close()
                    except OSError:
                        pass
                self.client = client
            self.rx_queue.put(f"[上位机] 板子已连接: {addr[0]}:{addr[1]}")
            self._read_client(client)

        self.rx_queue.put("[上位机] TCP 服务已停止")

    def _read_client(self, client: socket.socket) -> None:
        client.settimeout(0.5)
        buffer = b""
        while not self.stop_event.is_set():
            try:
                data = client.recv(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break
            if b"\n" not in data:
                text = data.decode("utf-8", errors="replace")
                shown = text.replace("\r", "\\r").replace("\n", "\\n")
                self.rx_queue.put(f"RXRAW len={len(data)} data={shown}")
            buffer += data
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                self.rx_queue.put(line.decode("utf-8", errors="replace").rstrip("\r"))
        with self.lock:
            if self.client is client:
                self.client = None
        try:
            client.close()
        except OSError:
            pass
        self.rx_queue.put("[上位机] 板子已断开")


class DronePanel(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("drone-H743 TCP 上位机")
        self.geometry("1120x760")
        self.rx_queue: "queue.Queue[str]" = queue.Queue()
        self.server = TcpServer(self.rx_queue)
        self.last_board_rx = 0.0
        self.hardware_labels: dict[str, dict[str, tk.StringVar]] = {}
        self.link_var = tk.StringVar(value="未连接")
        self._build_ui()
        self.after(1000, self._check_link_health)
        self.after(100, self._drain_rx)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        conn = ttk.Frame(root)
        conn.pack(fill=tk.X)
        ttk.Label(conn, text="监听地址").pack(side=tk.LEFT)
        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        ttk.Entry(conn, textvariable=self.host_var, width=16).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(conn, text="端口").pack(side=tk.LEFT)
        self.port_var = tk.IntVar(value=DEFAULT_PORT)
        ttk.Entry(conn, textvariable=self.port_var, width=8).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Button(conn, text="启动 TCP 服务", command=self._start).pack(side=tk.LEFT)
        ttk.Button(conn, text="停止", command=self._stop).pack(side=tk.LEFT, padx=6)
        ttk.Button(conn, text="连通测试", command=lambda: self._send("PING")).pack(side=tk.LEFT, padx=(18, 4))
        ttk.Button(conn, text="硬件状态", command=lambda: self._send("STATUS?")).pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="读取配置", command=lambda: self._send("CONFIG?")).pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="保存到 Flash", command=lambda: self._send("SAVE")).pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="从 Flash 读取", command=lambda: self._send("LOAD")).pack(side=tk.LEFT, padx=4)

        body = ttk.PanedWindow(root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        left = ttk.Frame(body)
        right = ttk.Frame(body)
        body.add(left, weight=3)
        body.add(right, weight=2)

        link_bar = ttk.Frame(left)
        link_bar.pack(fill=tk.X, pady=(0, 8))
        ttk.Label(link_bar, text="链路状态").pack(side=tk.LEFT)
        ttk.Label(link_bar, textvariable=self.link_var).pack(side=tk.LEFT, padx=(8, 0))

        health = ttk.LabelFrame(left, text="硬件健康状态", padding=8)
        health.pack(fill=tk.X, pady=(0, 8))
        self._build_hardware_health(health)

        self.status_text = tk.Text(left, height=18, wrap=tk.NONE)
        self.status_text.pack(fill=tk.BOTH, expand=True)
        self.status_text.configure(font=("Menlo", 12))

        cmd_frame = ttk.Frame(left)
        cmd_frame.pack(fill=tk.X, pady=(8, 0))
        self.cmd_var = tk.StringVar()
        cmd_entry = ttk.Entry(cmd_frame, textvariable=self.cmd_var)
        cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        cmd_entry.bind("<Return>", lambda _event: self._send_custom())
        ttk.Button(cmd_frame, text="发送", command=self._send_custom).pack(side=tk.LEFT, padx=(6, 0))

        servo_notebook = ttk.Notebook(right)
        servo_notebook.pack(fill=tk.BOTH, expand=True)
        self.servo_widgets: list[dict[str, tk.Variable]] = []
        for index in range(2):
            frame = ttk.Frame(servo_notebook, padding=10)
            servo_notebook.add(frame, text=f"舵机 {index}")
            self._build_servo_tab(frame, index)

        raw = ttk.LabelFrame(right, text="手动原始舵机指令", padding=10)
        raw.pack(fill=tk.X, pady=(10, 0))
        self.raw_var = tk.StringVar(value="{#001P1500T0500!#002P1500T0500!}")
        ttk.Entry(raw, textvariable=self.raw_var).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(raw, text="发送原始指令", command=self._send_raw).pack(side=tk.LEFT, padx=(6, 0))

    def _build_hardware_health(self, parent: ttk.Frame) -> None:
        headers = ["硬件", "状态", "失败阶段", "关键读数", "返回码", "提示"]
        for column, header in enumerate(headers):
            ttk.Label(parent, text=header).grid(row=0, column=column, sticky=tk.W, padx=4)

        items = [
            ("FLASH", "GD25Q32 Flash"),
            ("SPL06", "SPL06 气压计"),
            ("ICM42688", "ICM42688 IMU"),
            ("UART1", "USART1 透明链路"),
        ]
        for row, (key, title) in enumerate(items, start=1):
            values = {
                "state": tk.StringVar(value="等待数据"),
                "stage": tk.StringVar(value="-"),
                "value": tk.StringVar(value="-"),
                "code": tk.StringVar(value="-"),
                "hint": tk.StringVar(value="点击“硬件状态”或等待心跳"),
            }
            self.hardware_labels[key] = values
            ttk.Label(parent, text=title).grid(row=row, column=0, sticky=tk.W, padx=4, pady=3)
            ttk.Label(parent, textvariable=values["state"]).grid(row=row, column=1, sticky=tk.W, padx=4, pady=3)
            ttk.Label(parent, textvariable=values["stage"]).grid(row=row, column=2, sticky=tk.W, padx=4, pady=3)
            ttk.Label(parent, textvariable=values["value"]).grid(row=row, column=3, sticky=tk.W, padx=4, pady=3)
            ttk.Label(parent, textvariable=values["code"]).grid(row=row, column=4, sticky=tk.W, padx=4, pady=3)
            ttk.Label(parent, textvariable=values["hint"], wraplength=360).grid(row=row, column=5, sticky=tk.W, padx=4, pady=3)

        for column in range(len(headers)):
            parent.columnconfigure(column, weight=1 if column == 5 else 0)

    def _build_servo_tab(self, parent: ttk.Frame, index: int) -> None:
        values: dict[str, tk.Variable] = {
            "id": tk.IntVar(value=index + 1),
            "pulse": tk.IntVar(value=1500),
            "time": tk.IntVar(value=500),
            "mode": tk.IntVar(value=1),
            "enabled": tk.IntVar(value=1),
            "new_id": tk.IntVar(value=index + 1),
            "baud": tk.IntVar(value=4),
        }
        self.servo_widgets.append(values)

        row = 0
        ttk.Checkbutton(parent, text="启用此舵机槽位", variable=values["enabled"],
                        command=lambda i=index: self._servo_enable(i)).grid(row=row, column=0, sticky=tk.W)
        row += 1
        self._spin(parent, row, "当前舵机 ID", values["id"], 0, 255,
                   lambda i=index: self._servo_set_id(i))
        row += 1
        self._scale(parent, row, "目标位置 us", values["pulse"], 500, 2500)
        row += 1
        self._spin(parent, row, "运行时间 ms", values["time"], 0, 9999, None)
        row += 1
        self._spin(parent, row, "模式 1-8", values["mode"], 1, 8,
                   lambda i=index: self._servo_mode(i))
        row += 1
        ttk.Button(parent, text="移动此舵机", command=lambda i=index: self._servo_move(i)).grid(row=row, column=0, pady=6, sticky=tk.EW)
        ttk.Button(parent, text="按配置同时移动两路", command=lambda: self._send("SERVO MOVEALL")).grid(row=row, column=1, pady=6, sticky=tk.EW)
        row += 1

        id_box = ttk.LabelFrame(parent, text="修改实体舵机 ID", padding=8)
        id_box.grid(row=row, column=0, columnspan=2, sticky=tk.EW, pady=(8, 4))
        ttk.Spinbox(id_box, from_=0, to=255, textvariable=values["new_id"], width=8).pack(side=tk.LEFT)
        ttk.Button(id_box, text="写入新 ID", command=lambda i=index: self._servo_set_physical_id(i)).pack(side=tk.LEFT, padx=6)
        row += 1

        actions = ttk.LabelFrame(parent, text="众灵手册动作指令", padding=8)
        actions.grid(row=row, column=0, columnspan=2, sticky=tk.EW)
        action_names = [
            ("读取版本", "VER"),
            ("检测 ID", "PID"),
            ("读取位置", "RAD"),
            ("读取模式", "MOD?"),
            ("释放扭力", "ULK"),
            ("恢复扭力", "ULR"),
            ("暂停", "DPT"),
            ("继续", "DCT"),
            ("停止", "DST"),
            ("当前位置设中位", "SCK"),
            ("设置启动位置", "CSD"),
            ("清除启动位置", "CSM"),
            ("恢复启动位置", "CSR"),
            ("设置最小值", "SMI"),
            ("设置最大值", "SMX"),
            ("半恢复出厂", "CLEO"),
            ("全恢复出厂", "CLE"),
        ]
        for n, (label, command) in enumerate(action_names):
            ttk.Button(actions, text=label,
                       command=lambda i=index, c=command: self._servo_cmd(i, c)).grid(
                           row=n // 2, column=n % 2, padx=3, pady=3, sticky=tk.EW
                       )
        actions.columnconfigure(0, weight=1)
        actions.columnconfigure(1, weight=1)

        baud_box = ttk.Frame(parent)
        baud_box.grid(row=row + 1, column=0, columnspan=2, sticky=tk.EW, pady=(8, 0))
        ttk.Label(baud_box, text="波特率代码").pack(side=tk.LEFT)
        ttk.Spinbox(baud_box, from_=0, to=7, textvariable=values["baud"], width=5).pack(side=tk.LEFT, padx=6)
        ttk.Button(baud_box, text="设置波特率", command=lambda i=index: self._servo_baud(i)).pack(side=tk.LEFT)

        parent.columnconfigure(1, weight=1)

    def _spin(self, parent: ttk.Frame, row: int, label: str, variable: tk.Variable,
              minimum: int, maximum: int, command) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=4)
        box = ttk.Spinbox(parent, from_=minimum, to=maximum, textvariable=variable, width=10)
        box.grid(row=row, column=1, sticky=tk.EW, pady=4)
        if command is not None:
            ttk.Button(parent, text="应用", command=command).grid(row=row, column=2, padx=(6, 0))

    def _scale(self, parent: ttk.Frame, row: int, label: str, variable: tk.Variable,
               minimum: int, maximum: int) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=4)
        scale = ttk.Scale(parent, from_=minimum, to=maximum, variable=variable, orient=tk.HORIZONTAL)
        scale.grid(row=row, column=1, sticky=tk.EW, pady=4)
        ttk.Spinbox(parent, from_=minimum, to=maximum, textvariable=variable, width=8).grid(row=row, column=2, padx=(6, 0))

    def _start(self) -> None:
        try:
            port = int(self.port_var.get())
        except tk.TclError:
            messagebox.showerror("端口错误", "端口号无效")
            return
        self.server.start(self.host_var.get(), port)

    def _stop(self) -> None:
        self.server.stop()

    def _send(self, line: str) -> None:
        self._append(f"> {line}")
        if not self.server.send_line(line):
            self._append("[上位机] 板子尚未连接")
        elif line in {"PING", "STATUS?", "CONFIG?"}:
            self.after(1200, lambda sent=line: self._warn_if_no_reply(sent))

    def _send_custom(self) -> None:
        line = self.cmd_var.get().strip()
        if line:
            self._send(line)
            self.cmd_var.set("")

    def _send_raw(self) -> None:
        self._send(f"SERVO RAW {self.raw_var.get().strip()}")

    def _servo_values(self, index: int) -> dict[str, int]:
        widgets = self.servo_widgets[index]
        return {key: int(var.get()) for key, var in widgets.items()}

    def _servo_move(self, index: int) -> None:
        values = self._servo_values(index)
        self._send(f"SERVO MOVE {index} {values['pulse']} {values['time']}")

    def _servo_mode(self, index: int) -> None:
        values = self._servo_values(index)
        self._send(f"SERVO MODE {index} {values['mode']}")

    def _servo_enable(self, index: int) -> None:
        values = self._servo_values(index)
        self._send(f"SERVO ENABLE {index} {values['enabled']}")

    def _servo_set_id(self, index: int) -> None:
        values = self._servo_values(index)
        self._send(f"SERVO ID {index} {values['id']}")

    def _servo_set_physical_id(self, index: int) -> None:
        values = self._servo_values(index)
        self._send(f"SERVO SETID {index} {values['new_id']}")

    def _servo_cmd(self, index: int, command: str) -> None:
        self._send(f"SERVO CMD {index} {command}")

    def _servo_baud(self, index: int) -> None:
        values = self._servo_values(index)
        self._send(f"SERVO CMD {index} BD {values['baud']}")

    def _append(self, line: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.status_text.insert(tk.END, f"[{stamp}] {line}\n")
        self.status_text.see(tk.END)

    def _handle_board_line(self, line: str) -> None:
        if not line.startswith("[上位机]"):
            self.last_board_rx = time.monotonic()
            self.link_var.set("已收到 STM32 数据，TCP 与串口透明链路正常")

        if line.startswith("READY"):
            self.last_board_rx = time.monotonic()
            self.link_var.set("STM32 心跳正常")
        elif line.startswith("HW "):
            self._update_hardware_line(line)
        elif line.startswith("UART1 "):
            self._update_uart_line(line)

    def _update_hardware_line(self, line: str) -> None:
        parts = line.split(maxsplit=2)
        if len(parts) < 2:
            return
        key = parts[1]
        values = parse_kv(line)
        labels = self.hardware_labels.get(key)
        if labels is None:
            return

        ok = values.get("ok") == "1"
        labels["state"].set("正常" if ok else "异常")
        labels["stage"].set(self._stage_text(key, values.get("stage", "-")))

        if key == "FLASH":
            labels["value"].set(f"ID={values.get('id', '-')} 期望={values.get('exp', '-')}, SR1={values.get('sr1', '-')}")
            labels["code"].set(f"probe={values.get('probe', '-')} sr={values.get('sr', '-')} read={values.get('read', '-')}")
        elif key == "SPL06":
            labels["value"].set(
                f"ID={values.get('id', '-')} 期望={values.get('exp', '-')}, split={values.get('split_id', '-')}, txrx={values.get('txrx_id', '-')}"
            )
            labels["code"].set(f"init={values.get('init', '-')} split={values.get('split', '-')} txrx={values.get('txrx', '-')}")
        elif key == "ICM42688":
            labels["value"].set(f"WHO={values.get('who', '-')} 期望={values.get('exp', '-')}, n={values.get('n', '-')}")
            labels["code"].set(f"st={values.get('st', '-')}")

        labels["hint"].set(self._hardware_hint(key, ok, values))

    def _update_uart_line(self, line: str) -> None:
        values = parse_kv(line)
        labels = self.hardware_labels.get("UART1")
        if labels is None:
            return

        rx_bytes = int(values.get("rx_bytes", "0"))
        rx_lines = int(values.get("rx_lines", "0"))
        rx_errors = int(values.get("rx_errors", "0"))
        rx_overflows = int(values.get("rx_overflows", "0"))
        ok = rx_errors == 0 and rx_overflows == 0

        labels["state"].set("正常" if ok else "有错误")
        labels["stage"].set("接收统计")
        labels["value"].set(f"bytes={rx_bytes} lines={rx_lines}")
        labels["code"].set(f"overflow={rx_overflows} err={rx_errors}")
        if rx_bytes == 0:
            labels["hint"].set("STM32 正在发心跳，但还没收到上位机命令；检查 Ai-WB2 到 PB15 的 TX/RX。")
        elif rx_lines == 0:
            labels["hint"].set("已经收到字节但没有收到换行；检查上位机发送是否带 CR/LF。")
        elif ok:
            labels["hint"].set("USART1 收发链路已打通。")
        else:
            labels["hint"].set("串口出现溢出或错误；先降低发送频率，再检查波特率和接线。")

    def _stage_text(self, key: str, stage: str) -> str:
        mapping = {
            "ready": "初始化完成",
            "probe": "读取芯片 ID",
            "status": "读取状态寄存器",
            "read": "读取数据",
            "init": "初始化/识别",
        }
        return mapping.get(stage, stage)

    def _hardware_hint(self, key: str, ok: bool, values: dict[str, str]) -> str:
        if ok:
            return "初始化成功，当前读数符合预期。"
        if key == "FLASH":
            return "优先看 SPI1/CS/MISO/MOSI 和 GD25Q32 供电；ID 应为 C84016。"
        if key == "SPL06":
            return "当前重点怀疑 SPI4、CS、MISO、器件方向或实际焊接型号；ID 应为 0x10。"
        if key == "ICM42688":
            return "当前重点怀疑 SPI2/IMU 硬件链路；WHO_AM_I 应为 0x47。"
        if key == "UART1":
            return "确认 USART1: PB14 TX 接 Ai-WB2 RX，PB15 RX 接 Ai-WB2 TX，115200 8N1。"
        return "查看返回码和初始化阶段。"

    def _warn_if_no_reply(self, sent: str) -> None:
        if (time.monotonic() - self.last_board_rx) > 1.0:
            self.link_var.set("Ai-WB2 已连上 TCP，但没有收到 STM32 回复；请检查 USART1 RX/TX 透明串口链路")
            self._append(f"[上位机] {sent} 没收到 STM32 回复：TCP 客户端可能只是 Wi-Fi 模块，串口到 STM32 未通")

    def _check_link_health(self) -> None:
        if self.server.client is None:
            self.link_var.set("等待 Ai-WB2 连接 TCP")
        elif self.last_board_rx == 0.0:
            self.link_var.set("Ai-WB2 已连上 TCP，尚未收到 STM32 数据")
        elif (time.monotonic() - self.last_board_rx) > 5.0:
            self.link_var.set("超过 5 秒未收到 STM32 心跳")
        self.after(1000, self._check_link_health)

    def _drain_rx(self) -> None:
        try:
            while True:
                line = self.rx_queue.get_nowait()
                self._append(line)
                self._handle_board_line(line)
        except queue.Empty:
            pass
        self.after(100, self._drain_rx)

    def _on_close(self) -> None:
        self.server.stop()
        self.destroy()


def main() -> None:
    app = DronePanel()
    app.mainloop()


if __name__ == "__main__":
    main()
