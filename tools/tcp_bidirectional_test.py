#!/usr/bin/env python3
"""Bidirectional TCP <-> WiFi module <-> CH340 serial test.

Topology:
  PC (TCP server :6666) <--WiFi--> Ai-WB2 <--UART--> CH340 (/dev/cu.usbserial-130)

The Ai-WB2 is configured as TCP-client transparent mode to 192.168.223.205:6666.
"""

import argparse
import socket
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("pip install pyserial")
    sys.exit(1)

SERIAL_PORT = "/dev/cu.usbserial-130"
SERIAL_BAUD = 115200
TCP_HOST = "0.0.0.0"
TCP_PORT = 6666
WAIT_CLIENT_TIMEOUT = 15.0


def ts() -> str:
    return time.strftime("%H:%M:%S")


def main() -> None:
    parser = argparse.ArgumentParser(description="CH340 <-> WiFi module <-> TCP bidirectional test")
    parser.add_argument("--serial", default=SERIAL_PORT)
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD)
    parser.add_argument("--host", default=TCP_HOST)
    parser.add_argument("--port", type=int, default=TCP_PORT)
    args = parser.parse_args()

    # ---------- open serial ----------
    ser = serial.Serial(args.serial, args.baud, timeout=0.05)
    print(f"[{ts()}] 串口已打开: {args.serial} @ {args.baud}")

    # ---------- start TCP server ----------
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(1)
    server.settimeout(1.0)
    print(f"[{ts()}] TCP 服务端监听 {args.host}:{args.port}")
    print(f"[{ts()}] 等待 Ai-WB2 连接...")

    client: socket.socket | None = None
    deadline = time.time() + WAIT_CLIENT_TIMEOUT
    while client is None and time.time() < deadline:
        try:
            conn, addr = server.accept()
            client = conn
            client.settimeout(0.3)
            print(f"[{ts()}] TCP 客户端已连接: {addr[0]}:{addr[1]}")
        except socket.timeout:
            print(f"[{ts()}] 还在等...")
            # Flush any stale data from serial
            while ser.read(256):
                pass

    if client is None:
        print(f"[{ts()}] 超时: {WAIT_CLIENT_TIMEOUT}s 内没有 TCP 客户端连接")
        server.close()
        ser.close()
        sys.exit(1)

    # ---------- bidirectional test ----------
    test_count = 0
    ok_tx = 0
    ok_rx = 0

    def print_serial(label: str) -> None:
        time.sleep(0.3)
        data = ser.read(4096)
        if data:
            text = data.decode("utf-8", errors="replace").strip()
            print(f"[{ts()}] {label}: {text!r}")

    try:
        for i in range(3):
            test_count += 1
            print(f"\n{'='*50}")
            print(f"[{ts()}] 测试 #{test_count}")

            # Step 1: Serial -> WiFi -> TCP
            msg = f"hello-from-ch340-{i+1}\r\n"
            ser.write(msg.encode())
            ser.flush()
            print(f"[{ts()}] 串口发送: {msg.strip()!r}")

            time.sleep(0.5)
            try:
                data = client.recv(4096)
                if data:
                    text = data.decode("utf-8", errors="replace").strip()
                    print(f"[{ts()}] TCP 收到: {text!r}")
                    ok_tx += 1
                else:
                    print(f"[{ts()}] TCP 收到空/断开")
            except socket.timeout:
                print(f"[{ts()}] TCP 超时未收到 (Tx 方向)")

            # Step 2: TCP -> WiFi -> Serial
            reply = f"pong-from-pc-{i+1}\r\n"
            try:
                client.sendall(reply.encode())
                print(f"[{ts()}] TCP 发送: {reply.strip()!r}")
            except OSError as exc:
                print(f"[{ts()}] TCP 发送失败: {exc}")

            time.sleep(0.5)
            rx_data = ser.read(4096)
            if rx_data:
                text = rx_data.decode("utf-8", errors="replace").strip()
                print(f"[{ts()}] 串口收到: {text!r}")
                if reply.strip() in text:
                    ok_rx += 1
            else:
                print(f"[{ts()}] 串口超时未收到 (Rx 方向)")

    except KeyboardInterrupt:
        print(f"\n[{ts()}] 用户中断")
    finally:
        client.close()
        server.close()
        ser.close()

    # ---------- summary ----------
    print(f"\n{'='*50}")
    print(f"[{ts()}] 测试完成")
    print(f"  Tx (串口->TCP): {ok_tx}/{test_count}")
    print(f"  Rx (TCP->串口): {ok_rx}/{test_count}")
    if ok_tx + ok_rx == test_count * 2:
        print("  => 双向 TCP 透明传输正常")
    else:
        print("  => 存在问题，请检查")


if __name__ == "__main__":
    main()
