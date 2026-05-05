#!/usr/bin/env python3
"""End-to-end Ai-WB2 TCP transparent-mode loop test via CH340.

This script treats the CH340 serial port as the MCU side and verifies:
1) serial -> Wi-Fi module -> PC TCP server
2) PC TCP server -> Wi-Fi module -> serial
"""

from __future__ import annotations

import argparse
import os
import socket
import threading
import time

import serial


DEFAULT_WIFI_SSID = os.getenv("AIWB2_WIFI_SSID")
DEFAULT_WIFI_PASSWORD = os.getenv("AIWB2_WIFI_PASSWORD")


def env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        return default


DEFAULT_TCP_PORT = env_int("AIWB2_TCP_PORT", 6666)


def read_idle(port: serial.Serial, idle_s: float = 0.5, total_s: float = 8.0) -> bytes:
    end = time.monotonic() + total_s
    idle_end = time.monotonic() + idle_s
    chunks = bytearray()
    while time.monotonic() < end:
        data = port.read(4096)
        if data:
            chunks.extend(data)
            idle_end = time.monotonic() + idle_s
        elif time.monotonic() >= idle_end:
            break
    return bytes(chunks)


def send_at(port: serial.Serial, command: str, total_s: float = 8.0) -> str:
    port.write((command + "\r\n").encode("utf-8"))
    port.flush()
    response = read_idle(port, total_s=total_s).decode("utf-8", errors="replace")
    return response


def ensure_at_mode(port: serial.Serial) -> None:
    port.reset_input_buffer()
    port.reset_output_buffer()

    resp = send_at(port, "AT", total_s=2.0)
    if "OK" in resp or "ERROR" in resp or "+EVENT:" in resp:
        print("AT-mode probe:", repr(resp))
        return

    time.sleep(1.2)
    port.write(b"+++")
    port.flush()
    time.sleep(1.2)
    resp = read_idle(port, total_s=2.0).decode("utf-8", errors="replace")
    print("after +++:", repr(resp))

    resp = send_at(port, "AT", total_s=2.0)
    print("AT probe 2:", repr(resp))


def configure_module(port: serial.Serial, ssid: str, password: str, pc_ip: str, pc_port: int) -> None:
    commands = [
        ("ATE0", 2.0),
        ("AT+WMODE=1,1", 2.0),
        (f'AT+WJAP="{ssid}","{password}"', 20.0),
        ("AT+WAUTOCONN=1", 2.0),
        (f"AT+SOCKETAUTOTT=4,{pc_ip},{pc_port}", 3.0),
        ("AT+SOCKETDEL=1", 2.0),
    ]
    for command, timeout_s in commands:
        response = send_at(port, command, total_s=timeout_s)
        print(f">>> {command}")
        print(response or "<empty>")


def wait_for_client(server: socket.socket, timeout_s: float = 20.0) -> socket.socket:
    server.settimeout(timeout_s)
    client, addr = server.accept()
    print(f"TCP client connected from {addr[0]}:{addr[1]}")
    client.settimeout(0.2)
    return client


def read_tcp_until_contains(client: socket.socket, needle: bytes, timeout_s: float) -> bytes:
    end = time.monotonic() + timeout_s
    buf = bytearray()
    while time.monotonic() < end:
        try:
            data = client.recv(4096)
        except socket.timeout:
            continue
        if not data:
            break
        buf.extend(data)
        if needle in buf:
            break
    return bytes(buf)


def read_serial_until_contains(port: serial.Serial, needle: bytes, timeout_s: float) -> bytes:
    end = time.monotonic() + timeout_s
    buf = bytearray()
    while time.monotonic() < end:
        data = port.read(4096)
        if data:
            buf.extend(data)
            if needle in buf:
                break
        else:
            time.sleep(0.02)
    return bytes(buf)


def main() -> int:
    parser = argparse.ArgumentParser(description="Ai-WB2 TCP transparent-mode loop test")
    parser.add_argument("--serial-port", required=True)
    parser.add_argument("--ssid",
                        default=DEFAULT_WIFI_SSID,
                        required=DEFAULT_WIFI_SSID is None)
    parser.add_argument("--password",
                        default=DEFAULT_WIFI_PASSWORD,
                        required=DEFAULT_WIFI_PASSWORD is None)
    parser.add_argument("--pc-ip", required=True)
    parser.add_argument("--pc-port", type=int, default=DEFAULT_TCP_PORT)
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    serial_marker = f"SERIAL_TO_TCP_{int(time.time())}".encode("utf-8")
    tcp_marker = f"TCP_TO_SERIAL_{int(time.time())}".encode("utf-8")

    with serial.Serial(args.serial_port, args.baud, timeout=0.05) as port:
        ensure_at_mode(port)
        configure_module(port, args.ssid, args.password, args.pc_ip, args.pc_port)

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", args.pc_port))
        server.listen(1)
        print(f"TCP server listening on 0.0.0.0:{args.pc_port}")

        print(">>> AT+RST")
        print(send_at(port, "AT+RST", total_s=6.0) or "<empty>")
        client = wait_for_client(server, timeout_s=25.0)
        try:
            port.reset_input_buffer()
            port.reset_output_buffer()

            print("TEST 1 serial -> tcp")
            port.write(serial_marker + b"\r\n")
            port.flush()
            tcp_rx = read_tcp_until_contains(client, serial_marker, timeout_s=8.0)
            print("tcp_rx:", tcp_rx.decode("utf-8", errors="replace"))

            print("TEST 2 tcp -> serial")
            client.sendall(tcp_marker + b"\r\n")
            serial_rx = read_serial_until_contains(port, tcp_marker, timeout_s=8.0)
            print("serial_rx:", serial_rx.decode("utf-8", errors="replace"))

            ok1 = serial_marker in tcp_rx
            ok2 = tcp_marker in serial_rx
            print(f"RESULT serial_to_tcp={ok1} tcp_to_serial={ok2}")
            return 0 if (ok1 and ok2) else 1
        finally:
            client.close()
            server.close()


if __name__ == "__main__":
    raise SystemExit(main())
