#!/usr/bin/env python3
"""Bridge Ai-WB2 UDP transparent mode to fixed VOFA UDP ports.

Why this exists:
- Ai-WB2 auto transparent mode is configured as UDP client to the PC.
- The module sends from a dynamic source port.
- VOFA expects a fixed remote endpoint when it sends data back.

This bridge listens for module packets on one fixed port, remembers the most
recent module source address, forwards those packets to VOFA's local port, and
forwards VOFA packets back to the remembered module address.
"""

from __future__ import annotations

import argparse
import socket
import sys
import threading
import time


def log(prefix: str, data: bytes, addr: tuple[str, int] | None = None) -> None:
    ts = time.strftime("%H:%M:%S")
    text = data.decode("utf-8", errors="replace")
    if addr is None:
        print(f"[{ts}] {prefix}: {text!r}")
    else:
        print(f"[{ts}] {prefix} {addr[0]}:{addr[1]}: {text!r}")
    sys.stdout.flush()


def main() -> int:
    parser = argparse.ArgumentParser(description="VOFA <-> Ai-WB2 UDP bridge")
    parser.add_argument("--module-listen-port", type=int, default=6666,
                        help="port that module sends to on the PC")
    parser.add_argument("--vofa-listen-port", type=int, default=6668,
                        help="port that VOFA listens on")
    parser.add_argument("--vofa-send-port", type=int, default=6667,
                        help="port that VOFA sends to")
    parser.add_argument("--vofa-host", default="127.0.0.1",
                        help="VOFA host, usually 127.0.0.1")
    args = parser.parse_args()

    module_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    module_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    module_rx.bind(("0.0.0.0", args.module_listen_port))

    vofa_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    vofa_rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    vofa_rx.bind(("0.0.0.0", args.vofa_send_port))

    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    last_module_addr: list[tuple[str, int] | None] = [None]

    print("VOFA UDP bridge started.")
    print(f"Module -> PC bridge listening on 0.0.0.0:{args.module_listen_port}")
    print(f"VOFA  -> bridge should send to {args.vofa_host}:{args.vofa_send_port}")
    print(f"Bridge -> VOFA forwards to {args.vofa_host}:{args.vofa_listen_port}")
    print("Press Ctrl-C to stop.")

    def module_loop() -> None:
        while True:
            data, addr = module_rx.recvfrom(8192)
            last_module_addr[0] = addr
            log("MODULE RX", data, addr)
            tx.sendto(data, (args.vofa_host, args.vofa_listen_port))

    def vofa_loop() -> None:
        while True:
            data, addr = vofa_rx.recvfrom(8192)
            log("VOFA RX", data, addr)
            if last_module_addr[0] is None:
                print("No module source address known yet; drop VOFA packet.")
                sys.stdout.flush()
                continue
            tx.sendto(data, last_module_addr[0])
            log("MODULE TX", data, last_module_addr[0])

    threading.Thread(target=module_loop, daemon=True).start()
    threading.Thread(target=vofa_loop, daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping bridge.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
