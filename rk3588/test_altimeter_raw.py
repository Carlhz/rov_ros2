#!/usr/bin/env python3
"""裸串口测试 — 发查询命令，收高度计原始回复"""
import termios, os, fcntl, time, select, sys

tty = os.open("/dev/ttyS5", os.O_RDWR | os.O_NOCTTY)
attr = termios.tcgetattr(tty)
attr[4] = termios.B9600
attr[5] = termios.B9600
attr[2] = attr[2] & ~termios.CSTOPB    # 1 stop bit
attr[2] = attr[2] & ~termios.PARENB    # 8N1
attr[3] = attr[3] & ~termios.ICANON    # raw mode
termios.tcsetattr(tty, termios.TCSANOW, attr)

for i in range(3):
    cmd = bytes([0xAA, 0xA0, 0x01, 0x00, 0x00, 0xAB])
    os.write(tty, cmd)
    time.sleep(0.15)

    fcntl.fcntl(tty, fcntl.F_SETFL, fcntl.fcntl(tty, fcntl.F_GETFL) | os.O_NONBLOCK)
    r, _, _ = select.select([tty], [], [], 1.0)
    if r:
        data = os.read(tty, 64)
        print(f"[{i}] RAW: {data.hex()}  len={len(data)}")
    else:
        print(f"[{i}] TIMEOUT - no response")
os.close(tty)
