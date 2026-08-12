#!/usr/bin/env python3
"""高度计串口调试脚本 — 用 os/termios 直接测试"""
import os, sys, time, select, termios, struct

PORT = '/dev/ttyS3'
BAUD = 9600

# 打开串口
fd = os.open(PORT, os.O_RDWR | os.O_NOCTTY)
print(f"打开 {PORT}, fd={fd}")

# 配置串口
attr = termios.tcgetattr(fd)
attr[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
             | termios.ISTRIP | termios.INLCR | termios.IGNCR
             | termios.ICRNL | termios.IXON)
attr[1] &= ~termios.OPOST
attr[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
attr[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
attr[3] &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
attr[4] = termios.B9600
attr[5] = termios.B9600
attr[6][termios.VMIN] = 0
attr[6][termios.VTIME] = 0
termios.tcsetattr(fd, termios.TCSANOW, attr)
termios.tcflush(fd, termios.TCIOFLUSH)
print("串口已配置: 9600 8N1 raw")

# 构建命令
cmd = bytearray([0xAA, 0xA0, 0x01, 0x00, 0x00])
cmd.append(cmd[0] ^ cmd[1] ^ cmd[2] ^ cmd[3] ^ cmd[4])
print(f"命令: {bytes(cmd).hex()} ({len(cmd)} bytes)")

# 测试1: 发送命令后读取
print("\n=== 测试1: 发送命令 + 等待0.5s + 读取 ===")
termios.tcflush(fd, termios.TCIFLUSH)
n = os.write(fd, bytes(cmd))
print(f"发送: {n} bytes")
termios.tcdrain(fd)
print("tcdrain 完成")
time.sleep(0.5)

# 读取
buf = b''
deadline = time.time() + 2.0
while time.time() < deadline:
    r, _, _ = select.select([fd], [], [], 0.1)
    if r:
        chunk = os.read(fd, 64)
        if chunk:
            buf += chunk
            print(f"  收到 {len(chunk)} bytes: {chunk.hex()}")
        else:
            print("  os.read 返回空 (EOF)")
            break
    else:
        if buf:
            break  # 已经收到数据，再等一小段看有没有更多

print(f"总计收到 {len(buf)} bytes: {buf.hex() if buf else '(空)'}")

# 测试2: 不发命令，持续读取2秒
print("\n=== 测试2: 不发命令，持续读取3秒 ===")
buf2 = b''
deadline2 = time.time() + 3.0
while time.time() < deadline2:
    r, _, _ = select.select([fd], [], [], 0.1)
    if r:
        chunk = os.read(fd, 64)
        if chunk:
            buf2 += chunk
            print(f"  收到 {len(chunk)} bytes: {chunk.hex()}")
        else:
            break
    # 持续等待

print(f"总计收到 {len(buf2)} bytes: {buf2.hex() if buf2 else '(空)'}")

# 测试3: 发送命令后用 VMIN/VTIME 读取
print("\n=== 测试3: 发送命令 + VMIN=1 VTIME=20 (2s超时) ===")
attr3 = termios.tcgetattr(fd)
attr3[6][termios.VMIN] = 1
attr3[6][termios.VTIME] = 20
termios.tcsetattr(fd, termios.TCSANOW, attr3)
termios.tcflush(fd, termios.TCIFLUSH)
os.write(fd, bytes(cmd))
termios.tcdrain(fd)

try:
    data = os.read(fd, 64)
    print(f"收到 {len(data)} bytes: {data.hex() if data else '(空)'}")
except OSError as e:
    print(f"读取超时或错误: {e}")

# 解析
if buf or data:
    all_data = buf + buf2 + data
    print(f"\n=== 数据解析 ===")
    print(f"所有数据: {all_data.hex()}")
    for i in range(len(all_data) - 16):
        if all_data[i] == 0xAB and all_data[i+1] == 0xA0:
            d = all_data[i:i+17]
            nearest = (d[4] << 8) | d[5]
            strongest = (d[8] << 8) | d[9]
            print(f"帧头@{i}: nearest={nearest}cm={nearest/100.0:.2f}m strongest={strongest}cm={strongest/100.0:.2f}m")
            break

os.close(fd)
print("\n完成")
