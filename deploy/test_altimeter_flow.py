#!/usr/bin/env python3
"""在 RK3588 上模拟 altimeter_driver 的精确流程，对比测试。
直接运行（非 ROS2 节点），排除 ROS2 干扰。"""
import os, sys, time, struct, termios, fcntl, signal

PORT = '/dev/ttyS3'
BAUD = 9600
DEVICE_ID = 1
FRAME_LEN = 17
READ_BUF = FRAME_LEN + 6

BAUDS = {9600: termios.B9600, 19200: termios.B19200,
         38400: termios.B38400, 57600: termios.B57600,
         115200: termios.B115200}

def build_command(dev_id):
    cmd = bytearray([0xAA, 0xA0, dev_id, 0x00, 0x00])
    checksum = cmd[0] ^ cmd[1] ^ cmd[2] ^ cmd[3] ^ cmd[4]
    cmd.append(checksum)
    return bytes(cmd)

def open_serial(port, baudrate, timeout=1.5):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    attr = termios.tcgetattr(fd)
    attr[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
                 | termios.ISTRIP | termios.INLCR | termios.IGNCR
                 | termios.ICRNL | termios.IXON)
    attr[1] &= ~termios.OPOST
    attr[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
    attr[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    attr[3] &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)
    bconst = BAUDS.get(baudrate, termios.B9600)
    attr[4] = bconst
    attr[5] = bconst
    attr[6][termios.VMIN] = 1
    attr[6][termios.VTIME] = int(timeout * 10) if timeout else 0
    termios.tcsetattr(fd, termios.TCSANOW, attr)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd

cmd = build_command(DEVICE_ID)
print(f"命令: {cmd.hex()} ({len(cmd)} bytes)")

# === 方式A: 精确模拟驱动流程 (sleep 0.3 + VMIN=1 read) ===
print("\n=== 方式A: 模拟驱动 poll() 流程 ===")
fd = open_serial(PORT, BAUD, 1.5)
print(f"fd={fd}, 串口已配置")

# 模拟 poll() 的流程
termios.tcflush(fd, termios.TCIFLUSH)  # reset_input_buffer
n = os.write(fd, cmd)                   # write
print(f"write 返回 {n}")
termios.tcdrain(fd)                     # flush
print("tcdrain 完成")
time.sleep(0.3)                         # sleep 0.3
print(f"sleep 0.3 完成, 准备 read...")

# read with VMIN=1 + signal.alarm (跟驱动一样)
attr = termios.tcgetattr(fd)
attr[6][termios.VMIN] = 1
attr[6][termios.VTIME] = 15
termios.tcsetattr(fd, termios.TCSANOW, attr)

buf = b''
def _alarm_handler(signum, frame):
    raise TimeoutError('timeout')
old_handler = signal.signal(signal.SIGALRM, _alarm_handler)
signal.setitimer(signal.ITIMER_REAL, 1.5)
t0 = time.time()
try:
    while len(buf) < READ_BUF:
        chunk = os.read(fd, READ_BUF - len(buf))
        if not chunk:
            break
        buf += chunk
        print(f"  收到 {len(chunk)} bytes @ {time.time()-t0:.3f}s: {chunk.hex()}")
except (TimeoutError, OSError) as e:
    print(f"  异常: {e} @ {time.time()-t0:.3f}s")
finally:
    signal.setitimer(signal.ITIMER_REAL, 0)
    signal.signal(signal.SIGALRM, old_handler)
print(f"方式A 结果: {len(buf)} bytes: {buf.hex() if buf else '(空)'}")
os.close(fd)

# === 方式B: 跟测试脚本完全一样 (无 sleep, 直接 VMIN=1 read) ===
print("\n=== 方式B: 无 sleep, 直接 VMIN=1 read (测试脚本方式) ===")
fd = open_serial(PORT, BAUD, 2.0)
attr = termios.tcgetattr(fd)
attr[6][termios.VMIN] = 1
attr[6][termios.VTIME] = 20
termios.tcsetattr(fd, termios.TCSANOW, attr)
termios.tcflush(fd, termios.TCIFLUSH)
os.write(fd, cmd)
termios.tcdrain(fd)
print("write+drain 完成, 直接 read...")

t0 = time.time()
try:
    data = os.read(fd, 64)
    print(f"收到 {len(data)} bytes @ {time.time()-t0:.3f}s: {data.hex() if data else '(空)'}")
except OSError as e:
    print(f"读取错误: {e}")
os.close(fd)

# === 方式C: 无 sleep, 无 signal, 纯 VMIN=1 阻塞 ===
print("\n=== 方式C: 无 sleep, 无 signal, 纯 VMIN=1 VTIME=30 ===")
fd = open_serial(PORT, BAUD, 3.0)
attr = termios.tcgetattr(fd)
attr[6][termios.VMIN] = 1
attr[6][termios.VTIME] = 30  # 3s inter-char timeout
termios.tcsetattr(fd, termios.TCSANOW, attr)
termios.tcflush(fd, termios.TCIFLUSH)
os.write(fd, cmd)
termios.tcdrain(fd)
print("write+drain 完成, 纯阻塞 read (最多3s)...")

t0 = time.time()
try:
    data = os.read(fd, 64)
    print(f"收到 {len(data)} bytes @ {time.time()-t0:.3f}s: {data.hex() if data else '(空)'}")
except OSError as e:
    print(f"读取错误: {e}")
os.close(fd)

# === 方式D: sleep 0.3 后, 无 signal, 纯 VMIN=1 阻塞 ===
print("\n=== 方式D: sleep 0.3 + 纯 VMIN=1 VTIME=30 (无 signal) ===")
fd = open_serial(PORT, BAUD, 3.0)
attr = termios.tcgetattr(fd)
attr[6][termios.VMIN] = 1
attr[6][termios.VTIME] = 30
termios.tcsetattr(fd, termios.TCSANOW, attr)
termios.tcflush(fd, termios.TCIFLUSH)
os.write(fd, cmd)
termios.tcdrain(fd)
time.sleep(0.3)
print("sleep 0.3 完成, 纯阻塞 read...")

t0 = time.time()
try:
    data = os.read(fd, 64)
    print(f"收到 {len(data)} bytes @ {time.time()-t0:.3f}s: {data.hex() if data else '(空)'}")
except OSError as e:
    print(f"读取错误: {e}")
os.close(fd)

print("\n=== 完成 ===")
