#!/usr/bin/env python3
"""D30深度计原始串口诊断脚本 - 上传到RK3588运行"""
import termios, os, sys, struct, time, fcntl, select

TTY = "/dev/ttyS3"
SLAVE = 1

def modbus_crc(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

print("=== D30深度计诊断 ===")
print(f"打开 {TTY} ...")

try:
    tty = os.open(TTY, os.O_RDWR | os.O_NOCTTY)
except Exception as e:
    print(f"❌ 无法打开 {TTY}: {e}")
    sys.exit(1)

# 配置串口: 19200, 8N1, raw
attr = termios.tcgetattr(tty)
attr[4] = termios.B19200  # ispeed
attr[5] = termios.B19200  # ospeed
attr[2] = attr[2] & ~termios.CSTOPB   # 1 stop bit
attr[2] = attr[2] & ~termios.PARENB   # no parity
attr[2] = attr[2] & ~(termios.CSIZE)
attr[2] = attr[2] | termios.CS8        # 8 data bits
attr[3] = attr[3] & ~termios.ICANON    # raw mode
attr[3] = attr[3] & ~termios.ECHO
attr[6][termios.VMIN] = 0
attr[6][termios.VTIME] = 0
termios.tcsetattr(tty, termios.TCSANOW, attr)
termios.tcflush(tty, termios.TCIOFLUSH)

# 构造 MODBUS 读压力+温度指令
# 功能码 03, 寄存器 0x8939, 数量 4
cmd = bytes([SLAVE, 0x03, 0x89, 0x39, 0x00, 0x04])
crc = modbus_crc(cmd)
cmd_full = cmd + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
print(f"发送: {cmd_full.hex(' ')}")

for attempt in range(3):
    termios.tcflush(tty, termios.TCIOFLUSH)
    n = os.write(tty, cmd_full)
    print(f"\n--- 第{attempt+1}次 ---")
    
    # 等待响应 (最多1秒)
    time.sleep(0.1)
    buf = b""
    deadline = time.time() + 1.0
    while time.time() < deadline:
        fcntl.fcntl(tty, fcntl.F_SETFL, fcntl.fcntl(tty, fcntl.F_GETFL) | os.O_NONBLOCK)
        r, _, _ = select.select([tty], [], [], 0.1)
        if r:
            chunk = os.read(tty, 64)
            if chunk:
                buf += chunk
                if len(buf) >= 13:  # 完整响应帧
                    break
            else:
                break
    
    if len(buf) == 0:
        print("❌ 无响应 - 检查接线、供电、A/B线")
    else:
        print(f"收到 {len(buf)} 字节: {buf.hex(' ')}")
        
        if len(buf) >= 5:
            addr = buf[0]
            func = buf[1]
            byte_cnt = buf[2]
            print(f"  地址={addr} 功能码={func} 数据长度={byte_cnt}")
            
            if func == 0x03 and byte_cnt >= 8:
                # 压力 int32 (大端)
                p_raw = struct.unpack('>i', buf[3:7])[0]
                pressure = p_raw / 100.0  # 厘米水柱 -> 米
                # 温度 int32 (大端)
                t_raw = struct.unpack('>i', buf[7:11])[0]
                temp = t_raw / 1000.0
                print(f"  ✅ 压力={pressure:.2f}m  温度={temp:.2f}°C")
            elif func & 0x80:
                err = buf[2]
                print(f"  ❌ MODBUS异常 码={err}")
    
    time.sleep(0.3)

os.close(tty)
print("\n诊断完成")
