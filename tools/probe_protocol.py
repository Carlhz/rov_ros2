#!/usr/bin/env python3
"""发送声纳和 INS 协议命令到 192.168.0.5，验证设备身份"""
import paramiko, time

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.28.82", username="root", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    return out + err

# 1. 发送声纳 FE/FD 命令到 192.168.0.5:23
print("=" * 60)
print("1. 声纳协议测试: 192.168.0.5:23 (FE/FD 帧)")
print("=" * 60)
sonar_test = '''
import socket, struct, time

def make_cmd(status=0x23, srange=4, gain=20, logf=40, absorption=10,
             sound_speed=1485, train_angle=0, sector=3600, data_len=1000,
             pulse_type=0, gate=200, min_range=150, delay_us=0, frequency=0):
    """构造 28 字节 FE/FD 命令帧"""
    buf = bytearray(28)
    buf[0] = 0xFE          # 帧头
    buf[1] = status        # 工作状态
    buf[2] = srange        # 量程
    buf[3] = gain          # 增益
    buf[4] = logf & 0xFF
    buf[5] = (logf >> 8) & 0xFF
    buf[6] = absorption & 0xFF
    buf[7] = (absorption >> 8) & 0xFF
    buf[8] = sound_speed & 0xFF
    buf[9] = (sound_speed >> 8) & 0xFF
    buf[10] = train_angle & 0xFF
    buf[11] = (train_angle >> 8) & 0xFF
    buf[12] = sector & 0xFF
    buf[13] = (sector >> 8) & 0xFF
    buf[14] = data_len & 0xFF
    buf[15] = (data_len >> 8) & 0xFF
    buf[16] = pulse_type
    buf[17] = gate & 0xFF
    buf[18] = (gate >> 8) & 0xFF
    buf[19] = min_range & 0xFF
    buf[20] = (min_range >> 8) & 0xFF
    buf[21] = delay_us & 0xFF
    buf[22] = (delay_us >> 8) & 0xFF
    buf[23] = frequency & 0xFF
    buf[24] = (frequency >> 8) & 0xFF
    buf[25] = 0x00
    buf[26] = 0x00
    # checksum: sum of bytes 0-25 mod 256
    buf[27] = sum(buf[:27]) % 256
    # set FD tail
    buf[27] = 0xFD  # 实际上帧尾是 0xFD，前面已经用 checksum 算好了
    return bytes(buf)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(3)
s.bind(("0.0.0.0", 0))

# 先发停止命令
stop_cmd = make_cmd(status=0x2B)
print(f"发送 STOP 命令: {stop_cmd.hex()}")
s.sendto(stop_cmd, ("192.168.0.5", 23))
time.sleep(0.5)

# 发送运行命令
run_cmd = make_cmd(status=0x23, sector=3600)
print(f"发送 RUN 命令: {run_cmd.hex()}")
s.sendto(run_cmd, ("192.168.0.5", 23))

# 等待响应
for i in range(10):
    try:
        data, addr = s.recvfrom(4096)
        print(f"收到 {addr}: {len(data)} bytes, 头部={data[:8].hex()}")
    except socket.timeout:
        print(f"超时 {i+1}/10")
        break
s.close()
'''
print(run(f"python3 << 'PYEOF'\n{sonar_test}\nPYEOF", timeout=15))

# 2. 发送 INS 协议命令到 192.168.0.5:8007
print()
print("=" * 60)
print("2. INS 协议测试: 192.168.0.5:8007 (5A A5 帧)")
print("=" * 60)
ins_test = '''
import socket, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2)
s.bind(("0.0.0.0", 0))

# INS 停止
stop = bytes([0x5A, 0xA5, 0x47, 0x00, 0x01, 0x00, 0x00, 0x46, 0x55])
print(f"发送 STOP: {stop.hex()}")
s.sendto(stop, ("192.168.0.5", 8007))
time.sleep(0.3)

# INS 启动
start = bytes([0x5A, 0xA5, 0x47, 0x01, 0x01, 0x00, 0x00, 0x47, 0x55])
print(f"发送 START: {start.hex()}")
s.sendto(start, ("192.168.0.5", 8007))

for i in range(5):
    try:
        data, addr = s.recvfrom(4096)
        print(f"收到 {addr}: {len(data)} bytes, {data[:16].hex()}")
    except socket.timeout:
        print(f"超时 {i+1}/5")
        break
s.close()
'''
print(run(f"python3 << 'PYEOF'\n{ins_test}\nPYEOF", timeout=15))

# 3. 也试试 192.168.0.5:8008 (INS 数据端口)
print()
print("=" * 60)
print("3. 直接监听 192.168.0.5:8008 (INS数据端口)")
print("=" * 60)
ins_listen = '''
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(3)
s.bind(("0.0.0.0", 8008))
print("监听 UDP 8008...")
for i in range(5):
    try:
        data, addr = s.recvfrom(4096)
        if str(addr[0]).startswith("192.168.0."):
            print(f"收到 {addr}: {len(data)} bytes, {data[:20].hex()}")
    except socket.timeout:
        print(f"超时 {i+1}/5")
        break
s.close()
'''
print(run(f"python3 << 'PYEOF'\n{ins_listen}\nPYEOF", timeout=18))

ssh.close()
