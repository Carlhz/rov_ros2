#!/usr/bin/env python3
"""发送正确格式的 FE/FD 命令 + 抓取 USR IOT 配置页"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.28.82", username="root", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    return out + err

# 1. 抓取 USR IOT 模块配置页面
print("=" * 60)
print("1. USR IOT 模块首页 (admin:admin)")
print("=" * 60)
index = run("curl -s --connect-timeout 3 -u 'admin:admin' http://192.168.0.5/ 2>&1 | head -100", timeout=5)
# 找标题和关键配置
for line in index.split('\n'):
    line = line.strip()
    if line and ('title' in line.lower() or '型号' in line or 'model' in line.lower() or 'USR' in line or 'IP' in line or 'MAC' in line):
        print("  " + line[:120])
    elif '<h' in line.lower():
        print("  " + line[:120])

# 抓完整的页面用于分析
print()
print("=" * 60)
print("2. 完整页面内容 (前3000字符)")
print("=" * 60)
full = run("curl -s --connect-timeout 3 -u 'admin:admin' http://192.168.0.5/ 2>&1 | head -c 3000", timeout=5)
print(full)

# 2. 发送正确格式的声纳命令
print()
print("=" * 60)
print("3. 发送正确 FE/FD 命令到 192.168.0.5:23")
print("=" * 60)
correct_cmd = '''
import socket, struct, time

def make_cmd(work_status=0x23, srange=4, gain=20, logf=40, absorption=10,
             sound_speed=1485, train_angle=0, sector=3600, data_len=1000,
             pulse_type=0, gate=200, min_range=150, delay_us=0, frequency=0):
    """对照 C++ 源码构造 28 字节命令帧"""
    buf = bytearray(28)
    buf[0]  = 0xFE          # FRAME_HEAD
    buf[1]  = 0x00          # Broadcast
    buf[2]  = work_status   # 0x23=RUN
    buf[3]  = srange        # 量程 4
    buf[4]  = gain          # 增益 20
    buf[5]  = logf          # 40
    buf[6]  = absorption    # 10
    buf[7]  = 0x01          # StepSize
    buf[8]  = (sound_speed >> 8) & 0xFF
    buf[9]  = sound_speed & 0xFF
    buf[10] = (train_angle >> 8) & 0xFF
    buf[11] = train_angle & 0xFF
    buf[12] = (sector >> 8) & 0xFF
    buf[13] = sector & 0xFF
    buf[14] = (data_len >> 8) & 0xFF
    buf[15] = data_len & 0xFF
    buf[16] = pulse_type
    buf[17] = 0x00          # Res1
    buf[18] = (gate >> 8) & 0xFF
    buf[19] = gate & 0xFF
    buf[20] = (min_range >> 8) & 0xFF
    buf[21] = min_range & 0xFF
    buf[22] = (delay_us >> 8) & 0xFF
    buf[23] = delay_us & 0xFF
    buf[24] = 0x00          # Res2
    buf[25] = 0x00          # Res3
    buf[26] = frequency
    buf[27] = 0xFD          # FRAME_TAIL
    return bytes(buf)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2)
s.bind(("0.0.0.0", 0))

# STOP
stop = make_cmd(work_status=0x2B)
print("STOP: " + stop.hex())
s.sendto(stop, ("192.168.0.5", 23))
time.sleep(0.3)

# RUN
run = make_cmd(work_status=0x23)
print("RUN:  " + run.hex())
s.sendto(run, ("192.168.0.5", 23))

for i in range(15):
    try:
        data, addr = s.recvfrom(4096)
        print("RECV " + str(addr) + ": " + str(len(data)) + " bytes, head=" + data[:16].hex())
    except socket.timeout:
        print("timeout " + str(i+1) + "/15")
        break
s.close()
'''
print(run("python3 << 'PYEOF'\n" + correct_cmd + "\nPYEOF", timeout=20))

ssh.close()
