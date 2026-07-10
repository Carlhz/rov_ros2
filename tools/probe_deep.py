#!/usr/bin/env python3
"""深入探测 192.168.0.5 + UDP 嗅探"""
import paramiko, time

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.28.82", username="root", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    return out + err

# 1. 端口扫描 (nc 逐个试)
print("=" * 60)
print("1. 端口扫描 192.168.0.5 (TCP)")
print("=" * 60)
for port in [23, 80, 8080, 443, 8000, 8007, 8008, 5000, 5001, 502, 2000, 8001, 9999]:
    result = run(f"timeout 2 bash -c 'echo > /dev/tcp/192.168.0.5/{port}' 2>&1 && echo 'PORT {port} OPEN' || echo 'PORT {port} closed'", timeout=3)
    if "OPEN" in result:
        print(result.strip())

print()
print("=" * 60)
print("2. 端口扫描 192.168.0.5 (UDP)")
print("=" * 60)
for port in [23, 8007, 8008, 5000, 2000, 9999]:
    result = run(f"timeout 2 nc -u -v -w1 192.168.0.5 {port} < /dev/null 2>&1 || true", timeout=4)
    print(f"UDP {port}: {result.strip()[:100]}")

# 3. 用 Python raw socket 抓 5 秒
print()
print("=" * 60)
print("3. Python raw socket 嗅探 8 秒")
print("=" * 60)
sniff = '''
import socket, struct, time
s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(0x0003))
s.settimeout(1)
start = time.time()
count = 0
while time.time() - start < 8:
    try:
        data, _ = s.recvfrom(2048)
        # 只关注 192.168.0.x 来源
        src = ".".join(str(b) for b in data[26:30])
        dst = ".".join(str(b) for b in data[30:34])
        proto = data[23]
        if src.startswith("192.168.0.") or dst.startswith("192.168.0."):
            count += 1
            if proto == 17:  # UDP
                sp = (data[34]<<8)+data[35]; dp = (data[36]<<8)+data[37]
                print(f"UDP {src}:{sp} -> {dst}:{dp}")
            elif proto == 6:
                print(f"TCP {src} -> {dst}")
            else:
                print(f"PROTO{proto} {src} -> {dst}")
    except socket.timeout:
        continue
s.close()
print(f"总共捕获 {count} 个 192.168.0.x 相关包")
'''
print(run(f"python3 -c '{sniff}'", timeout=12))

# 4. 尝试 HTTP 请求
print()
print("=" * 60)
print("4. HTTP 请求 192.168.0.5:80")
print("=" * 60)
print(run("curl -s --connect-timeout 3 http://192.168.0.5/ 2>&1 | head -20 || echo 'no_http'"))

# 5. 也检查 192.168.0.100
print()
print("=" * 60)
print("5. 192.168.0.100 端口扫描")
print("=" * 60)
for port in [22, 23, 80, 443, 3389, 445, 139]:
    result = run(f"timeout 2 bash -c 'echo > /dev/tcp/192.168.0.100/{port}' 2>&1 && echo 'PORT {port} OPEN' || echo 'PORT {port} closed'", timeout=3)
    if "OPEN" in result:
        print(result.strip())

ssh.close()
