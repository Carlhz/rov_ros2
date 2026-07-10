#!/usr/bin/env python3
"""探测 192.168.0.5 — 确认是否是声纳 + MAC 厂商查询"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.28.82", username="root", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    return out + err

print("=" * 60)
print("1. MAC 厂商查询")
print("=" * 60)
print("192.168.0.5  MAC: 9c:a5:25:f8:14:8d")
print(run("cat /usr/share/ieee-data/oui.txt 2>/dev/null | grep -i '9C-A5-25' || echo 'no_oui_db'"))

print("=" * 60)
print("2. nmap 扫描 192.168.0.5 的开放端口")
print("=" * 60)
print(run("nmap -sU -p 23,8007,8008,8080,80,443 192.168.0.5 2>&1 || echo 'nmap_not_installed'", timeout=30))

print("=" * 60)
print("3. 用 netcat 尝试连接 192.168.0.5:23")
print("=" * 60)
print(run("echo 'test' | timeout 3 nc -u -w2 192.168.0.5 23 2>&1; echo 'exit_code='$?", timeout=10))

print("=" * 60)
print("4. 用 Python 尝试 192.168.0.5 UDP 23 收发")
print("=" * 60)
probe_script = '''
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(2)
s.bind(("0.0.0.0", 0))

# 尝试发送查询命令 (Oculus 声纳可能是这种握手)
test_cmds = [
    bytes([0xAA, 0xA0, 0x01, 0x00, 0x00, 0xAB]),  # 可能的查询命令
    bytes([0x5A, 0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55]),  # INS 风格
    b"HELLO",
    b"",
]
for i, cmd in enumerate(test_cmds):
    try:
        s.sendto(cmd, ("192.168.0.5", 23))
        data, addr = s.recvfrom(4096)
        print(f"CMD{i}: 收到 {addr} 响应: {data.hex()}")
    except socket.timeout:
        print(f"CMD{i}: 无响应")
    except Exception as e:
        print(f"CMD{i}: {e}")
s.close()
'''
print(run(f"python3 << 'PYEOF'\n{probe_script}\nPYEOF", timeout=15))

# 5. 尝试用 tcpdump 替代 — 检查有没有已接收的包
print("=" * 60)
print("5. 网口统计")
print("=" * 60)
print(run("cat /proc/net/dev"))
print(run("ip -s link show eth0"))

# 6. 检查是否安装有 scapy/其他抓包工具
print("=" * 60)
print("6. 检查可用工具")
print("=" * 60)
print(run("which tcpdump tshark ngrep nmap nc 2>&1; dpkg -l | grep -E 'tcpdump|nmap|netcat' 2>/dev/null | head -5"))

ssh.close()
