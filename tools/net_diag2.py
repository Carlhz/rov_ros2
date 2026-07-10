#!/usr/bin/env python3
"""深度排查: 识别设备、UDP抓包、声纳驱动状态"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.28.82", username="root", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    return out + err

# 1. 探测 192.168.0.5 和 0.100 的 MAC + 主机名
print("=" * 60)
print("1. 探测在线设备的 MAC 地址")
print("=" * 60)
for ip in ["192.168.0.5", "192.168.0.7", "192.168.0.9", "192.168.0.100"]:
    # arping-like probe
    out = run(f"ping -c1 -W1 {ip} 2>&1; arp -n {ip} 2>/dev/null")
    print(f"--- {ip} ---")
    print(out)

# 2. 用 Python 做简单的 UDP 嗅探（因为没有tcpdump）
print("=" * 60)
print("2. Python UDP 嗅探 10 秒 (port 23, 8007, 8008)")
print("=" * 60)
sniff_script = '''
import socket, select, time
s = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_UDP)
s.settimeout(1)
start = time.time()
print(f"嗅探开始 @ {time.strftime('%H:%M:%S')}")
while time.time() - start < 10:
    try:
        data, addr = s.recvfrom(2048)
        ip_header = data[:20]
        src_ip = ".".join(str(b) for b in ip_header[12:16])
        dst_ip = ".".join(str(b) for b in ip_header[16:20])
        udp_header = data[20:28]
        src_port = (udp_header[0]<<8) + udp_header[1]
        dst_port = (udp_header[2]<<8) + udp_header[3]
        length = (udp_header[4]<<8) + udp_header[5]
        print(f"[{time.strftime('%H:%M:%S')}] {src_ip}:{src_port} -> {dst_ip}:{dst_port} len={length}")
    except socket.timeout:
        continue
    except Exception as e:
        print(f"ERR: {e}")
s.close()
print("嗅探结束")
'''
print(run(f"python3 -c '{sniff_script}' 2>&1 || python -c '{sniff_script}' 2>&1 || echo 'raw_socket_failed'", timeout=15))

# 3. 检查声纳驱动 systemd 服务
print("=" * 60)
print("3. 声纳相关服务 / ros2 进程")
print("=" * 60)
print(run("systemctl list-units --type=service --all 2>/dev/null | grep -i -E 'sonar|ros|rov' || echo 'no_systemd_services'"))
print(run("ps aux | grep -E 'ros|sonar|ins' | grep -v grep"))

# 4. 检查 netstat 监听端口
print("=" * 60)
print("4. UDP 监听端口")
print("=" * 60)
print(run("ss -ulnp 2>/dev/null || netstat -ulnp 2>/dev/null"))

# 5. 扫描 192.168.0.1-20 更仔细
print("=" * 60)
print("5. 仔细扫描 192.168.0.1-20")
print("=" * 60)
for i in range(1, 21):
    out = run(f"ping -c1 -W1 192.168.0.{i} 2>&1")
    if "1 received" in out or "ttl=" in out.lower():
        # get arp
        arp_out = run(f"arp -n 192.168.0.{i} 2>/dev/null")
        print(f"  ALIVE: 192.168.0.{i} -> {arp_out.strip()}")

ssh.close()
print("\n深度诊断完成")
