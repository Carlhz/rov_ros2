#!/usr/bin/env python3
"""RK3588 网络诊断 — 排查声纳 IP 和子网发包情况"""
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
print("1. 网口配置")
print("=" * 60)
print(run("ip addr show"))

print("=" * 60)
print("2. ARP 表")
print("=" * 60)
print(run("arp -n"))

print("=" * 60)
print("3. 声纳驱动进程")
print("=" * 60)
print(run("ps aux | grep -i sonar | grep -v grep"))

print("=" * 60)
print("4. 驱动代码中的 IP 配置")
print("=" * 60)
print(run("grep -rn '192.168' /opt/ros/rov_ros2_ws/ --include='*.py' --include='*.cpp' --include='*.hpp' 2>/dev/null | head -30"))

print("=" * 60)
print("5. tcpdump 抓包 10 秒 (any interface, udp)")
print("=" * 60)
print(run("timeout 10 tcpdump -i any -n -l 'udp' 2>&1 || echo 'tcpdump_failed'", timeout=15))

print("=" * 60)
print("6. 子网扫描 (192.168.0.1-254)")
print("=" * 60)
scan_cmd = """
for i in $(seq 1 254); do
    (ping -c1 -W1 192.168.0.$i >/dev/null 2>&1 && echo "ALIVE: 192.168.0.$i") &
done
wait
echo "SCAN_DONE"
"""
print(run(scan_cmd, timeout=70))

ssh.close()
print("\n诊断完成")
