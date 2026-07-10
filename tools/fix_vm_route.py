#!/usr/bin/env python3
"""Fix VM route to 192.168.0.x subnet (lost after reboot)."""

import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.30.0", username="carl", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

print("=" * 60)
print("Step 1: 添加临时路由")
print("=" * 60)
# Route 192.168.0.0/24 via RK3588 (172.16.28.82)
result = run("sudo ip route add 192.168.0.0/24 via 172.16.28.82 2>&1")
print(result)

print("验证路由:")
print(run("ip route | grep 192.168"))

print()
print("=" * 60)
print("Step 2: 测试 ros2 topic echo 数据流")
print("=" * 60)
echo_result = run(
    'bash -c "source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=0; timeout 4 ros2 topic echo /sonar/omni/original 2>&1 | head -30"',
    timeout=15
)
print(echo_result)

print()
print("=" * 60)
print("Step 3: 运行 quick_view 监控")
print("=" * 60)
qv_result = run(
    'bash -c "source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=0; timeout 5 python3 ~/rov_ros2_ws/monitor/sonar_quick_view.py 2>&1 | tail -10"',
    timeout=15
)
print(qv_result)

print()
print("=" * 60)
print("Step 4: 持久化路由 (netplan)")
print("=" * 60)
# Add persistent route
netplan_cmd = """
cat > /tmp/01-netcfg.yaml << 'NETPLAN'
network:
  version: 2
  renderer: networkd
  ethernets:
    ens33:
      dhcp4: true
      routes:
        - to: 192.168.0.0/24
          via: 172.16.28.82
NETPLAN
sudo cp /tmp/01-netcfg.yaml /etc/netplan/01-network-manager-all.yaml
sudo netplan apply 2>&1
echo "netplan applied"
"""
print(run(netplan_cmd, timeout=10))

print("验证持久化:")
print(run("cat /etc/netplan/01-network-manager-all.yaml"))

ssh.close()
print("\nDone.")
