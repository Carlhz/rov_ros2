#!/usr/bin/env python3
"""Diagnose why VM monitor shows offline after reboot."""

import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.30.0", username="carl", password="159357", timeout=10)

def run(cmd, timeout=10):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

print("=" * 60)
print("1. 网络路由 - 192.168.0.x")
print("=" * 60)
print(run("ip route | grep 192.168"))
print(run("ip addr show | grep -E 'inet |eth|ens'"))

print("=" * 60)
print("2. Ping 192.168.0.5 (声纳) 和 192.168.0.99 (RK3588)")
print("=" * 60)
print(run("ping -c 2 -W 2 192.168.0.5 2>&1"))
print(run("ping -c 2 -W 2 192.168.0.99 2>&1"))

print("=" * 60)
print("3. ROS2 话题列表 (domain=0)")
print("=" * 60)
print(run('bash -c "source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=0; ros2 topic list 2>&1"'))

print("=" * 60)
print("4. ros2 topic echo --once /sonar/omni/original")
print("=" * 60)
print(run('bash -c "source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=0; timeout 5 ros2 topic echo --once /sonar/omni/original 2>&1"', timeout=15))

print("=" * 60)
print("5. start_monitor.sh 配置")
print("=" * 60)
print(run("head -15 ~/rov_ros2_ws/monitor/start_monitor.sh"))
print("Python:", run("which python3; python3 --version"))

print("=" * 60)
print("6. 检查 ROS_LOCALHOST_ONLY 和 multicast")
print("=" * 60)
print(run("printenv | grep -i ros; printenv | grep -i dds"))
print("CycloneDDS config:", run("cat /etc/cyclonedds/cyclonedds.xml 2>/dev/null || echo 'no global config'; cat ~/.ros/cyclonedds.xml 2>/dev/null || echo 'no user config'"))

print("=" * 60)
print("7. 检查 ROS2 能否从网络接口发现")
print("=" * 60)
print(run('bash -c "source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=0; ros2 node list 2>&1; ros2 topic info /sonar/omni/original 2>&1"'))

ssh.close()
