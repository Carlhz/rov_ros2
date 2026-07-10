#!/usr/bin/env python3
"""Fix start_monitor.sh on VM: replace hardcoded ROS_DOMAIN_ID=42 with 0."""

import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.30.0", username="carl", password="159357", timeout=10)

def run(cmd, timeout=10):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

# Replace ALL hardcoded ROS_DOMAIN_ID=42 with 0 in the script
print("=== 修复前 domain 设置 ===")
print(run("grep -n 'ROS_DOMAIN_ID' ~/rov_ros2_ws/monitor/start_monitor.sh"))

# Fix all occurrences
run("sed -i 's/export ROS_DOMAIN_ID=42/export ROS_DOMAIN_ID=0  # sonar domain/g' ~/rov_ros2_ws/monitor/start_monitor.sh")

print()
print("=== 修复后 domain 设置 ===")
print(run("grep -n 'ROS_DOMAIN_ID' ~/rov_ros2_ws/monitor/start_monitor.sh"))

print()
print("=== 现在手动测试 monitor (domain=0) ===")
print(run(
    'bash -c "source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=0; timeout 4 python3 ~/rov_ros2_ws/monitor/sonar_monitor.py 2>&1" | grep -E "在线|离线|刷新|点数|FPS" | head -5',
    timeout=15
))

ssh.close()
print("\nDone. VM start_monitor.sh 已修复。")
