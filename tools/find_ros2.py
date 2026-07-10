#!/usr/bin/env python3
"""查找 ROS2 安装位置"""
import paramiko
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.28.82", username="root", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

print("=== /opt/ros/ 目录 ===")
print(run("ls /opt/ros/"))

print("=== setup.bash 位置 ===")
print(run("find /opt/ros -name setup.bash -maxdepth 4 2>/dev/null"))

print("=== ros2 命令 ===")
print(run("which ros2 2>/dev/null; find /opt/ros -name ros2 -type f 2>/dev/null | head -5"))

print("=== 当前 ROS 环境 ===")
print(run("printenv | grep -i ros 2>/dev/null; echo '---'; cat /etc/profile.d/*ros* 2>/dev/null | head -5; echo '---'; grep -r 'source.*ros' /root/.bashrc 2>/dev/null | head -5"))

ssh.close()
