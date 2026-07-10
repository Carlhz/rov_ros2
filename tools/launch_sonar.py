#!/usr/bin/env python3
"""正确启动声纳驱动"""
import paramiko, time
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.28.82", username="root", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

# 清理
run("pkill -f sonar_omni_driver 2>/dev/null; sleep 1; echo ok")

# 启动 - 用脚本文件方式
script = """#!/bin/bash
source /opt/ros/setup.bash
source /opt/ros/rov_ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
echo "ROS2: $(which ros2)"
exec ros2 launch rov_sonar_driver sonar_omni.launch.py server_ip:=192.168.0.5
"""
# 写入脚本到 RK3588
sftp = ssh.open_sftp()
f = sftp.file("/tmp/start_sonar.sh", "w")
f.write(script)
f.close()
sftp.close()
run("chmod +x /tmp/start_sonar.sh")

# 启动
print("=== 启动驱动 ===")
print(run("nohup /tmp/start_sonar.sh > /tmp/sonar_omni.log 2>&1 & sleep 5; echo done"))

print()
print("=== 驱动日志 ===")
print(run("cat /tmp/sonar_omni.log | tail -30"))

print()
print("=== 进程检查 ===")
print(run("ps aux | grep sonar_omni | grep -v grep"))

print()
print("=== 声纳话题 ===")
topic_check = """bash -c '
source /opt/ros/setup.bash
source /opt/ros/rov_ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
ros2 topic list 2>/dev/null | grep -i sonar
'"""
print(run(topic_check, timeout=10))

ssh.close()
