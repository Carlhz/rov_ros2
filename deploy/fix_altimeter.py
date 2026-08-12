#!/usr/bin/env python3
"""快速部署高度计驱动到 RK3588 并重启"""
import paramiko, time

rk = {'hostname': '172.16.28.82', 'username': 'root', 'password': '159357'}
local = r'D:\Carl_WorkStation\rov_ros2\sensors\altimeter_driver.py'
remote = '/opt/ros/rov_ros2_ws/sensors/altimeter_driver.py'

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(**rk, timeout=10)

# 上传文件
sftp = ssh.open_sftp()
sftp.put(local, remote)
sftp.close()
print('上传完成: altimeter_driver.py')

# 杀掉旧进程
stdin, stdout, stderr = ssh.exec_command('pkill -f altimeter_driver.py; sleep 1; echo killed')
print(stdout.read().decode())

# 重启驱动
cmd = 'cd /opt/ros/rov_ros2_ws && source /opt/ros/setup.bash && export ROS_DOMAIN_ID=42 && nohup python3 sensors/altimeter_driver.py > /tmp/altimeter.log 2>&1 &'
ssh.exec_command(cmd)
time.sleep(5)

# 查看日志
stdin, stdout, stderr = ssh.exec_command('cat /tmp/altimeter.log')
print('=== 高度计日志 ===')
print(stdout.read().decode())

# 检查进程
stdin, stdout, stderr = ssh.exec_command('ps aux | grep altimeter | grep -v grep')
print('=== 进程状态 ===')
print(stdout.read().decode())

ssh.close()
print('完成')
