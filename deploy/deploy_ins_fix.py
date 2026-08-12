#!/usr/bin/env python3
"""部署修改后的 ins_driver_auto.py 到 RK3588 并重启"""
import paramiko
import time

rk = {'hostname': '172.16.28.82', 'username': 'root', 'password': '159357'}
LOCAL = r'D:\Carl_WorkStation\rov_ros2\rk3588\ins_driver_auto.py'
REMOTE = '/opt/ros/rov_ros2_ws/ins_driver_auto.py'

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(**rk, timeout=10)

# 1. 上传文件
sftp = ssh.open_sftp()
sftp.put(LOCAL, REMOTE)
sftp.close()
print(f'[1] 上传完成: ins_driver_auto.py')

# 2. 杀掉旧进程
stdin, stdout, stderr = ssh.exec_command('pkill -9 -f ins_driver_auto.py; sleep 1; pgrep -af ins_driver_auto.py || echo "OK 进程已停止"')
out = stdout.read().decode()
print(f'[2] 停止旧进程: {out.strip()}')

# 3. 重启 INS 驱动
cmd = '''cd /opt/ros/rov_ros2_ws && bash -c "source /opt/ros/setup.bash && export ROS_DOMAIN_ID=42 && export ROS_LOCALHOST_ONLY=0 && nohup python3 ins_driver_auto.py > /tmp/ins_driver.log 2>&1 &"'''
stdin, stdout, stderr = ssh.exec_command(cmd)
time.sleep(4)

# 4. 查看进程和初始日志
stdin, stdout, stderr = ssh.exec_command('pgrep -af ins_driver_auto.py && echo "---" && tail -30 /tmp/ins_driver.log')
print('=== 重启后状态 ===')
print(stdout.read().decode())

ssh.close()
print('=== 完成 ===')