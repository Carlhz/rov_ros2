#!/usr/bin/env python3
"""部署 ins_driver_auto.py 到 RK3588 并重启"""
import paramiko, time

rk = {'hostname': '172.16.28.82', 'username': 'root', 'password': '159357'}
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(**rk, timeout=10)

# 1. 上传新驱动
local = r'D:\Carl_WorkStation\rov_ros2\rk3588\ins_driver_auto.py'
remote = '/opt/ros/rov_ros2_ws/ins_driver_auto.py'
sftp = ssh.open_sftp()
sftp.put(local, remote)
sftp.close()
print('[1] 上传 ins_driver_auto.py 完成')

# 2. 杀掉旧驱动进程
ssh.exec_command('pkill -9 -f ins_driver_auto.py')
time.sleep(1)

# 3. 启动新驱动
cmd = 'cd /opt/ros/rov_ros2_ws && source /opt/ros/setup.bash && export ROS_DOMAIN_ID=42 && nohup python3 ins_driver_auto.py > /tmp/ins_driver.log 2>&1 &'
ssh.exec_command(f'bash -c "{cmd}" &')
time.sleep(6)  # 给 ROS2 节点初始化留时间

# 4. 查日志确认启动成功
stdin, stdout, stderr = ssh.exec_command('tail -5 /tmp/ins_driver.log')
log = stdout.read().decode()
print('[2] 启动日志:')
print(log)

# 5. 检查进程
stdin, stdout, stderr = ssh.exec_command('pgrep -af ins_driver_auto.py')
print('[3] 进程:')
print(stdout.read().decode())

ssh.close()
