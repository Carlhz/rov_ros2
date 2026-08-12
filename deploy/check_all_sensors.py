#!/usr/bin/env python3
"""等待并检查高度计和深度计数据"""
import paramiko, time

rk = {'hostname': '172.16.28.82', 'username': 'root', 'password': '159357'}

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(**rk, timeout=10)

# 等15秒让数据积累
print('等待15秒...')
time.sleep(15)

# 高度计日志
stdin, stdout, stderr = ssh.exec_command('cat /tmp/altimeter.log')
stdout.channel.settimeout(10)
print('=== 高度计日志 ===')
print(stdout.read().decode())

# 深度计日志
stdin, stdout, stderr = ssh.exec_command('tail -5 /tmp/depth_sensor.log 2>/dev/null; tail -5 /tmp/depth_sensor_driver.log 2>/dev/null')
stdout.channel.settimeout(10)
print('=== 深度计日志 ===')
print(stdout.read().decode())

# 检查高度计话题数据
cmd = 'source /opt/ros/setup.bash && export ROS_DOMAIN_ID=42 && timeout 3 ros2 topic echo /rov/altitude 2>&1 | head -5'
stdin, stdout, stderr = ssh.exec_command(cmd)
stdout.channel.settimeout(10)
print('=== /rov/altitude 话题 ===')
print(stdout.read().decode())

# 检查深度话题
cmd = 'source /opt/ros/setup.bash && export ROS_DOMAIN_ID=42 && timeout 3 ros2 topic echo /rov/depth 2>&1 | head -5'
stdin, stdout, stderr = ssh.exec_command(cmd)
stdout.channel.settimeout(10)
print('=== /rov/depth 话题 ===')
print(stdout.read().decode())

# INS 话题（验证坐标系转换）
cmd = 'source /opt/ros/setup.bash && export ROS_DOMAIN_ID=42 && timeout 3 ros2 topic echo /ins/attitude 2>&1 | head -5'
stdin, stdout, stderr = ssh.exec_command(cmd)
stdout.channel.settimeout(10)
print('=== /ins/attitude 话题 ===')
print(stdout.read().decode())

# 进程总览
stdin, stdout, stderr = ssh.exec_command('ps aux | grep -E "python3.*rov|python3.*sensor|python3.*driver|python3.*motor|python3.*ins" | grep -v grep')
stdout.channel.settimeout(10)
print('=== Python 驱动进程 ===')
print(stdout.read().decode())

ssh.close()
