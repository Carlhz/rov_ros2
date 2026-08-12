#!/usr/bin/env python3
"""检查高度计 ROS2 话题数据"""
import paramiko, time

rk = {'hostname': '172.16.28.82', 'username': 'root', 'password': '159357'}

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(**rk, timeout=10)

# 等待更长时间让驱动稳定
print('等待5秒让驱动稳定...')
time.sleep(5)

# 查看完整日志
stdin, stdout, stderr = ssh.exec_command('cat /tmp/altimeter.log')
stdout.channel.settimeout(10)
print('=== 高度计完整日志 ===')
print(stdout.read().decode())

# 检查 ROS2 话题
cmd = 'source /opt/ros/setup.bash && export ROS_DOMAIN_ID=42 && timeout 5 ros2 topic echo /rov/altitude --once 2>&1'
stdin, stdout, stderr = ssh.exec_command(cmd)
stdout.channel.settimeout(10)
print('=== /rov/altitude 话题 ===')
print(stdout.read().decode())

# 检查所有 rov 话题
cmd = 'source /opt/ros/setup.bash && export ROS_DOMAIN_ID=42 && ros2 topic list 2>&1 | grep rov'
stdin, stdout, stderr = ssh.exec_command(cmd)
stdout.channel.settimeout(10)
print('=== /rov 话题列表 ===')
print(stdout.read().decode())

# 检查话题发布频率
cmd = 'source /opt/ros/setup.bash && export ROS_DOMAIN_ID=42 && timeout 5 ros2 topic hz /rov/altitude 2>&1'
stdin, stdout, stderr = ssh.exec_command(cmd)
stdout.channel.settimeout(10)
print('=== /rov/altitude 频率 ===')
print(stdout.read().decode())

ssh.close()
