#!/usr/bin/env python3
"""检查深度计 + 查找 rov_3588_node 启动来源"""
import paramiko, time

rk = {'hostname': '172.16.28.82', 'username': 'root', 'password': '159357'}

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(**rk, timeout=10)

# 1. 深度计话题
cmd = 'source /opt/ros/setup.bash && export ROS_DOMAIN_ID=42 && timeout 3 ros2 topic echo /rov/depth 2>&1 | head -5'
stdin, stdout, stderr = ssh.exec_command(cmd)
stdout.channel.settimeout(10)
print('=== /rov/depth ===')
print(stdout.read().decode())

# 2. 深度计日志
stdin, stdout, stderr = ssh.exec_command('tail -5 /tmp/depth_sensor.log 2>/dev/null; tail -5 /tmp/depth_sensor_driver.log 2>/dev/null')
stdout.channel.settimeout(10)
print('=== 深度计日志 ===')
print(stdout.read().decode())

# 3. 检查 ttyS5 占用
stdin, stdout, stderr = ssh.exec_command('fuser /dev/ttyS5 2>&1; ps aux | grep -E "depth_sensor" | grep -v grep')
stdout.channel.settimeout(10)
print('=== ttyS5 / 深度计进程 ===')
print(stdout.read().decode())

# 4. 查找 rov_3588_node 的启动来源
stdin, stdout, stderr = ssh.exec_command('systemctl list-units --all | grep -i -E "rov|tronlong|3588"; echo "---"; ls /etc/systemd/system/ | grep -i -E "rov|tronlong|3588"; echo "---"; grep -r "rov_3588" /etc/systemd/system/ 2>/dev/null; echo "---"; grep -r "rov_3588" /etc/rc.local 2>/dev/null; echo "---"; crontab -l 2>/dev/null')
stdout.channel.settimeout(10)
print('=== rov_3588_node 启动来源 ===')
print(stdout.read().decode())

# 5. 检查 rov_3588_node 是否还在运行
stdin, stdout, stderr = ssh.exec_command('ps aux | grep rov_3588 | grep -v grep')
stdout.channel.settimeout(10)
print('=== rov_3588_node 进程 ===')
print(stdout.read().decode())

ssh.close()
