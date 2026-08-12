#!/usr/bin/env python3
"""查找并杀掉占用 ttyS3 的进程"""
import paramiko, time

rk = {'hostname': '172.16.28.82', 'username': 'root', 'password': '159357'}

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(**rk, timeout=10)

# 查看占用 ttyS3 的进程详情
stdin, stdout, stderr = ssh.exec_command('ps aux | head -1; ps aux | grep -E "12169|ttyS3|altimeter|rov_sensors|depth_sensor" | grep -v grep')
stdout.channel.settimeout(10)
print('=== 占用 ttyS3 的进程 ===')
print(stdout.read().decode())

# 查看 rov-sensors.service 状态
stdin, stdout, stderr = ssh.exec_command('systemctl is-active rov-sensors.service 2>&1; systemctl is-enabled rov-sensors.service 2>&1')
stdout.channel.settimeout(10)
print('=== rov-sensors.service ===')
print(stdout.read().decode())

# 查看所有 python3 进程
stdin, stdout, stderr = ssh.exec_command('ps aux | grep python3 | grep -v grep')
stdout.channel.settimeout(10)
print('=== Python3 进程 ===')
print(stdout.read().decode())

# 查看 start_all.sh 启动的进程
stdin, stdout, stderr = ssh.exec_command('ps aux | grep -E "start_all|rov_ros2" | grep -v grep')
stdout.channel.settimeout(10)
print('=== ROV 相关进程 ===')
print(stdout.read().decode())

# 查看进程 12169 的完整命令行
stdin, stdout, stderr = ssh.exec_command('cat /proc/12169/cmdline 2>/dev/null | tr "\\0" " "; echo; cat /proc/12169/comm 2>/dev/null')
stdout.channel.settimeout(10)
print('=== PID 12169 详情 ===')
print(stdout.read().decode())

ssh.close()
