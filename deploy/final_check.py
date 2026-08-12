#!/usr/bin/env python3
"""最终验证所有传感器状态"""
import paramiko, time

rk = {'hostname': '172.16.28.82', 'username': 'root', 'password': '159357'}

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(**rk, timeout=10)

# 高度计
stdin, stdout, stderr = ssh.exec_command('tail -5 /tmp/altimeter.log 2>/dev/null')
stdout.channel.settimeout(10)
print('=== 高度计日志 (最近5行) ===')
print(stdout.read().decode())

# 深度计
stdin, stdout, stderr = ssh.exec_command('tail -3 /tmp/depth_sensor.log 2>/dev/null')
stdout.channel.settimeout(10)
print('=== 深度计日志 (最近3行) ===')
print(stdout.read().decode())

# INS
stdin, stdout, stderr = ssh.exec_command('tail -3 /tmp/ins_driver.log 2>/dev/null')
stdout.channel.settimeout(10)
print('=== INS 日志 (最近3行) ===')
print(stdout.read().decode())

# DVL
stdin, stdout, stderr = ssh.exec_command('tail -3 /tmp/dvl_driver.log 2>/dev/null')
stdout.channel.settimeout(10)
print('=== DVL 日志 (最近3行) ===')
print(stdout.read().decode())

# 确认无 TL3588 进程
stdin, stdout, stderr = ssh.exec_command('ps aux | grep -E "rov_3588|rov_light|depth_sensor_driver_node|start_tronlong" | grep -v grep')
stdout.channel.settimeout(10)
tl_procs = stdout.read().decode().strip()
print('=== TL3588 进程检查 ===')
print(tl_procs if tl_procs else '无 TL3588 进程 (OK)')

# 端口占用
stdin, stdout, stderr = ssh.exec_command('echo "ttyS3:"; fuser /dev/ttyS3 2>&1; echo "ttyS5:"; fuser /dev/ttyS5 2>&1')
stdout.channel.settimeout(10)
print('=== 端口占用 ===')
print(stdout.read().decode())

ssh.close()
