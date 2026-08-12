#!/usr/bin/env python3
"""查找 TL3588 SDK 节点的自动启动来源"""
import paramiko, time

rk = {'hostname': '172.16.28.82', 'username': 'root', 'password': '159357'}

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(**rk, timeout=10)

# 1. 检查所有 systemd service 是否有 Restart=always
stdin, stdout, stderr = ssh.exec_command('grep -r "Restart" /etc/systemd/system/rov* 2>/dev/null; echo "---"; systemctl cat rov-sensors.service 2>&1')
stdout.channel.settimeout(10)
print('=== rov-sensors.service 内容 ===')
print(stdout.read().decode())

# 2. 检查桌面自启动
stdin, stdout, stderr = ssh.exec_command('ls -la /home/Tronlong/.config/autostart/ 2>/dev/null; echo "---"; ls -la /etc/xdg/autostart/ 2>/dev/null | grep -i rov; echo "---"; find /home/Tronlong -name "*.desktop" -exec grep -l "rov_3588\|rov_3588_node\|depth_sensor_driver" {} \\; 2>/dev/null')
stdout.channel.settimeout(10)
print('=== 桌面自启动 ===')
print(stdout.read().decode())

# 3. 检查 Tronlong 用户的启动脚本
stdin, stdout, stderr = ssh.exec_command('cat /home/Tronlong/.bashrc 2>/dev/null | grep -i "rov_3588\|ros_tronlong\|depth_sensor" ; echo "---"; cat /home/Tronlong/.profile 2>/dev/null | grep -i "rov_3588\|ros_tronlong\|depth_sensor"; echo "---"; cat /home/Tronlong/.bash_profile 2>/dev/null | grep -i "rov_3588\|ros_tronlong\|depth_sensor"')
stdout.channel.settimeout(10)
print('=== 用户 bashrc/profile ===')
print(stdout.read().decode())

# 4. 查找所有包含 rov_3588 的脚本
stdin, stdout, stderr = ssh.exec_command('grep -r "rov_3588_node\|rov_3588_app" /etc/ /opt/ /home/ 2>/dev/null | grep -v ".pyc\|Binary\|__pycache__" | head -20')
stdout.channel.settimeout(15)
print('=== 包含 rov_3588 的文件 ===')
print(stdout.read().decode())

# 5. 检查 PID 21616 的父进程
stdin, stdout, stderr = ssh.exec_command('ps -o ppid= -p 21616 2>/dev/null; echo "---"; ps -o ppid= -p 21574 2>/dev/null; echo "---"; ps -o pid,ppid,cmd -p $(ps -o ppid= -p 21616 2>/dev/null) 2>/dev/null')
stdout.channel.settimeout(10)
print('=== 父进程追踪 ===')
print(stdout.read().decode())

# 6. 检查 rc.local
stdin, stdout, stderr = ssh.exec_command('cat /etc/rc.local 2>/dev/null | grep -v "^#" | grep -v "^$"')
stdout.channel.settimeout(10)
print('=== rc.local ===')
print(stdout.read().decode())

# 7. 检查所有 systemd 服务中包含 rov 的
stdin, stdout, stderr = ssh.exec_command('find /etc/systemd/system/ -name "*.service" -exec grep -l "rov_3588\|ros_tronlong\|depth_sensor_driver_node" {} \\; 2>/dev/null')
stdout.channel.settimeout(10)
print('=== 相关 systemd 服务文件 ===')
print(stdout.read().decode())

ssh.close()
