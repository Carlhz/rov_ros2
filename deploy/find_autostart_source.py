#!/usr/bin/env python3
"""查找 start_tronlong_3588.sh 的自启动来源并禁用"""
import paramiko, time

rk = {'hostname': '172.16.28.82', 'username': 'root', 'password': '159357'}

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(**rk, timeout=10)

# 1. 检查 root 的 crontab
stdin, stdout, stderr = ssh.exec_command('crontab -l 2>/dev/null; echo "---"; cat /etc/crontab 2>/dev/null | grep -v "^#" | grep -v "^$"')
stdout.channel.settimeout(10)
print('=== crontab ===')
print(stdout.read().decode())

# 2. 检查 profile.d
stdin, stdout, stderr = ssh.exec_command('grep -r "start_tronlong\|rov_3588" /etc/profile.d/ 2>/dev/null; echo "---"; grep -r "start_tronlong\|rov_3588" /root/.bashrc /root/.profile /root/.bash_profile 2>/dev/null')
stdout.channel.settimeout(10)
print('=== profile/bashrc ===')
print(stdout.read().decode())

# 3. 检查桌面自启动（root 和 Tronlong）
stdin, stdout, stderr = ssh.exec_command('find / -name "*.desktop" -exec grep -l "start_tronlong" {} \\; 2>/dev/null; echo "---"; find / -name "*.service" -exec grep -l "start_tronlong" {} \\; 2>/dev/null; echo "---"; grep -r "start_tronlong" /etc/ 2>/dev/null')
stdout.channel.settimeout(15)
print('=== desktop/service ===')
print(stdout.read().decode())

# 4. 查看 start_tronlong_3588.sh 的内容（关键部分）
stdin, stdout, stderr = ssh.exec_command('head -50 /root/Documents/start_tronlong_3588.sh 2>/dev/null')
stdout.channel.settimeout(10)
print('=== start_tronlong_3588.sh 内容 ===')
print(stdout.read().decode())

# 5. 检查 PID 2294 的启动方式 - 查看它的 environ
stdin, stdout, stderr = ssh.exec_command('cat /proc/2294/environ 2>/dev/null | tr "\\0" "\\n" | head -20; echo "---"; cat /proc/2294/cmdline 2>/dev/null | tr "\\0" " "; echo')
stdout.channel.settimeout(10)
print('=== PID 2294 environ ===')
print(stdout.read().decode())

# 6. 检查是否是桌面会话启动的
stdin, stdout, stderr = ssh.exec_command('loginctl list-sessions 2>/dev/null; echo "---"; systemctl list-units --type=scope | grep session')
stdout.channel.settimeout(10)
print('=== 登录会话 ===')
print(stdout.read().decode())

ssh.close()
