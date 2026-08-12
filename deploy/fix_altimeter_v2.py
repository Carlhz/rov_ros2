#!/usr/bin/env python3
"""杀掉 TL3588 SDK 冲突进程，重启高度计驱动"""
import paramiko, time

rk = {'hostname': '172.16.28.82', 'username': 'root', 'password': '159357'}

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(**rk, timeout=10)

# 1. 查找是什么启动了 rov_3588_node
stdin, stdout, stderr = ssh.exec_command('systemctl list-units --type=service --state=running | grep -i -E "rov|tronlong|sensor|3588"')
stdout.channel.settimeout(10)
print('=== 相关 systemd 服务 ===')
print(stdout.read().decode())

# 2. 查找 rov_3588_node 的启动方式
stdin, stdout, stderr = ssh.exec_command('cat /proc/12169/cmdline 2>/dev/null | tr "\\0" " "; echo; ls -la /proc/12169/cwd 2>/dev/null; cat /proc/12169/environ 2>/dev/null | tr "\\0" "\\n" | grep -i ROS_DOMAIN 2>/dev/null; systemctl status rov-sensors.service 2>&1 | head -20')
stdout.channel.settimeout(10)
print('=== rov_3588_node 启动信息 ===')
print(stdout.read().decode())

# 3. 杀掉冲突进程
print('=== 杀掉冲突进程 ===')
stdin, stdout, stderr = ssh.exec_command('kill -9 12169 13190 2>&1; sleep 1; echo killed')
stdout.channel.settimeout(10)
print(stdout.read().decode())

# 4. 验证 ttyS3 已释放
stdin, stdout, stderr = ssh.exec_command('fuser /dev/ttyS3 2>&1; echo "---"; fuser /dev/ttyS5 2>&1')
stdout.channel.settimeout(10)
print('=== 端口占用检查 ===')
print(stdout.read().decode())

# 5. 也杀掉之前的 altimeter_driver.py 进程
stdin, stdout, stderr = ssh.exec_command('pkill -f "altimeter_driver" 2>/dev/null; sleep 1; echo alt_killed')
stdout.channel.settimeout(10)
print(stdout.read().decode())

# 6. 重启高度计驱动
cmd = 'cd /opt/ros/rov_ros2_ws && source /opt/ros/setup.bash && export ROS_DOMAIN_ID=42 && nohup python3 sensors/altimeter_driver.py > /tmp/altimeter.log 2>&1 &'
ssh.exec_command(cmd)
print('高度计驱动已启动，等待5秒...')
time.sleep(5)

# 7. 查看日志
stdin, stdout, stderr = ssh.exec_command('cat /tmp/altimeter.log')
stdout.channel.settimeout(10)
print('=== 高度计日志 ===')
print(stdout.read().decode())

# 8. 再等5秒看有没有数据
time.sleep(5)
stdin, stdout, stderr = ssh.exec_command('cat /tmp/altimeter.log')
stdout.channel.settimeout(10)
print('=== 高度计日志 (10s后) ===')
print(stdout.read().decode())

# 9. 确认 ttyS3 状态
stdin, stdout, stderr = ssh.exec_command('fuser /dev/ttyS3 2>&1; ps aux | grep altimeter | grep -v grep')
stdout.channel.settimeout(10)
print('=== 最终状态 ===')
print(stdout.read().decode())

ssh.close()
