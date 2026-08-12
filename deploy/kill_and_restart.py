#!/usr/bin/env python3
"""上传杀进程脚本并执行，然后重启高度计"""
import paramiko, time

rk = {'hostname': '172.16.28.82', 'username': 'root', 'password': '159357'}

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(**rk, timeout=10)

# 上传杀进程脚本
sftp = ssh.open_sftp()
sftp.put(r'D:\Carl_WorkStation\rov_ros2\deploy\kill_tronlong.sh', '/tmp/kill_tronlong.sh')
sftp.chmod('/tmp/kill_tronlong.sh', 0o755)
sftp.close()
print('上传杀进程脚本')

# 执行
stdin, stdout, stderr = ssh.exec_command('bash /tmp/kill_tronlong.sh 2>&1')
stdout.channel.settimeout(15)
print(stdout.read().decode())

# 等一下确认进程不会重启
print('等待3秒确认进程不重启...')
time.sleep(3)
stdin, stdout, stderr = ssh.exec_command('ps aux | grep -E "rov_3588|rov_light|depth_sensor_driver_node" | grep -v grep')
stdout.channel.settimeout(10)
remaining = stdout.read().decode().strip()
print('=== TL3588 进程检查 ===')
print(remaining if remaining else '(无 - 全部已杀掉)')

# 重启高度计驱动
print('\n=== 重启高度计驱动 ===')
cmd = 'cd /opt/ros/rov_ros2_ws && source /opt/ros/setup.bash && export ROS_DOMAIN_ID=42 && nohup python3 sensors/altimeter_driver.py > /tmp/altimeter.log 2>&1 &'
ssh.exec_command(cmd)
print('已启动，等待8秒...')
time.sleep(8)

# 查看日志
stdin, stdout, stderr = ssh.exec_command('cat /tmp/altimeter.log')
stdout.channel.settimeout(10)
print('=== 高度计日志 ===')
print(stdout.read().decode())

# 确认 ttyS3 只有我们的驱动
stdin, stdout, stderr = ssh.exec_command('fuser /dev/ttyS3 2>&1')
stdout.channel.settimeout(10)
print('ttyS3 占用:', stdout.read().decode().strip())

ssh.close()
