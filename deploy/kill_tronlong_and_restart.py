#!/usr/bin/env python3
"""杀掉 start_tronlong_3588.sh 及所有 TL3588 子进程，重启高度计"""
import paramiko, time

rk = {'hostname': '172.16.28.82', 'username': 'root', 'password': '159357'}

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(**rk, timeout=10)

# 1. 杀掉 start_tronlong_3588.sh 脚本 (PID 2294) 和所有 TL3588 节点
print('=== 杀掉 TL3588 进程 ===')
kill_cmd = '''
kill -9 2294 2>/dev/null
pkill -f "start_tronlong_3588" 2>/dev/null
pkill -f "rov_3588_node" 2>/dev/null
pkill -f "rov_light_rs485" 2>/dev/null
pkill -f "depth_sensor_driver_node" 2>/dev/null
pkill -f "altimeter_driver" 2>/dev/null
sleep 2
echo "done"
'''
stdin, stdout, stderr = ssh.exec_command(kill_cmd)
stdout.channel.settimeout(10)
print(stdout.read().decode().strip())

# 2. 验证全部已杀掉
stdin, stdout, stderr = ssh.exec_command('ps aux | grep -E "rov_3588|rov_light|depth_sensor_driver_node|start_tronlong|altimeter" | grep -v grep')
stdout.channel.settimeout(10)
remaining = stdout.read().decode().strip()
print('=== 剩余进程 ===')
print(remaining if remaining else '(无)')

# 3. 验证 ttyS3 和 ttyS5 已释放
stdin, stdout, stderr = ssh.exec_command('fuser /dev/ttyS3 2>&1; echo "---"; fuser /dev/ttyS5 2>&1')
stdout.channel.settimeout(10)
print('=== 端口占用 ===')
print(stdout.read().decode())

# 4. 重启高度计驱动
print('\n=== 重启高度计驱动 ===')
cmd = 'cd /opt/ros/rov_ros2_ws && source /opt/ros/setup.bash && export ROS_DOMAIN_ID=42 && nohup python3 sensors/altimeter_driver.py > /tmp/altimeter.log 2>&1 &'
ssh.exec_command(cmd)
print('已启动，等待8秒...')
time.sleep(8)

# 5. 查看日志
stdin, stdout, stderr = ssh.exec_command('cat /tmp/altimeter.log')
stdout.channel.settimeout(10)
print('=== 高度计日志 ===')
print(stdout.read().decode())

# 6. 检查 ttyS3 只被我们的驱动占用
stdin, stdout, stderr = ssh.exec_command('fuser /dev/ttyS3 2>&1; ps aux | grep altimeter | grep -v grep')
stdout.channel.settimeout(10)
print('=== 最终 ttyS3 状态 ===')
print(stdout.read().decode())

ssh.close()
