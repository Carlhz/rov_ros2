#!/usr/bin/env python3
"""部署 start_all.sh 到 RK3588"""
import paramiko, time

rk = {'hostname': '172.16.28.82', 'username': 'root', 'password': '159357'}

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(**rk, timeout=10)

# 上传 start_all.sh
sftp = ssh.open_sftp()
sftp.put(r'D:\Carl_WorkStation\rov_ros2\rk3588\start_all.sh', '/opt/ros/rov_ros2_ws/start_all.sh')
sftp.chmod('/opt/ros/rov_ros2_ws/start_all.sh', 0o755)
sftp.close()
print('start_all.sh 部署完成')

# 验证文件
stdin, stdout, stderr = ssh.exec_command('head -5 /opt/ros/rov_ros2_ws/start_all.sh; echo "---"; grep -n "TL3588" /opt/ros/rov_ros2_ws/start_all.sh')
stdout.channel.settimeout(10)
print('验证:')
print(stdout.read().decode())

ssh.close()
print('完成')
