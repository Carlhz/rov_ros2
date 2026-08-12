#!/usr/bin/env python3
"""上传并运行简化高度计测试"""
import paramiko, time

rk = {'hostname': '172.16.28.82', 'username': 'root', 'password': '159357'}

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(**rk, timeout=10)

# 杀掉 altimeter 驱动（用 pkill -f 加引号避免匹配自身）
stdin, stdout, stderr = ssh.exec_command('pkill -f "altimeter_driver" 2>/dev/null; sleep 2; echo pkill_done')
stdout.channel.settimeout(10)
print('pkill:', stdout.read().decode().strip())

# 检查 ttyS3 是否被占用
stdin, stdout, stderr = ssh.exec_command('fuser /dev/ttyS3 2>&1; lsof /dev/ttyS3 2>&1 | head -5')
stdout.channel.settimeout(10)
print('ttyS3 占用:', stdout.read().decode().strip())

# 上传测试脚本
sftp = ssh.open_sftp()
sftp.put(r'D:\Carl_WorkStation\rov_ros2\deploy\test_alt_simple.py', '/tmp/test_alt_simple.py')
sftp.close()
print('上传完成')

# 运行测试（PYTHONUNBUFFERED 确保输出不缓冲）
stdin, stdout, stderr = ssh.exec_command('PYTHONUNBUFFERED=1 timeout 30 python3 /tmp/test_alt_simple.py 2>&1')
stdout.channel.settimeout(35)
try:
    out = stdout.read().decode()
    print('=== 输出 ===')
    print(out)
except Exception as e:
    print(f'读取超时: {e}')

ssh.close()
