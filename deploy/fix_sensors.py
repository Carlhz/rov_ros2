#!/usr/bin/env python3
"""Fix depth sensor and altimeter on RK3588"""
import paramiko, time, sys

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('172.16.28.82', username='root', password='159357', timeout=10)

def run(cmd, timeout=10):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode(), stderr.read().decode()

# 1. Upload modified altimeter driver
print("=== 上传高度计驱动 ===")
sftp = ssh.open_sftp()
sftp.put('D:/Carl_WorkStation/rov_ros2/sensors/altimeter_driver.py',
         '/opt/ros/rov_ros2_ws/sensors/altimeter_driver.py')
sftp.close()
print("已上传(超时1.5s)")

# 2. Kill old drivers
print("\n=== 停止旧驱动 ===")
run('pkill -f altimeter_driver 2>/dev/null; pkill -f depth_sensor_driver.py 2>/dev/null; sleep 1')
print("已停止")

# 3. Start depth sensor
print("\n=== 启动深度计 ===")
out, err = run(
    'export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 DEPTH_PORT=/dev/ttyS5 && '
    'cd /opt/ros/rov_ros2_ws && '
    'source /opt/ros/setup.bash && '
    'nohup python3 sensors/depth_sensor_driver.py > /tmp/depth_sensor.log 2>&1 & '
    'echo PID=$!', timeout=15)
print(out.strip())

# 4. Start altimeter
print("\n=== 启动高度计 ===")
out, err = run(
    'export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 ALTI_PORT=/dev/ttyS3 && '
    'cd /opt/ros/rov_ros2_ws && '
    'source /opt/ros/setup.bash && '
    'nohup python3 sensors/altimeter_driver.py > /tmp/altimeter.log 2>&1 & '
    'echo PID=$!', timeout=15)
print(out.strip())

# 5. Wait and check
time.sleep(8)

print("\n=== 深度计日志 ===")
out, _ = run('tail -5 /tmp/depth_sensor.log 2>&1')
print(out)

print("=== 高度计日志 ===")
out, _ = run('tail -10 /tmp/altimeter.log 2>&1')
print(out)

print("=== 进程状态 ===")
out, _ = run('ps aux | grep -E "depth_sensor_driver.py|altimeter_driver.py" | grep -v grep')
print(out)

# 6. Check INS yaw vs DVL heading
print("=== INS 姿态(最新) ===")
out, _ = run('tail -3 /tmp/ins_driver.log 2>&1')
print(out)

print("=== DVL 姿态(话题) ===")
out, _ = run(
    'source /opt/ros/setup.bash && export ROS_DOMAIN_ID=42 && '
    'timeout 3 ros2 topic echo /rov/dvl/status --once 2>/dev/null | grep -A5 attitude', timeout=10)
print(out if out.strip() else "(无数据)")

ssh.close()
print("\n完成!")
