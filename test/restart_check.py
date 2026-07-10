#!/usr/bin/env python3
import os
import time
import subprocess

os.system("pkill -9 -f motor_controller")
time.sleep(2)

# 启动motor_controller
env = os.environ.copy()
env['ROS_DOMAIN_ID'] = '42'
proc = subprocess.Popen(
    ['python3', '-u', '/opt/ros/rov_ros2_ws/motor_controller.py'],
    stdout=open('/tmp/motor_controller.log', 'w'),
    stderr=subprocess.STDOUT,
    env=env,
    cwd='/opt/ros/rov_ros2_ws'
)

print(f"motor_controller started, PID={proc.pid}")
time.sleep(10)

# 检查进程
result = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
for line in result.stdout.split('\n'):
    if 'motor_controller' in line and 'grep' not in line:
        print(line)

# 检查日志
print("\n=== 日志检查 ===")
with open('/tmp/motor_controller.log', 'r') as f:
    lines = f.readlines()
    for line in lines[-50:]:
        print(line.rstrip())
