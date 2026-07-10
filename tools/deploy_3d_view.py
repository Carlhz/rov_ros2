#!/usr/bin/env python3
"""Upload sonar_3d_view.py to VM and verify imports."""

import paramiko, os

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.30.0", username="carl", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

# Step 1: upload
local = r"D:\Carl_WorkStation\rov_ros2\monitor\sonar_3d_view.py"
remote = "/home/carl/rov_ros2_ws/monitor/sonar_3d_view.py"

with ssh.open_sftp() as sftp:
    sftp.put(local, remote)
    sftp.chmod(remote, 0o755)
print(f"已上传: {remote}")

# Step 2: syntax check
print("\n=== Python 语法检查 ===")
print(run("python3 -m py_compile ~/rov_ros2_ws/monitor/sonar_3d_view.py 2>&1 && echo 'OK' || echo 'FAIL'"))

# Step 3: import check (headless, should fail at plt.show() but ok)
print("\n=== import 测试 ===")
result = run("""
cd ~/rov_ros2_ws
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=0
python3 -c "
import matplotlib
matplotlib.use('Agg')  # headless
from monitor.sonar_3d_view import SonarCache, build_3d_view
print('SonarCache: OK')
print('build_3d_view: OK')
" 2>&1
""", timeout=15)
print(result)

# Step 4: quick data test (subscribe for 3s, verify callbacks fire)
print("\n=== 数据回调测试 (3秒) ===")
print(run("""
cd ~/rov_ros2_ws
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=0
python3 -c "
import sys; sys.path.insert(0, '.')
from monitor.sonar_3d_view import SonarCache
import rclpy, threading, time
rclpy.init()
node = SonarCache()
from rclpy.executors import SingleThreadedExecutor
ex = SingleThreadedExecutor(); ex.add_node(node)
t = threading.Thread(target=ex.spin, daemon=True); t.start()
time.sleep(3)
pts, ang, fps = node.snapshot()
print(f'点数: {len(pts)}, 角度: {ang:.1f}°, FPS: {fps:.1f}')
if pts:
    print(f'首个点: x={pts[0][0]:.3f} y={pts[0][1]:.3f} z={pts[0][2]:.3f} intensity={pts[0][3]:.0f}')
ex.shutdown(); node.destroy_node(); rclpy.shutdown()
" 2>&1
""", timeout=15))

ssh.close()
print("\nDone.")
