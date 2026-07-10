"""Sample sonar data from VM to assess water bucket test"""
import paramiko, math

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.30.0", username="carl", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace")

# Use a Python script on VM to parse PointCloud2 directly
sample_py = r'''
import rclpy, sys, struct, time
sys.path.insert(0, "/home/carl/rov_ros2_ws/monitor")
from sonar_3d_view import SonarCache
from rclpy.executors import SingleThreadedExecutor
import threading

rclpy.init()
node = SonarCache()
ex = SingleThreadedExecutor()
ex.add_node(node)
thr = threading.Thread(target=lambda: ex.spin(), daemon=True)
thr.start()

time.sleep(3.0)
pts, angle, fps = node.snapshot()
rclpy.shutdown()

if pts:
    dists = [math.hypot(p[0], p[1]) for p in pts]
    print(f"=== 声纳水桶数据摘要 ===")
    print(f"采样点数  : {len(pts)}")
    print(f"距离范围  : {min(dists):.3f} ~ {max(dists):.3f} m")
    print(f"距离均值  : {sum(dists)/len(dists):.3f} m")
    print(f"强度范围  : {min(p[3] for p in pts):.1f} ~ {max(p[3] for p in pts):.1f}")
    print(f"强度均值  : {sum(p[3] for p in pts)/len(pts):.1f}")
    print(f"z 轴范围  : {min(p[2] for p in pts):.4f} ~ {max(p[2] for p in pts):.4f}")
    print(f"当前角度  : {angle:.1f} deg")
    print(f"\n前8个点 (x, y, z, intensity, distance):")
    for i, p in enumerate(pts[:8]):
        d = math.hypot(p[0], p[1])
        print(f"  [{i:2d}] x={p[0]:+.5f} y={p[1]:+.5f} z={p[2]:+.4f} dist={d:.4f}m int={p[3]:.0f}")
    bins = [0, 0.1, 0.2, 0.3, 0.5, 1.0, 5.0]
    print(f"\n距离分布:")
    for i in range(len(bins)-1):
        cnt = sum(1 for d in dists if bins[i] <= d < bins[i+1])
        print(f"  {bins[i]:.1f}-{bins[i+1]:.1f}m: {cnt} 点")
else:
    print("WARN: 未收到声纳数据!")
'''

import math as _m
# Upload
sftp = ssh.open_sftp()
with sftp.file("/tmp/sample_sonar.py", "w") as f:
    f.write("import math\n" + sample_py)
sftp.close()

result = run(
    'bash -c "source /opt/ros/foxy/setup.bash; '
    'export ROS_DOMAIN_ID=0; '
    'timeout 15 python3 /tmp/sample_sonar.py 2>&1"',
    timeout=25
)
print(result)
ssh.close()
