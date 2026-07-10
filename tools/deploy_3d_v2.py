"""Deploy v2 sonar_3d_view.py to VM and test"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.30.0", username="carl", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

# 1. Upload
with open(r"D:\Carl_WorkStation\rov_ros2\monitor\sonar_3d_view.py", "rb") as f:
    content = f.read()

sftp = ssh.open_sftp()
with sftp.file("/home/carl/rov_ros2_ws/monitor/sonar_3d_view.py", "wb") as f:
    f.write(content)
# Clear pycache
run("rm -rf ~/rov_ros2_ws/monitor/__pycache__/sonar_3d* 2>/dev/null; echo cache_cleared")
sftp.close()
print("Upload OK, cache cleared")

# 2. Syntax check
print("\n=== 语法检查 ===")
r = run("python3 -m py_compile /home/carl/rov_ros2_ws/monitor/sonar_3d_view.py 2>&1")
print("Syntax OK" if not r.strip() else r)

# 3. Key changes verification
print("\n=== 关键变更验证 ===")
print(run("grep -n 'sc_holder\\[0\\] is None\\|sc_holder\\[0\\].remove()\\|fig.colorbar\\|cbar_holder' ~/rov_ros2_ws/monitor/sonar_3d_view.py"))

# 4. Test import and ROS data callback (no GUI)
print("\n=== 无 GUI 功能测试 (5秒) ===")
test_script = """
import rclpy, sys, threading, time
sys.path.insert(0, '/home/carl/rov_ros2_ws/monitor')
from sonar_3d_view import SonarCache
from rclpy.executors import SingleThreadedExecutor

rclpy.init()
node = SonarCache()
ex = SingleThreadedExecutor()
ex.add_node(node)
thr = threading.Thread(target=lambda: ex.spin(), daemon=True)
thr.start()

for i in range(5):
    time.sleep(1.0)
    pts, angle, fps = node.snapshot()
    print(f"t={i+1}s  pts={len(pts)}  angle={angle:.1f}  fps={fps:.1f}")

rclpy.shutdown()
"""

sftp = ssh.open_sftp()
with sftp.file("/tmp/test3d_v2.py", "w") as f:
    f.write(test_script)
sftp.close()

r = run(
    'bash -c "source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=0; '
    'timeout 12 python3 /tmp/test3d_v2.py 2>&1"',
    timeout=20
)
print(r)

ssh.close()
print("\nDone - v2 deployed to VM")
