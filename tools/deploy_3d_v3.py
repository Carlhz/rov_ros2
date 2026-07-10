"""Deploy fixed sonar_3d_view.py (all-English + full scatter rebuild) to VM."""
import paramiko
import time

VM_IP = "172.16.30.0"
LOCAL = r"D:\Carl_WorkStation\rov_ros2\monitor\sonar_3d_view.py"
REMOTE = "/home/carl/rov_ros2_ws/monitor/sonar_3d_view.py"

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(VM_IP, username="carl", password="159357", timeout=10)


def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")


# Upload
with open(LOCAL, "rb") as f:
    content = f.read()
sftp = ssh.open_sftp()
with sftp.file(REMOTE, "wb") as f:
    f.write(content)
sftp.close()
print("Upload OK")

# Syntax check
print("\n=== Syntax check ===")
result = run("python3 -m py_compile " + REMOTE + " 2>&1")
if result.strip():
    print("SYNTAX ERROR:", result)
else:
    print("Syntax OK")

# Clear any .pyc cache
print("\n=== Clear .pyc cache ===")
print(run("rm -f /home/carl/rov_ros2_ws/monitor/__pycache__/sonar_3d_view*.pyc 2>/dev/null; echo done"))

# Quick test: run callbacks for 5 sec
print("\n=== 5s callback test (data flow check) ===")
result = run(
    'bash -c "source /opt/ros/foxy/setup.bash && export ROS_DOMAIN_ID=0 '
    '&& export MPLBACKEND=Agg '
    '&& timeout 8 python3 -c \\"'
    'import rclpy; rclpy.init(); '
    'import sys; sys.path.insert(0, chr(47)+chr(104)+chr(111)+chr(109)+chr(101)+'
    'chr(47)+chr(99)+chr(97)+chr(114)+chr(108)+chr(47)+chr(114)+chr(111)+'
    'chr(118)+chr(95)+chr(114)+chr(111)+chr(115)+chr(50)+chr(95)+chr(119)+'
    'chr(115)+chr(47)+chr(109)+chr(111)+chr(110)+chr(105)+chr(116)+chr(111)+chr(114)); '
    'from sonar_3d_view import SonarCache; '
    'import threading; from rclpy.executors import SingleThreadedExecutor; '
    'node = SonarCache(); ex = SingleThreadedExecutor(); ex.add_node(node); '
    'thr = threading.Thread(target=ex.spin, daemon=True); thr.start(); '
    'import time; '
    'for i in range(5): '
    '    time.sleep(1.0); '
    '    pts, angle, fps = node.snapshot(); '
    '    print(f\\\"t={i+1}s  pts={len(pts)}  angle={angle:.1f}  fps={fps:.1f}\\\"); '
    'rclpy.shutdown(); '
    '\\" 2>&1"',
    timeout=20
)
print(result)

ssh.close()
print("\nDeploy complete. Run in VM terminal:")
print("  cd ~/rov_ros2_ws && source /opt/ros/foxy/setup.bash")
print("  export ROS_DOMAIN_ID=0")
print("  python3 monitor/sonar_3d_view.py --range 5")
