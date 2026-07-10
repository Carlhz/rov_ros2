"""Deploy sonar_3d_view.py v4 (plt.pause loop, no FuncAnimation) to VM."""
import paramiko

VM_IP = "172.16.30.0"
LOCAL = r"D:\Carl_WorkStation\rov_ros2\monitor\sonar_3d_view.py"
REMOTE = "/home/carl/rov_ros2_ws/monitor/sonar_3d_view.py"

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(VM_IP, username="carl", password="159357", timeout=10)

# Upload
with open(LOCAL, "rb") as f:
    content = f.read()
sftp = ssh.open_sftp()
with sftp.file(REMOTE, "wb") as f:
    f.write(content)
sftp.close()
print("Upload OK")

# Clear pyc cache
stdin, stdout, stderr = ssh.exec_command(
    "rm -f /home/carl/rov_ros2_ws/monitor/__pycache__/sonar_3d_view*.pyc 2>/dev/null; echo done")
print("Cache clear:", stdout.read().decode().strip())

# Syntax check
stdin, stdout, stderr = ssh.exec_command(
    "python3 -m py_compile " + REMOTE + " 2>&1")
err = stderr.read().decode()
if err.strip():
    print("SYNTAX ERROR:", err.strip())
else:
    print("Syntax OK")

# Data flow test (5 seconds, Agg backend)
stdin, stdout, stderr = ssh.exec_command(
    "bash -c 'source /opt/ros/foxy/setup.bash && export ROS_DOMAIN_ID=0 && export MPLBACKEND=Agg && "
    "python3 -c \""
    "import rclpy; rclpy.init(); "
    "import sys; s='/home/carl/rov_ros2_ws/monitor'; sys.path.insert(0,s); "
    "from sonar_3d_view import SonarCache; "
    "import threading; from rclpy.executors import SingleThreadedExecutor; "
    "node = SonarCache(); ex = SingleThreadedExecutor(); ex.add_node(node); "
    "thr = threading.Thread(target=ex.spin, daemon=True); thr.start(); "
    "import time; "
    "for i in range(5): "
    "    time.sleep(1.0); "
    "    pts, angle, fps = node.snapshot(); "
    "    print(f\\\"t={i+1}s  pts={len(pts)}  angle={angle:.1f}  fps={fps:.1f}\"); "
    "rclpy.shutdown(); "
    "\" 2>&1'",
    timeout=25)
print("\n=== Data flow test (5s) ===")
print(stdout.read().decode().strip())

ssh.close()
print("\nDone. Run in VM terminal:")
print("  cd ~/rov_ros2_ws && source /opt/ros/foxy/setup.bash")
print("  export ROS_DOMAIN_ID=0")
print("  python3 monitor/sonar_3d_view.py --range 5")
