"""Quick verify: data flow + no Chinese chars in deployed file."""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.30.0", username="carl", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

# 1. Data flow check
print("=== Data flow (echo once) ===")
r = run('bash -c "source /opt/ros/foxy/setup.bash && export ROS_DOMAIN_ID=0 && ros2 topic echo --once /sonar/omni/original 2>&1"', timeout=12)
# Show just key lines
for line in r.split("\n"):
    if any(w in line.lower() for w in ("width:", "height:", "x:", "y:", "intensity:", "---")):
        print(line)
    if "---" in line:
        break

# 2. Check for non-ASCII chars (Chinese)
print("\n=== Non-ASCII check ===")
r2 = run("python3 -c \"import re; f=open('/home/carl/rov_ros2_ws/monitor/sonar_3d_view.py'); s=f.read(); f.close(); non_ascii=[c for c in s if ord(c)>127]; print('Non-ASCII chars:', len(non_ascii), '| First 100 chars:', repr(s[:80]))\"")
print(r2)

# 3. Check module imports cleanly
print("\n=== Import test ===")
r3 = run("python3 -c \"import sys; sys.path.insert(0,'/home/carl/rov_ros2_ws/monitor'); exec(open('/home/carl/rov_ros2_ws/monitor/sonar_3d_view.py').read().split('import rclpy')[0]); print('Top section OK')\"")
print(r3.strip())

ssh.close()
