"""Test if VM can actually receive sonar data with correct ROS2 path"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.30.0", username="carl", password="159357", timeout=10)
def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

print("=== VM 上收到声纳数据了吗？echo 5秒 ===")
cmd = (
    "bash -c 'source /opt/ros/foxy/setup.bash; "
    "export ROS_DOMAIN_ID=0; "
    "timeout 6 ros2 topic echo /sonar/omni/original 2>&1 | head -40'"
)
print(run(cmd, timeout=15))

print("\n=== topic info ===")
cmd = (
    "bash -c 'source /opt/ros/foxy/setup.bash; "
    "export ROS_DOMAIN_ID=0; "
    "ros2 topic info /sonar/omni/original 2>&1'"
)
print(run(cmd))

print("\n=== topic hz ===")
cmd = (
    "bash -c 'source /opt/ros/foxy/setup.bash; "
    "export ROS_DOMAIN_ID=0; "
    "timeout 5 ros2 topic hz /sonar/omni/original 2>&1'"
)
print(run(cmd, timeout=10))

ssh.close()
