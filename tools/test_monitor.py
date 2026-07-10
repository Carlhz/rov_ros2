"""Run sonar monitor directly on VM via SSH to diagnose"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.30.0", username="carl", password="159357", timeout=10)
def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

print("=== 先测试 quick_view (文本行, 5秒) ===")
cmd = (
    "bash -c 'cd ~/rov_ros2_ws; "
    "source /opt/ros/foxy/setup.bash; "
    "source install/local_setup.bash 2>/dev/null || true; "
    "export ROS_DOMAIN_ID=0; "
    "export ROS_LOCALHOST_ONLY=0; "
    "PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages:\\$PYTHONPATH; "
    "timeout 6 python3 monitor/sonar_quick_view.py --topic original 2>&1'"
)
result = run(cmd, timeout=15)
print(result)

print("\n=== 检查 Python rclpy 是否正常 ===")
print(run("bash -c 'source /opt/ros/foxy/setup.bash; python3 -c \"import rclpy; print(rclpy.__file__)\" 2>&1'"))

print("\n=== 检查当前终端 PATH 有没有被 SDK 污染 ===")
print(run("echo $PYTHONPATH | tr ':' '\n' | grep -i ros; echo ---; echo $PATH | tr ':' '\n' | grep -i ros"))

ssh.close()
