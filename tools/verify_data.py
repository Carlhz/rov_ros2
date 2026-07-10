"""Verify if sonar actually producing data - try echo with timeout"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.28.82", username="root", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

print("=== 声纳回声尝试 (最多5秒) ===")
result = run('bash -c "source /opt/ros/setup.bash; source /opt/ros/rov_ros2_ws/install/local_setup.bash; export ROS_DOMAIN_ID=0; timeout 6 ros2 topic echo /sonar/omni/original 2>&1 | head -30 || echo NO_DATA_TIMEOUT"', timeout=12)
print(result)

print("=== 直接抓包看看声纳有没有回数据 ===")
print(run("timeout 3 tcpdump -i any -c 5 -n 'host 192.168.0.5 and udp port 23' 2>&1 || echo NO_PACKETS", timeout=10))

print("=== 日志中是否有错误 ===")
print(run("cat /root/.ros/log/*/sonar_omni_driver*stdout*.log 2>/dev/null | tail -20 || echo no_ros_log"))

ssh.close()
