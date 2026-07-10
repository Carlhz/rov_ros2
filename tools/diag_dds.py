"""Diagnose DDS data flow issue between VM and RK3588"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.30.0", username="carl", password="159357", timeout=10)
def run(cmd, timeout=10):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

print("=== 网络接口 ===")
print(run("ip addr show | grep -E '^[0-9]|inet '"))

print("=== 路由表 ===")
print(run("ip route"))

print("=== Ping 192.168.0.5 (声纳) ===")
print(run("ping -c 2 -W 2 192.168.0.5 2>&1"))

print("=== Ping 192.168.0.99 (RK3588) ===")
print(run("ping -c 2 -W 2 192.168.0.99 2>&1"))

print("=== Ping 172.16.28.82 (RK3588 main) ===")
print(run("ping -c 2 -W 2 172.16.28.82 2>&1"))

print("=== ROS2 topic list (domain=0) ===")
# Use ; instead of &&
cmd = 'bash -c \'source /opt/ros/setup.bash 2>/dev/null; source ~/rov_ros2_ws/install/setup.bash 2>/dev/null; export ROS_DOMAIN_ID=0; ros2 topic list 2>&1\''
print(run(cmd))

print("=== ROS2 topic info /sonar/omni/original ===")
cmd = 'bash -c \'source /opt/ros/setup.bash 2>/dev/null; source ~/rov_ros2_ws/install/setup.bash 2>/dev/null; export ROS_DOMAIN_ID=0; ros2 topic info /sonar/omni/original 2>&1\''
print(run(cmd))

print("=== 尝试 echo (5秒) ===")
cmd = 'bash -c \'source /opt/ros/setup.bash 2>/dev/null; source ~/rov_ros2_ws/install/setup.bash 2>/dev/null; export ROS_DOMAIN_ID=0; timeout 5 ros2 topic echo /sonar/omni/original 2>&1 | head -30\''
print(run(cmd, timeout=12))

print("=== RMW Implementation ===")
print(run("bash -c 'source /opt/ros/setup.bash 2>/dev/null; echo RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION; dpkg -l | grep ros-foxy-rmw 2>/dev/null; which ros2'"))

ssh.close()
