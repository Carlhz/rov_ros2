"""Check sonar driver status on RK3588"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.28.82", username="root", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

print("=== 声纳驱动进程 ===")
print(run("ps aux | grep sonar_omni | grep -v grep"))

print("=== 最近日志 ===")
print(run("tail -20 /tmp/sonar_omni.log"))

print("=== 声纳话题 ===")
print(run('bash -c "source /opt/ros/setup.bash; source /opt/ros/rov_ros2_ws/install/local_setup.bash; export ROS_DOMAIN_ID=0; ros2 topic list 2>/dev/null"'))

print("=== 声纳话题频率 ===")
print(run('bash -c "source /opt/ros/setup.bash; source /opt/ros/rov_ros2_ws/install/local_setup.bash; export ROS_DOMAIN_ID=0; timeout 3 ros2 topic hz /sonar/omni/original 2>&1 || echo END"'))

ssh.close()
