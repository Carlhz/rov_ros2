"""Deep check: is sonar actually producing data?"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.28.82", username="root", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

print("=== 声纳话题 info ===")
print(run('bash -c "source /opt/ros/setup.bash; source /opt/ros/rov_ros2_ws/install/local_setup.bash; export ROS_DOMAIN_ID=0; ros2 topic info /sonar/omni/original 2>/dev/null"'))

print("=== 声纳话题 echo（采2条） ===")
print(run('bash -c "source /opt/ros/setup.bash; source /opt/ros/rov_ros2_ws/install/local_setup.bash; export ROS_DOMAIN_ID=0; timeout 5 ros2 topic echo /sonar/omni/original --once 2>&1 || echo NO_DATA"', timeout=10))

print("=== 驱动日志全量 ===")
print(run("cat /tmp/sonar_omni.log"))

ssh.close()
