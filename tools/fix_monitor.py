"""Fix start_monitor.sh to force domain 0 on VM"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.30.0", username="carl", password="159357", timeout=10)
def run(cmd, timeout=10):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace")

# Fix: force domain 0 for sonar monitoring
run("sed -i 's|ROS_DOMAIN=\"${ROS_DOMAIN_ID:-0}\"|ROS_DOMAIN=\"0\"  # sonar fixed domain 0|g' ~/rov_ros2_ws/monitor/start_monitor.sh")

# Verify
print("=== After fix ===")
print(run("head -15 ~/rov_ros2_ws/monitor/start_monitor.sh"))

# Update .bashrc comment
run("sed -i 's|export ROS_DOMAIN_ID=42|export ROS_DOMAIN_ID=42  # INS; sonar uses domain 0|g' ~/.bashrc")
print("=== bashrc domain ===")
print(run("grep DOMAIN ~/.bashrc"))

# Verify monitor now works
print("\n=== Quick test monitor ===")
cmd = "bash -c 'cd ~/rov_ros2_ws; ./monitor/start_monitor.sh quick 2>&1'"
print(run(cmd, timeout=8)[:500])

ssh.close()
