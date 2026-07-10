"""Deploy sonar_log_snapshot.py to VM and run it."""
import paramiko
import os

VM_IP = "172.16.30.0"
LOCAL = r"D:\Carl_WorkStation\rov_ros2\tools\sonar_log_snapshot.py"
REMOTE = "/tmp/sonar_log_snapshot.py"

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

# Run on VM
stdin, stdout, stderr = ssh.exec_command(
    "bash -c 'source /opt/ros/foxy/setup.bash && "
    "export ROS_DOMAIN_ID=0 && python3 /tmp/sonar_log_snapshot.py'",
    timeout=30)
print(stdout.read().decode())
err = stderr.read().decode()
if err.strip():
    print("STDERR:", err.strip()[:2000])

ssh.close()
