"""Check VM version of sonar_3d_view.py"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.30.0", username="carl", password="159357", timeout=10)

def run(cmd, timeout=10):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace")

print("=== scatter init (line ~107-110) ===")
print(run("sed -n '107,111p' ~/rov_ros2_ws/monitor/sonar_3d_view.py"))

print("\n=== _update function (line ~164-200) ===")
print(run("sed -n '164,205p' ~/rov_ros2_ws/monitor/sonar_3d_view.py"))

print("\n=== Check for old code patterns ===")
print(run("grep -n 'c=\\[\\]' ~/rov_ros2_ws/monitor/sonar_3d_view.py"))
print(run("grep -n 'sc_holder' ~/rov_ros2_ws/monitor/sonar_3d_view.py"))

# Also check the file size to confirm it's the new version
print("\n=== File info ===")
print(run("wc -l ~/rov_ros2_ws/monitor/sonar_3d_view.py"))
print(run("md5sum ~/rov_ros2_ws/monitor/sonar_3d_view.py"))

ssh.close()
