"""Fix sonar IP back to 192.168.0.5 on RK3588 (accidentally reverted)"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.28.82", username="root", password="159357", timeout=10)

def run(cmd, timeout=10):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

base = "/opt/ros/rov_ros2_ws/install/rov_sonar_driver/share/rov_sonar_driver"
files = [
    f"{base}/config/sonar_omni.yaml",
    f"{base}/launch/sonar_omni.launch.py",
]
for f in files:
    run(f"sed -i 's/192\\.168\\.0\\.7/192.168.0.5/g' {f}")
    result = run(f"grep '192.168.0' {f}")
    print(f"{f}: {result.strip()}")

# Final verify
print()
print("=== Final IP config ===")
print(run('grep -rn "192.168.0" /opt/ros/rov_ros2_ws/install/ --include="*.py" --include="*.yaml" --include="*.launch.py" 2>/dev/null | grep -v pycache'))

ssh.close()
