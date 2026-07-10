"""部署修复后的 sonar_3d_view.py 到 VM 并验证"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.30.0", username="carl", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

# 1. 上传修复后的脚本
sftp = ssh.open_sftp()
with open(r"D:\Carl_WorkStation\rov_ros2\monitor\sonar_3d_view.py", "rb") as f:
    content = f.read()

with sftp.file("/home/carl/rov_ros2_ws/monitor/sonar_3d_view.py", "wb") as f:
    f.write(content)
print("Upload OK")

# 2. 语法检查
print("\n=== 语法检查 ===")
result = run("python3 -m py_compile /home/carl/rov_ros2_ws/monitor/sonar_3d_view.py 2>&1")
if result.strip():
    print(result)
else:
    print("Syntax OK")

# 3. 数据流测试（无 GUI）
print("\n=== 当前声纳数据采样 (ros2 topic echo) ===")
result = run(
    'bash -c "source /opt/ros/foxy/setup.bash && export ROS_DOMAIN_ID=0 && '
    'timeout 3 ros2 topic echo --once /sonar/omni/original 2>&1 | head -30"',
    timeout=10
)
print(result)

# 4. 检查关键代码段是否已更新
print("=== 验证幽灵点初始化 ===")
result = run("grep -n 'c=np.array' /home/carl/rov_ros2_ws/monitor/sonar_3d_view.py")
print(result)

print("=== 验证 sc_holder ===")
result = run("grep -n 'sc_holder' /home/carl/rov_ros2_ws/monitor/sonar_3d_view.py")
print(result)

sftp.close()
ssh.close()
print("\nDone - 修复已部署到 VM")
