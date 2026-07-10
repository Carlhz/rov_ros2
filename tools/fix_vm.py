"""Fix VM: find correct ROS2 + add 192.168.0.x route"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.30.0", username="carl", password="159357", timeout=10)
def run(cmd, timeout=10):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

print("=== 查找 ROS2 安装 ===")
print(run("ls /opt/ros/"))
print(run("ls /opt/ros/foxy/ 2>/dev/null; ls /opt/ros/galactic/ 2>/dev/null; echo done"))

print("=== 查找 local_setup.bash ===")
print(run("find /opt -name 'local_setup.bash' -maxdepth 5 2>/dev/null"))

print("=== 当前 PATH 中的 ros ===")
print(run("echo $PATH | tr ':' '\n' | grep -i ros"))

print("=== 检查 Ros2 命令位置 ===")
print(run("which ros2"))
print(run("ls -la $(which ros2) 2>/dev/null"))

print("=== 原生 rclpy ===")
print(run("python3 -c 'import rclpy; print(rclpy.__file__)' 2>&1"))

# --- Now fix: add route and source correct path ---
print("\n=== 添加路由 192.168.0.0/24 via 172.16.28.82 ===")
route_cmd = "sudo -S ip route add 192.168.0.0/24 via 172.16.28.82"
print(run(f"echo '159357' | {route_cmd} 2>&1"))

print("=== 验证新路由 ===")
print(run("ip route | grep 192.168"))

print("=== 测试 192.168.0.5 连通性 ===")
print(run("ping -c 2 -W 2 192.168.0.5 2>&1"))

# --- Use correct ROS2 ---
print("\n=== 使用正确路径测 ros2 topic list ===")
print(run("bash -c 'source /opt/ros/foxy/setup.bash 2>/dev/null; source /opt/ros/galactic/setup.bash 2>/dev/null; export ROS_DOMAIN_ID=0; ros2 topic list 2>&1'"))

print("=== 同一命令再次执行（verbose） ===")
print(run("bash -c 'source /opt/ros/foxy/setup.bash; export ROS_DOMAIN_ID=0; ros2 topic list 2>&1'"))

ssh.close()
