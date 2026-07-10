#!/usr/bin/env python3
"""Fix start_monitor.sh path issues + persist route."""

import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.30.0", username="carl", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

def run_sudo(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(f"echo '159357' | sudo -S {cmd}", timeout=timeout)
    return stdout.read().decode("utf-8", errors="replace") + stderr.read().decode("utf-8", errors="replace")

print("=" * 60)
print("1. 检查 VM 上 monitor 目录和脚本位置")
print("=" * 60)
print(run("ls -la ~/rov_ros2_ws/monitor/"))
print(run("ls -la ~/rov_ros2_ws/src/rov_sonar_driver 2>&1 | head -5"))

print()
print("=" * 60)
print("2. 当前 start_monitor.sh 完整内容")
print("=" * 60)
print(run("cat ~/rov_ros2_ws/monitor/start_monitor.sh"))

print()
print("=" * 60)
print("3. 检查 ros2 在桌面终端的 source 路径")
print("=" * 60)
print(run("cat ~/.bashrc | grep -i ros"))
print(run("ls /opt/ros/foxy/setup.bash 2>/dev/null && echo 'foxy exists' || echo 'no foxy'"))
print(run("ls /opt/ros/setup.bash 2>/dev/null && echo '/opt/ros/setup.bash exists' || echo 'no /opt/ros/setup.bash'"))

print()
print("=" * 60)
print("4. 手动模拟桌面终端跑 monitor (带 bashrc)")
print("=" * 60)
simulate = """bash -l -c '
cd ~/rov_ros2_ws
echo "当前 ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "当前 ros2 路径: $(which ros2 2>/dev/null || echo not_found)"
export ROS_DOMAIN_ID=0
source /opt/ros/foxy/setup.bash 2>/dev/null
echo "加载后 ros2: $(which ros2)"
python3 ~/rov_ros2_ws/monitor/sonar_monitor.py &
PID=$!
sleep 4
kill $PID 2>/dev/null
wait $PID 2>/dev/null
echo "exit_code=$?"
'
"""
result = run(simulate, timeout=15)
print(result)

ssh.close()
