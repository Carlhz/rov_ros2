#!/usr/bin/env python3
"""启动声纳驱动 — 正确 source ROS2 环境"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.28.82", username="root", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    return out + err

# 先杀旧进程
print("=" * 60)
print("1. 清理旧进程")
print("=" * 60)
print(run("pkill -f sonar_omni_driver 2>/dev/null; sleep 1; echo 'cleaned'"))

# 正确的启动方式
print()
print("=" * 60)
print("2. 启动声纳驱动")
print("=" * 60)
start = """
bash -c '
source /opt/ros/humble/setup.bash
source /opt/ros/rov_ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
echo "ROS2 env loaded: $(which ros2)"
nohup ros2 launch rov_sonar_driver sonar_omni.launch.py server_ip:=192.168.0.5 > /tmp/sonar_omni.log 2>&1 &
echo "PID=$!"
sleep 4
'
"""
print(run(start, timeout=12))

# 查看日志
print()
print("=" * 60)
print("3. 驱动日志")
print("=" * 60)
print(run("cat /tmp/sonar_omni.log 2>/dev/null | tail -30", timeout=5))

# 检查进程
print()
print("=" * 60)
print("4. 进程状态")
print("=" * 60)
print(run("ps aux | grep sonar_omni | grep -v grep"))

# 话题
print()
print("=" * 60)
print("5. ROS2 话题")
print("=" * 60)
topics = run("bash -c 'source /opt/ros/humble/setup.bash; source /opt/ros/rov_ros2_ws/install/local_setup.bash; export ROS_DOMAIN_ID=0; ros2 topic list 2>/dev/null' | grep -i sonar", timeout=10)
if topics.strip():
    print(topics)
else:
    print("未发现 sonar 话题 (驱动可能未正常启动)")

ssh.close()
