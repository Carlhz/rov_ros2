#!/usr/bin/env python3
"""在 RK3588 上批量修改所有 192.168.0.7 → 192.168.0.5"""
import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect("172.16.28.82", username="root", password="159357", timeout=10)

def run(cmd, timeout=15):
    stdin, stdout, stderr = ssh.exec_command(cmd, timeout=timeout)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    return out + err

# 1. 先找出所有包含 192.168.0.7 的文件
print("=" * 60)
print("1. 查找所有包含 192.168.0.7 的文件")
print("=" * 60)
result = run("grep -rl '192\.168\.0\.7' /opt/ros/rov_ros2_ws/ --include='*.py' --include='*.cpp' --include='*.hpp' --include='*.yaml' --include='*.xml' 2>/dev/null | grep -v '.workbuddy'")
print(result)

# 2. 批量替换（排除 workbuddy 目录）
print("=" * 60)
print("2. 执行替换")
print("=" * 60)
sed_result = run("grep -rl '192\.168\.0\.7' /opt/ros/rov_ros2_ws/ --include='*.py' --include='*.cpp' --include='*.hpp' --include='*.yaml' --include='*.xml' 2>/dev/null | grep -v '.workbuddy' | xargs -r sed -i 's/192\.168\.0\.7/192.168.0.5/g' 2>&1; echo 'DONE'")
print(sed_result)

# 3. 验证没有遗留
print("=" * 60)
print("3. 验证残留")
print("=" * 60)
leftover = run("grep -rn '192\.168\.0\.7' /opt/ros/rov_ros2_ws/ --include='*.py' --include='*.cpp' --include='*.hpp' --include='*.yaml' --include='*.xml' 2>/dev/null | grep -v '.workbuddy'")
if leftover.strip():
    print("还有残留:")
    print(leftover)
else:
    print("全部替换完毕，无残留！")

# 4. 重启声纳驱动
print()
print("=" * 60)
print("4. 启动声纳驱动 (正确 IP: 192.168.0.5)")
print("=" * 60)
# 先杀掉旧进程
run("pkill -f sonar_omni_driver 2>/dev/null; sleep 1; echo 'killed'")

# Source ROS2 环境并启动
launch_cmd = """
source /opt/ros/rov_ros2_ws/install/setup.bash 2>/dev/null || source /opt/ros/rov_ros2_ws/install/local_setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
nohup ros2 launch rov_sonar_driver sonar_omni.launch.py server_ip:=192.168.0.5 > /tmp/sonar_omni.log 2>&1 &
echo "PID=$!"
sleep 3
"""
print(run(launch_cmd, timeout=10))

# 5. 检查驱动状态
print()
print("=" * 60)
print("5. 驱动日志 (最后 20 行)")
print("=" * 60)
print(run("tail -20 /tmp/sonar_omni.log 2>/dev/null || echo 'no_log'"))

print()
print("=" * 60)
print("6. 声纳话题列表")
print("=" * 60)
ros2_check = """
source /opt/ros/rov_ros2_ws/install/setup.bash 2>/dev/null || source /opt/ros/rov_ros2_ws/install/local_setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0
ros2 topic list 2>/dev/null | grep -i sonar
"""
print(run(ros2_check, timeout=10))

ssh.close()
print("\nRK3588 修改完成！")
