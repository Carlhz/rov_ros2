#!/bin/bash
source /opt/ros/setup.bash
export ROS_DOMAIN_ID=42
cd /opt/ros/rov_ros2_ws

# 清理旧进程和日志
pkill -9 -f motor_controller
sleep 2
rm -f /tmp/motor_controller.log

# 启动motor_controller
python3 -u motor_controller.py > /tmp/motor_controller.log 2>&1 &

# 等待启动
sleep 5

# 检查进程
echo "=== 进程检查 ==="
ps aux | grep motor_controller | grep -v grep

# 检查日志
echo "=== 日志检查 (最后30行) ==="
tail -30 /tmp/motor_controller.log

# 检查传感器文件
echo "=== 传感器文件检查 ==="
ls -la /tmp/sensor_data.json 2>/dev/null || echo "sensor_data.json not found"

# 检查sensor_bridge进程
echo "=== sensor_bridge 进程检查 ==="
ps aux | grep sensor_bridge | grep -v grep || echo "sensor_bridge not running"
