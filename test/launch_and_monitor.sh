#!/bin/bash
# 启动motor_controller并实时监控
pkill -9 -f motor_controller
sleep 1

source /opt/ros/setup.bash
export ROS_DOMAIN_ID=42
cd /opt/ros/rov_ros2_ws

# 清理旧日志
rm -f /tmp/motor_controller.log

# 后台启动
nohup python3 -u motor_controller.py > /tmp/motor_controller.log 2>&1 &
PID=$!
echo "Started motor_controller PID=$PID"
sleep 1

# 检查进程是否存活
if ps -p $PID > /dev/null 2>&1; then
    echo "Process is running"
    # 等待3秒让初始化完成
    sleep 3
    # 显示最新日志
    tail -30 /tmp/motor_controller.log
else
    echo "Process died immediately!"
    cat /tmp/motor_controller.log
fi
