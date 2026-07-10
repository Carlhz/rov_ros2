#!/bin/bash
source /opt/ros/setup.bash
export ROS_DOMAIN_ID=42
cd /opt/ros/rov_ros2_ws

# 清理旧日志
> /tmp/motor_controller.log
> /tmp/sensor_bridge_error.log
> /tmp/sensor_bridge_stderr.log
> /tmp/sensor_pipe_raw.log

# 启动motor_controller
python3 -u motor_controller.py > /tmp/motor_controller.log 2>&1 &

echo "motor_controller started, PID=$!"
echo "Monitoring logs..."
