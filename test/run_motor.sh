#!/bin/bash
source /opt/ros/setup.bash
export ROS_DOMAIN_ID=42
cd /opt/ros/rov_ros2_ws

# 清理旧日志
rm -f /tmp/motor_controller.log

# 启动motor_controller (前台运行, 便于查看输出)
python3 -u motor_controller.py 2>&1 | tee /tmp/motor_controller.log
