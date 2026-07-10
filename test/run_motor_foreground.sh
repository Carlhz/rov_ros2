#!/bin/bash
source /opt/ros/setup.bash
export ROS_DOMAIN_ID=42
cd /opt/ros/rov_ros2_ws

# 清理旧进程
pkill -9 -f motor_controller
sleep 2

# 前台运行，实时查看输出
python3 -u motor_controller.py
