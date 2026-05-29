#!/bin/bash
# RK3588 INS 控制器启动脚本

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

cd /opt/ros/rov_ros2_ws
source /opt/ros/setup.bash

python3 rk3588_ins_controller.py
