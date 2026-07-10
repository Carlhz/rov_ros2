#!/bin/bash
# VM 桌面一键启动 — ROV 综合监控
cd /home/carl/rov_ros2_ws
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
python3 monitor/integrated_monitor.py
