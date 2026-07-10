#!/usr/bin/env python3
"""测试用：从RK3588直接发布定深指令，触发motor_controller本地PID"""
import os
os.environ['ROS_DOMAIN_ID'] = '42'

import rclpy
from geometry_msgs.msg import Twist
import time

rclpy.init()
n = rclpy.create_node("test_dive_trigger")
p = n.create_publisher(Twist, "/rov/cmd_vel", 10)

msg = Twist()
msg.linear.y = 1.0     # dive_flag
msg.linear.z = 0.5      # target_depth=0.5m

print("发布定深指令: target=0.5m, dive_flag=1.0, 10Hz x 60s...")
for i in range(600):    # 60s @ 10Hz
    p.publish(msg)
    rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.1)

print("停止发布")
n.destroy_node()
rclpy.shutdown()
