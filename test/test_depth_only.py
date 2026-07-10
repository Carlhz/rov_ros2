#!/usr/bin/env python3
import os
os.environ['ROS_DOMAIN_ID'] = '42'
import rclpy
from std_msgs.msg import Float32

rclpy.init()
n = rclpy.create_node("depth_test")

def dep_cb(msg):
    print(f"Depth: {msg.data:.2f}m", flush=True)

n.create_subscription(Float32, "/rov/depth", dep_cb, 10)
print("Subscribed to /rov/depth, waiting for data...", flush=True)

try:
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
except KeyboardInterrupt:
    pass

n.destroy_node()
rclpy.shutdown()
