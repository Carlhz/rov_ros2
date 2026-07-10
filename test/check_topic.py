#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

def depth_cb(msg):
    print(f"Depth: {msg.data:.2f}m")

def att_cb(msg):
    print(f"Attitude: yaw={msg.z:.1f} pitch={msg.x:.1f} roll={msg.y:.1f}")

rclpy.init()
node = rclpy.create_node("topic_checker")

node.create_subscription(Float32, "/rov/depth", depth_cb, 10)
node.create_subscription(Vector3, "/ins/attitude", att_cb, 10)

print("Subscribed to /rov/depth and /ins/attitude")
print("Waiting for messages...")

import time
start = time.time()
while time.time() - start < 5:
    rclpy.spin_once(node, timeout_sec=0.1)

print("Done")
node.destroy_node()
rclpy.shutdown()
