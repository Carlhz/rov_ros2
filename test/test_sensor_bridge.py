#!/usr/bin/env python3
import os
os.environ['ROS_DOMAIN_ID'] = '42'
import rclpy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
import json
import sys

rclpy.init()
n = rclpy.create_node("sensor_bridge_test")

msg_count = 0

def att_cb(msg):
    global msg_count
    msg_count += 1
    data = {"yaw": float(msg.z), "pitch": float(msg.x), "roll": float(msg.y)}
    print(f"INS [{msg_count}]: {data}", flush=True)
    sys.stderr.write(f"INS_CB: yaw={msg.z:.1f}, pitch={msg.x:.1f}, roll={msg.y:.1f}\n")
    sys.stderr.flush()

def dep_cb(msg):
    global msg_count
    msg_count += 1
    data = {"depth": float(msg.data)}
    print(f"DEPTH [{msg_count}]: {data}", flush=True)
    sys.stderr.write(f"DEPTH_CB: depth={msg.data:.2f}m\n")
    sys.stderr.flush()

n.create_subscription(Vector3, "/ins/attitude", att_cb, 10)
n.create_subscription(Float32, "/rov/depth", dep_cb, 10)

sys.stderr.write("SENSOR_BRIDGE_TEST_READY\n")
sys.stderr.flush()

print("Waiting for messages...", flush=True)

try:
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
except KeyboardInterrupt:
    pass

n.destroy_node()
rclpy.shutdown()
