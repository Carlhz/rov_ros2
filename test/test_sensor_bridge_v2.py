#!/usr/bin/env python3
import os, sys, time, json
os.environ['ROS_DOMAIN_ID'] = '42'
import rclpy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

rclpy.init()
n = rclpy.create_node("test_sensor_bridge")

state = {"yaw": 0.0, "pitch": 0.0, "roll": 0.0, "depth": 0.0}
ins_count = 0
dep_count = 0

def att_cb(msg):
    global ins_count
    state["yaw"] = float(msg.z)
    state["pitch"] = float(msg.x)
    state["roll"] = float(msg.y)
    ins_count += 1
    if ins_count % 10 == 0:
        out = {"yaw": state["yaw"], "pitch": state["pitch"], "roll": state["roll"]}
        if dep_count > 0:
            out["depth"] = state["depth"]
        print(json.dumps(out), flush=True)
        sys.stdout.flush()

def dep_cb(msg):
    global dep_count
    state["depth"] = float(msg.data)
    dep_count += 1
    out = {"depth": state["depth"]}
    if ins_count > 0:
        out["yaw"] = state["yaw"]
        out["pitch"] = state["pitch"]
        out["roll"] = state["roll"]
    print(json.dumps(out), flush=True)
    sys.stdout.flush()

n.create_subscription(Vector3, "/ins/attitude", att_cb, 10)
n.create_subscription(Float32, "/rov/depth", dep_cb, 10)

print("TEST_SENSOR_BRIDGE_READY", flush=True)
sys.stderr.write("TEST_SENSOR_BRIDGE_READY\n")
sys.stderr.flush()

try:
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)
except KeyboardInterrupt:
    pass
finally:
    n.destroy_node()
    rclpy.shutdown()
