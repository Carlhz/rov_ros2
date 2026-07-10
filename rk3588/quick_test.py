#!/usr/bin/env python3
"""Quick test: 0.5m target, 20s"""
import os, time, json
os.environ['ROS_DOMAIN_ID'] = '42'
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('quick_test')
pub = node.create_publisher(Twist, '/rov/cmd_vel', 10)

depth, pitch, yaw = 0.0, 0.0, 0.0

def cb(msg):
    global depth, pitch, yaw
    d = json.loads(msg.data)
    depth = d.get('current_depth', 0)
    pitch = d.get('ins_pitch', 0)
    yaw = d.get('ins_yaw', 0)

node.create_subscription(String, '/rov/motor_state', cb, 10)

cmd = Twist()
cmd.linear.y = 1.0
cmd.linear.z = 0.5
pub.publish(cmd)
print('TEST: target=0.50m | v5.5.1: 1380+1170 + pitch_safe 15->45')

start = time.time()
last_print = start
while time.time() - start < 22:
    rclpy.spin_once(node, timeout_sec=0.1)
    pub.publish(cmd)
    if time.time() - last_print > 1.0:
        t = time.time() - start
        print(f'T+{t:.0f}s | depth={depth:.3f}m pitch={pitch:.1f}deg yaw={yaw:.1f}deg')
        last_print = time.time()
    time.sleep(0.09)

# STOP
cmd = Twist()
pub.publish(cmd)
print('=== STOP ===')
time.sleep(0.5)
node.destroy_node()
rclpy.shutdown()
