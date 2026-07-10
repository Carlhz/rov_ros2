#!/usr/bin/env python3
"""上传文件到RK3588"""
import paramiko
import sys

HOST = "172.16.28.82"
USER = "root"
PASS = "tronlong"

local_file = sys.argv[1] if len(sys.argv) > 1 else r"D:\Carl_WorkStation\rov_ros2\rk3588\motor_controller.py"
remote_file = sys.argv[2] if len(sys.argv) > 2 else "/opt/ros/rov_ros2_ws/motor_controller.py"

client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(HOST, username=USER, password=PASS, timeout=5)

sftp = client.open_sftp()
sftp.put(local_file, remote_file)
sftp.close()
client.close()

print(f"Uploaded: {local_file} -> {remote_file}")
