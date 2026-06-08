#!/usr/bin/env python3
"""传感器驱动部署 - 从 Windows 到 RK3588 (paramiko)"""
import sys, os

# 添加本地库路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '._lib'))

from paramiko import SSHClient, AutoAddPolicy
from scp import SCPClient

RK3588 = {"hostname": "172.16.28.82", "username": "root", "password": "159357"}
DEST = "/opt/ros/rov_ros2_ws/"
PROJECT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

FILES = [
    "sensors/depth_sensor_driver.py",
    "sensors/altimeter_driver.py",
    "rk3588/start_sensors.sh",
]

ssh = SSHClient()
ssh.set_missing_host_key_policy(AutoAddPolicy())
print(f"连接 {RK3588['hostname']} ...")
ssh.connect(**RK3588, timeout=10)
print("已连接\n")

with SCPClient(ssh.get_transport()) as scp:
    for f in FILES:
        src = os.path.join(PROJECT, f)
        if os.path.exists(src):
            print(f"上传: {f}")
            scp.put(src, DEST + os.path.basename(f))
        else:
            print(f"跳过: {f} (不存在)")

cmd = f"chmod +x {DEST}start_sensors.sh {DEST}depth_sensor_driver.py {DEST}altimeter_driver.py"
ssh.exec_command(cmd)

print(f"\n=== 部署完成 ===\n")
print(f"在 RK3588 启动:  cd {DEST} && ./start_sensors.sh bg")
print(f"在 VM 监控:      cd ~/rov_ros2_ws/ && python3 vm/sensor_monitor.py")

ssh.close()
