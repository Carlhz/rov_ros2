#!/usr/bin/env python3
"""ROV 驱动一键部署 - 从 Windows 到 RK3588 + 到 VM (paramiko + scp)"""
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '._lib'))
from paramiko import SSHClient, AutoAddPolicy
from scp import SCPClient

# ── 目标配置 ────────────────────────────────
RK3588 = {"hostname": "172.16.28.82", "username": "root", "password": "159357"}
VM     = {"hostname": "172.16.30.0",   "username": "carl", "password": "159357"}

PROJECT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ── RK3588 部署文件 ──────────────────────────
RK_FILES = [
    ("rk3588/ins_driver_auto.py",       "/opt/ros/rov_ros2_ws/ins_driver_auto.py"),
    ("rk3588/start_all.sh",              "/opt/ros/rov_ros2_ws/start_all.sh"),
    ("rk3588/setup_ip.sh",               "/opt/ros/rov_ros2_ws/setup_ip.sh"),
    ("rk3588/setup_can.sh",              "/opt/ros/rov_ros2_ws/setup_can.sh"),
    ("rk3588/motor_controller.py",       "/opt/ros/rov_ros2_ws/motor_controller.py"),
    ("rk3588/thrust_allocator.py",        "/opt/ros/rov_ros2_ws/thrust_allocator.py"),
    ("rk3588/dvl_driver.py",             "/opt/ros/rov_ros2_ws/dvl_driver.py"),
    ("sensors/depth_sensor_driver.py",   "/opt/ros/rov_ros2_ws/sensors/depth_sensor_driver.py"),
    ("sensors/altimeter_driver.py",      "/opt/ros/rov_ros2_ws/sensors/altimeter_driver.py"),
]

# ── VM 部署文件 ─────────────────────────────
VM_FILES = [
    ("vm/integrated_monitor.py",         "/home/carl/rov_ros2_ws/monitor/integrated_monitor.py"),
    ("vm/joy_controller.py",             "/home/carl/rov_ros2_ws/monitor/joy_controller.py"),
    ("vm/start_joy.sh",                  "/home/carl/rov_ros2_ws/start_joy.sh"),
    ("vm/test_v411.py",                  "/home/carl/rov_ros2_ws/test_v411.py"),
    ("vm/auto_depth_test.py",            "/home/carl/rov_ros2_ws/auto_depth_test.py"),
]


def deploy(target, name, files):
    print(f"\n{'='*60}")
    print(f"  部署到 {name} ({target['hostname']})")
    print(f"{'='*60}")

    ssh = SSHClient()
    ssh.set_missing_host_key_policy(AutoAddPolicy())
    try:
        ssh.connect(**target, timeout=10)
        print(f"已连接 {target['hostname']}\n")

        with SCPClient(ssh.get_transport()) as scp:
            for local_rel, remote_abs in files:
                src = os.path.join(PROJECT, local_rel)
                if os.path.exists(src):
                    # 确保远程目录存在
                    remote_dir = os.path.dirname(remote_abs)
                    ssh.exec_command(f"mkdir -p {remote_dir}")
                    print(f"  上传: {local_rel} -> {remote_abs}")
                    scp.put(src, remote_abs)
                else:
                    print(f"  跳过: {local_rel} (文件不存在)")

        # chmod 可执行文件
        for _, remote_abs in files:
            if remote_abs.endswith('.sh') or remote_abs.endswith('.py'):
                ssh.exec_command(f"chmod +x {remote_abs}")

        print(f"\n[OK] {name} 部署完成")

    except Exception as e:
        print(f"\n[FAIL] {name} 部署失败: {e}")
    finally:
        ssh.close()


def main():
    import argparse
    ap = argparse.ArgumentParser(description='ROV 驱动部署')
    ap.add_argument('--rk3588', action='store_true', default=True, help='部署到 RK3588 (默认)')
    ap.add_argument('--vm',     action='store_true', help='部署到 VM')
    ap.add_argument('--all',    action='store_true', help='部署到全部')
    args = ap.parse_args()

    if args.vm or args.all:
        deploy(VM, "VM Ubuntu", VM_FILES)
    if args.rk3588 or args.all:
        deploy(RK3588, "RK3588", RK_FILES)

    print(f"\n{'='*60}")
    print("  部署完成!")
    print(f"{'='*60}")
    print()
    print("  RK3588 一键启动:")
    print("    ssh root@172.16.28.82")
    print("    cd /opt/ros/rov_ros2_ws")
    print("    ./start_all.sh bg")
    print()
    print("  RK3588 IP 配置 (首次):")
    print("    cd /opt/ros/rov_ros2_ws")
    print("    ./setup_ip.sh")
    print()
    print("  VM 手柄控制:")
    print("    bash ~/rov_ros2_ws/start_joy.sh")
    print()
    print("  VM 监控:")
    print("    source /opt/ros/foxy/setup.bash")
    print("    export ROS_DOMAIN_ID=42")
    print("    python3 ~/rov_ros2_ws/monitor/integrated_monitor.py")
    print()


if __name__ == '__main__':
    main()
