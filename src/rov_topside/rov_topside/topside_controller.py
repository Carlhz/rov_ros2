#!/usr/bin/env python3
"""
ROV 上位机控制节点 (Topside Controller)
运行位置: Ubuntu 20.04 虚拟机（上位机）
功能:
  1. 订阅来自 RK3588 的 INS 数据并显示
  2. 向 RK3588 发送 INS 控制命令（启动/停止）
  3. 提供命令行交互控制界面
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import threading
import math

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from rov_msgs_ros2.msg import InsData, InsCommand


class TopsideControllerNode(Node):
    """上位机控制节点"""

    def __init__(self):
        super().__init__('topside_controller')

        # ── QoS ──────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── 订阅 INS 数据（来自 RK3588） ─────────────────────
        self.create_subscription(InsData,   '/rov/ins/data',   self.ins_data_cb,   sensor_qos)
        self.create_subscription(Imu,       '/rov/ins/imu',    self.imu_cb,        sensor_qos)
        self.create_subscription(Odometry,  '/rov/ins/odom',   self.odom_cb,       sensor_qos)
        self.create_subscription(String,    '/rov/ins/status', self.status_cb,     reliable_qos)

        # ── 发布 INS 命令（发给 RK3588） ─────────────────────
        self.cmd_pub = self.create_publisher(InsCommand, '/rov/ins/command', reliable_qos)

        # ── 状态 ──────────────────────────────────────────────
        self.last_ins: InsData = None
        self.recv_count = 0

        # ── 定时打印状态 ──────────────────────────────────────
        self.create_timer(1.0, self.print_status)

        self.get_logger().info('Topside Controller started, waiting for INS data...')
        self.get_logger().info('Commands: send_start() / send_stop() via ROS2 topic /rov/ins/command')

    # ── 回调 ──────────────────────────────────────────────────
    def ins_data_cb(self, msg: InsData):
        self.last_ins = msg
        self.recv_count += 1

    def imu_cb(self, msg: Imu):
        pass  # 可扩展处理

    def odom_cb(self, msg: Odometry):
        pass

    def status_cb(self, msg: String):
        self.get_logger().info(f'[INS Status] {msg.data}')

    # ── 定时状态输出 ──────────────────────────────────────────
    def print_status(self):
        if self.last_ins is None:
            self.get_logger().warn(f'No INS data received yet (waiting...)')
            return

        ins = self.last_ins
        self.get_logger().info(
            f'\n{"="*50}\n'
            f'  INS Data  (frame #{ins.frame_count}, total recv: {self.recv_count})\n'
            f'{"─"*50}\n'
            f'  Pitch : {ins.pitch:+8.3f} deg   Roll : {ins.roll:+8.3f} deg\n'
            f'  Yaw   : {ins.yaw:+8.3f} deg\n'
            f'  Vel E : {ins.velocity_east:+7.3f} m/s   Vel N: {ins.velocity_north:+7.3f} m/s\n'
            f'  SOG   : {ins.speed_over_ground:7.3f} m/s\n'
            f'  DVL↓  : {ins.dvl_distance_to_bottom:7.3f} m\n'
            f'  GNSS  : {ins.gnss_pos_status} (sats: {ins.gnss_satellites})\n'
            f'  Status: GNSS={ins.gnss_valid} DVL={ins.dvl_valid} VEL={ins.velocity_valid}\n'
            f'{"="*50}'
        )

    # ── 发命令 API ────────────────────────────────────────────
    def send_ins_command(self, command: int):
        msg = InsCommand()
        msg.command = command
        self.cmd_pub.publish(msg)
        names = {InsCommand.CMD_START: 'START', InsCommand.CMD_STOP: 'STOP', InsCommand.CMD_RESET: 'RESET'}
        self.get_logger().info(f'Sent INS command: {names.get(command, str(command))}')

    def send_start(self):
        self.send_ins_command(InsCommand.CMD_START)

    def send_stop(self):
        self.send_ins_command(InsCommand.CMD_STOP)


def main(args=None):
    rclpy.init(args=args)
    node = TopsideControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
