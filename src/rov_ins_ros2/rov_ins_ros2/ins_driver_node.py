#!/usr/bin/env python3
"""
ROV INS ROS2 驱动节点
运行位置: RK3588 开发板
功能:
  1. 通过 UDP 接收 INS 的 202 字节数据帧 (端口 8008)
  2. 向 INS 发送启停命令 (UDP 8007)
  3. 解析数据并发布到 ROS2 话题
  4. 订阅上位机发来的 INS 控制命令

网络要求:
  - RK3588 需在 192.168.0.x 网段有一个 IP（用于与 INS 通信）
  - 例如: ip addr add 192.168.0.100/24 dev eth0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import socket
import struct
import threading
import math
from typing import Optional

# ROS2 消息类型
from std_msgs.msg import Header, Bool, String
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time

# 自定义消息
from rov_msgs_ros2.msg import InsData, InsCommand


class InsDriverNode(Node):
    """INS ROS2 驱动节点 - 运行在 RK3588 上"""

    FRAME_LENGTH = 202

    # INS 命令字节
    START_CMD = b'\x5A\xA5\x47\x01\x01\x00\x00\x47\x55'
    STOP_CMD  = b'\x5A\xA5\x47\x00\x01\x00\x00\x46\x55'

    def __init__(self):
        super().__init__('ins_driver_node')

        # ── 参数声明 ──────────────────────────────────────────
        self.declare_parameter('local_ip',       '0.0.0.0')
        self.declare_parameter('local_port',     8008)
        self.declare_parameter('ins_ip',         '192.168.0.7')
        self.declare_parameter('ins_cmd_port',   8007)
        self.declare_parameter('frame_id',       'ins_link')
        self.declare_parameter('auto_start',     True)

        self.local_ip     = self.get_parameter('local_ip').value
        self.local_port   = self.get_parameter('local_port').value
        self.ins_ip       = self.get_parameter('ins_ip').value
        self.ins_cmd_port = self.get_parameter('ins_cmd_port').value
        self.frame_id     = self.get_parameter('frame_id').value
        self.auto_start   = self.get_parameter('auto_start').value

        # ── QoS 配置 ──────────────────────────────────────────
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

        # ── 发布者（发给上位机） ──────────────────────────────
        self.ins_pub    = self.create_publisher(InsData,    '/rov/ins/data',   sensor_qos)
        self.imu_pub    = self.create_publisher(Imu,        '/rov/ins/imu',    sensor_qos)
        self.odom_pub   = self.create_publisher(Odometry,   '/rov/ins/odom',   sensor_qos)
        self.navsat_pub = self.create_publisher(NavSatFix,  '/rov/ins/navsat', sensor_qos)
        self.status_pub = self.create_publisher(String,     '/rov/ins/status', reliable_qos)

        # ── 订阅者（接收上位机命令） ──────────────────────────
        self.cmd_sub = self.create_subscription(
            InsCommand, '/rov/ins/command',
            self.cmd_callback, reliable_qos)

        # ── UDP Socket ────────────────────────────────────────
        self.sock: Optional[socket.socket] = None
        self.is_running = False
        self.frame_count = 0

        # ── 启动 ──────────────────────────────────────────────
        self._init_socket()
        if self.auto_start:
            self.send_start()
        self._start_recv_thread()

        self.get_logger().info(
            f'INS Driver Node started\n'
            f'  Listen : {self.local_ip}:{self.local_port}\n'
            f'  INS    : {self.ins_ip}:{self.ins_cmd_port}'
        )

    # ── Socket ────────────────────────────────────────────────
    def _init_socket(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.local_ip, self.local_port))
            self.sock.settimeout(1.0)
            self.get_logger().info(f'UDP socket bound to {self.local_ip}:{self.local_port}')
        except Exception as e:
            self.get_logger().error(f'Socket init failed: {e}')

    def send_command(self, cmd: bytes):
        if self.sock:
            try:
                self.sock.sendto(cmd, (self.ins_ip, self.ins_cmd_port))
                self.get_logger().info(f'Sent: {cmd.hex(" ")}')
            except Exception as e:
                self.get_logger().error(f'Send failed: {e}')

    def send_start(self):
        self.get_logger().info('Sending START command to INS')
        self.send_command(self.START_CMD)

    def send_stop(self):
        self.get_logger().info('Sending STOP command to INS')
        self.send_command(self.STOP_CMD)

    # ── 接收线程 ──────────────────────────────────────────────
    def _start_recv_thread(self):
        self.is_running = True
        t = threading.Thread(target=self._recv_loop, daemon=True)
        t.start()

    def _recv_loop(self):
        while self.is_running:
            try:
                data, _ = self.sock.recvfrom(2048)
                if len(data) == self.FRAME_LENGTH:
                    self._process_frame(data)
                else:
                    self.get_logger().warn(f'Wrong frame length: {len(data)}')
            except socket.timeout:
                continue
            except Exception as e:
                if self.is_running:
                    self.get_logger().error(f'Recv error: {e}')

    # ── 命令回调（来自上位机） ────────────────────────────────
    def cmd_callback(self, msg: InsCommand):
        if msg.command == InsCommand.CMD_START:
            self.send_start()
        elif msg.command == InsCommand.CMD_STOP:
            self.send_stop()
        elif msg.command == InsCommand.CMD_RESET:
            self.send_stop()
            import time; time.sleep(0.1)
            self.send_start()
        self.get_logger().info(f'Received command from topside: {msg.command}')

    # ── 帧解析 ────────────────────────────────────────────────
    def _process_frame(self, frame: bytes):
        self.frame_count += 1
        now = self.get_clock().now().to_msg()

        try:
            ins = self._parse_ins(frame, now)
            self.ins_pub.publish(ins)
            self.imu_pub.publish(self._to_imu(ins))
            self.odom_pub.publish(self._to_odom(ins))
        except Exception as e:
            self.get_logger().error(f'Parse error: {e}')

    def _parse_ins(self, f: bytes, stamp) -> InsData:
        msg = InsData()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.frame_id

        # 状态字节
        msg.work_status       = f[2]
        msg.dvl_calib_status  = f[3]
        msg.gnss_pos_status   = f[4]
        msg.gnss_satellites   = f[5]
        msg.gnss_heading_status = f[6]
        msg.dvl_status        = f[8]

        def fu(i): return struct.unpack('<f', f[i:i+4])[0]

        # 角速度 (deg/s)
        msg.angular_velocity_x = fu(9)
        msg.angular_velocity_y = fu(13)
        msg.angular_velocity_z = fu(17)

        # 加速度 (m/s²)
        msg.linear_acceleration_x = fu(21)
        msg.linear_acceleration_y = fu(25)
        msg.linear_acceleration_z = fu(29)

        # 姿态角 (deg)
        msg.pitch = fu(33)
        msg.roll  = fu(37)
        msg.yaw   = fu(41)

        # 导航速度 (m/s)
        msg.velocity_east        = fu(45)
        msg.velocity_north       = fu(49)
        msg.speed_over_ground    = fu(53)
        msg.track_angle          = fu(57)

        # GNSS 时间
        msg.gnss_date = int(fu(61))
        msg.gnss_time = int(fu(65))

        # GNSS 速度
        msg.gnss_velocity_east  = fu(69)
        msg.gnss_velocity_north = fu(73)
        msg.gnss_altitude       = fu(77)
        msg.gnss_speed          = fu(81)
        msg.gnss_track_angle    = fu(85)

        # GNSS 精度
        msg.gnss_hdop              = fu(89)
        msg.gnss_dual_ant_yaw      = fu(93)
        msg.gnss_pos_update_period = fu(97)

        # DVL 速度 (m/s)
        msg.dvl_sway_velocity  = fu(105)
        msg.dvl_surge_velocity = fu(109)
        msg.dvl_heave_velocity = fu(113)

        # DVL 位移 (m)
        msg.dvl_sway_distance  = fu(117)
        msg.dvl_surge_distance = fu(121)
        msg.dvl_heave_distance = fu(125)

        # 水深/底距 (m)
        msg.dvl_distance_to_bottom = fu(149)

        # 组合导航状态
        msg.combination_status = f[197]
        msg.gnss_valid     = bool(f[197] & 0x01)
        msg.velocity_valid = bool(f[197] & 0x02)
        msg.dvl_valid      = bool(f[197] & 0x04)
        msg.pressure_valid = bool(f[197] & 0x08)

        msg.frame_count = self.frame_count
        return msg

    def _to_imu(self, ins: InsData) -> Imu:
        imu = Imu()
        imu.header = ins.header

        r = math.radians(ins.roll)
        p = math.radians(ins.pitch)
        y = math.radians(ins.yaw)

        # 欧拉角 → 四元数
        cy, sy = math.cos(y/2), math.sin(y/2)
        cp, sp = math.cos(p/2), math.sin(p/2)
        cr, sr = math.cos(r/2), math.sin(r/2)

        imu.orientation.w = cr*cp*cy + sr*sp*sy
        imu.orientation.x = sr*cp*cy - cr*sp*sy
        imu.orientation.y = cr*sp*cy + sr*cp*sy
        imu.orientation.z = cr*cp*sy - sr*sp*cy

        imu.angular_velocity.x = math.radians(ins.angular_velocity_x)
        imu.angular_velocity.y = math.radians(ins.angular_velocity_y)
        imu.angular_velocity.z = math.radians(ins.angular_velocity_z)

        imu.linear_acceleration.x = ins.linear_acceleration_x
        imu.linear_acceleration.y = ins.linear_acceleration_y
        imu.linear_acceleration.z = ins.linear_acceleration_z

        imu.orientation_covariance[0]         = -1.0  # 未知
        imu.angular_velocity_covariance[0]    = -1.0
        imu.linear_acceleration_covariance[0] = -1.0
        return imu

    def _to_odom(self, ins: InsData) -> Odometry:
        odom = Odometry()
        odom.header          = ins.header
        odom.child_frame_id  = 'base_link'
        odom.twist.twist.linear.x = ins.velocity_east
        odom.twist.twist.linear.y = ins.velocity_north
        odom.twist.twist.angular.z = math.radians(ins.angular_velocity_z)
        return odom

    def destroy_node(self):
        self.is_running = False
        self.send_stop()
        if self.sock:
            self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InsDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
