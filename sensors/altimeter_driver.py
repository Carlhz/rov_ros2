#!/usr/bin/env python3
"""
SF 超声波测深仪（高度计）ROS2 驱动
运行于 RK3588，通过 RS485（ttyS5）轮询距离数据，发布到 ROS2 话题。

话题：
  /rov/altitude       std_msgs/Float32  高度/距离（米）
  /rov/altitude_nearest std_msgs/Float32 最近目标距离（米）
  /rov/altitude_raw    std_msgs/Float32  最强目标距离原始值（cm）

部署路径：/opt/ros/rov_ros2_ws/altimeter_driver.py
"""

import os
import time
import struct
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# ── 配置 ──────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyS5'      # SF 测深仪连接的 RS485 端口
BAUDRATE = 9600
DEVICE_ID = 1                   # 机号
FRAME_LEN = 17                  # 响应帧长度
BLIND_ZONE_CM = 20              # 盲区（<20cm 不可靠）


def build_command(dev_id: int) -> bytes:
    """构建测量指令"""
    cmd = bytearray([0xAA, 0xA0, dev_id, 0x00, 0x00])
    checksum = cmd[0] ^ cmd[1] ^ cmd[2] ^ cmd[3] ^ cmd[4]
    cmd.append(checksum)
    return bytes(cmd)


def parse_response(data: bytes):
    """
    解析 17 字节响应帧。
    返回 (nearest_cm, strongest_cm) 或 None。
    """
    if len(data) < FRAME_LEN:
        return None
    if data[0] != 0xAB or data[1] != 0xA0:
        return None

    nearest = (data[4] << 8) | data[5]       # 字节4-5 最近目标
    strongest = (data[8] << 8) | data[9]      # 字节8-9 最强目标
    return nearest, strongest


class AltimeterDriver(Node):
    def __init__(self):
        super().__init__('altimeter_driver')

        # 发布者
        self.pub_altitude = self.create_publisher(Float32, '/rov/altitude', 10)
        self.pub_altitude_nearest = self.create_publisher(Float32, '/rov/altitude_nearest', 10)
        self.pub_altitude_raw = self.create_publisher(Float32, '/rov/altitude_raw', 10)

        # 打开串口
        self.get_logger().info(f'打开串口 {SERIAL_PORT} @ {BAUDRATE} ...')
        try:
            self.ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUDRATE,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=0.5,
            )
        except Exception as e:
            self.get_logger().error(f'串口打开失败: {e}')
            raise

        # 上电后清空残留数据
        self._flush_startup_junk()

        self.cmd = build_command(DEVICE_ID)
        self.timer = self.create_timer(1.0, self.poll_sensor)  # 1Hz
        self.fail_count = 0
        self.get_logger().info('SF 高度计驱动已启动')

    def _flush_startup_junk(self):
        """清空上电残留数据"""
        time.sleep(0.3)
        junk_count = 0
        while True:
            junk = self.ser.read(64)
            if len(junk) == 0:
                break
            junk_count += len(junk)
        if junk_count:
            self.get_logger().info(f'已清除 {junk_count} 字节上电残留数据')

    def poll_sensor(self):
        try:
            self.ser.reset_input_buffer()
            self.ser.write(self.cmd)
            self.ser.flush()
            time.sleep(0.1)

            resp = self.ser.read(FRAME_LEN)

            if len(resp) < FRAME_LEN:
                self.fail_count += 1
                if self.fail_count <= 1:
                    self.get_logger().warn(f'响应不足 {len(resp)}/{FRAME_LEN} 字节')
                return

            result = parse_response(resp)
            if result is None:
                self.fail_count += 1
                if self.fail_count <= 1:
                    self.get_logger().warn('帧头错误或解析失败')
                return

            nearest_cm, strongest_cm = result
            self.fail_count = 0

            # 盲区过滤
            nearest_m = nearest_cm / 100.0
            strongest_m = strongest_cm / 100.0

            if nearest_cm < BLIND_ZONE_CM:
                nearest_m = -1.0  # 标记无效
            if strongest_cm < BLIND_ZONE_CM:
                strongest_m = -1.0  # 标记无效

            # 发布（以最强目标距离为主高度值）
            msg = Float32()
            msg.data = strongest_m
            self.pub_altitude.publish(msg)

            msg.data = nearest_m
            self.pub_altitude_nearest.publish(msg)

            msg.data = float(strongest_cm)
            self.pub_altitude_raw.publish(msg)

            # 每 10 秒打印
            if int(time.time()) % 10 == 0:
                self.get_logger().info(
                    f'最强={strongest_m:.2f}m  最近={nearest_m:.2f}m'
                )

        except Exception as e:
            self.fail_count += 1
            if self.fail_count <= 1:
                self.get_logger().error(f'轮询异常: {e}')

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AltimeterDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
