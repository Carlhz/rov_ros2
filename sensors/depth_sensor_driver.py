#!/usr/bin/env python3
"""
D30 深温计 ROS2 驱动 — MODBUS-RTU 协议
运行于 RK3588，读取 RS485（ttyS3）上的深度和温度数据，发布到 ROS2 话题。

话题：
  /rov/depth          std_msgs/Float32  水深（米）
  /rov/depth_pressure  std_msgs/Float32  压力（MPa）
  /rov/depth_temp      std_msgs/Float32  水温（摄氏度）

部署路径：/opt/ros/rov_ros2_ws/depth_sensor_driver.py
"""

import os
import time
import struct
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# ── 配置 ──────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyS3'      # D30 连接的 RS485 端口
BAUDRATE = 19200                # 默认波特率
DEVICE_ADDR = 0x01              # 设备 MODBUS 地址

# ── MODBUS CRC16 ──────────────────────────────────────
def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_read_cmd(addr: int) -> bytes:
    """构建读取压力+温度的命令帧"""
    cmd = struct.pack('>BBHH', addr, 0x03, 0x8939, 4)
    crc = modbus_crc16(cmd)
    return cmd + struct.pack('<H', crc)


def parse_response(data: bytes):
    """
    解析 13 字节响应帧。
    返回 (pressure_cm, temp_001c) 或 None。
      pressure_cm: 水深厘米值 (int)，÷100 = 米
      temp_001c: 温度 (int)，÷1000 = °C
    """
    if len(data) < 13:
        return None
    addr, func, byte_count = data[0], data[1], data[2]
    if func != 0x03 or byte_count != 8:
        return None

    # 压力 int32（大端）
    pressure = struct.unpack('>i', data[3:7])[0]
    # 温度 int32（大端，带符号）
    temp = struct.unpack('>i', data[7:11])[0]

    # 校验 CRC
    crc_expected = modbus_crc16(data[:11])
    crc_received = struct.unpack('<H', data[11:13])[0]
    if crc_expected != crc_received:
        return None

    return pressure, temp


class DepthSensorDriver(Node):
    def __init__(self):
        super().__init__('depth_sensor_driver')

        # 发布者
        self.pub_depth = self.create_publisher(Float32, '/rov/depth', 10)
        self.pub_pressure = self.create_publisher(Float32, '/rov/depth_pressure', 10)
        self.pub_temp = self.create_publisher(Float32, '/rov/depth_temp', 10)

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

        # 定时轮询
        self.timer = self.create_timer(0.5, self.poll_sensor)  # 2Hz
        self.cmd = build_read_cmd(DEVICE_ADDR)

        self.fail_count = 0
        self.get_logger().info('D30 深温计驱动已启动')

    def poll_sensor(self):
        try:
            # 清空接收缓冲（避免旧数据干扰）
            self.ser.reset_input_buffer()

            # 发送读取命令
            self.ser.write(self.cmd)
            self.ser.flush()

            # 等待响应
            time.sleep(0.05)
            resp = self.ser.read(13)

            if len(resp) < 13:
                self.fail_count += 1
                if self.fail_count <= 1:
                    self.get_logger().warn(f'响应不足 {len(resp)}/13 字节')
                return

            result = parse_response(resp)
            if result is None:
                self.fail_count += 1
                if self.fail_count <= 1:
                    self.get_logger().warn('CRC 校验失败或解析错误')
                return

            pressure_cm, temp_001c = result
            self.fail_count = 0

            # 转换单位
            depth_m = pressure_cm / 100.0          # cm → m
            pressure_mpa = pressure_cm / 10000.0   # cm → MPa
            temp_c = temp_001c / 1000.0            # 0.001°C → °C

            # 发布
            now = self.get_clock().now()
            for pub, val, label in [
                (self.pub_depth, depth_m, 'depth'),
                (self.pub_pressure, pressure_mpa, 'pressure'),
                (self.pub_temp, temp_c, 'temp'),
            ]:
                msg = Float32()
                msg.data = float(val)
                pub.publish(msg)

            # 每 20 次打印一次
            if self.fail_count == 0 and int(time.time()) % 10 == 0:
                self.get_logger().info(
                    f'深度={depth_m:.2f}m 压力={pressure_mpa:.4f}MPa 水温={temp_c:.2f}°C'
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
    node = DepthSensorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
