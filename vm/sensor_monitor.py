#!/usr/bin/env python3
"""
传感器数据监控节点 — 运行于 VM Ubuntu
订阅 RK3588 发布的深度和高度话题，彩色终端显示。

话题订阅：
  /rov/depth          水深（米）
  /rov/depth_temp      水温（°C）
  /rov/altitude        高度/距离（米）
  /rov/depth_pressure  压力（MPa）

运行方式：
  python3 sensor_monitor.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

DEPTH_TOPIC = '/rov/depth'
TEMP_TOPIC = '/rov/depth_temp'
ALTITUDE_TOPIC = '/rov/altitude'
PRESSURE_TOPIC = '/rov/depth_pressure'


class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')

        self.depth = None
        self.temp = None
        self.altitude = None
        self.pressure = None
        self.depth_age = 999
        self.temp_age = 999
        self.alti_age = 999

        self.sub_depth = self.create_subscription(
            Float32, DEPTH_TOPIC, self.cb_depth, 10)
        self.sub_temp = self.create_subscription(
            Float32, TEMP_TOPIC, self.cb_temp, 10)
        self.sub_altitude = self.create_subscription(
            Float32, ALTITUDE_TOPIC, self.cb_altitude, 10)
        self.sub_pressure = self.create_subscription(
            Float32, PRESSURE_TOPIC, self.cb_pressure, 10)

        self.timer = self.create_timer(1.0, self.print_status)
        self.get_logger().info('传感器监控已启动')

    def cb_depth(self, msg):
        self.depth = msg.data
        self.depth_age = 0

    def cb_temp(self, msg):
        self.temp = msg.data
        self.temp_age = 0

    def cb_altitude(self, msg):
        self.altitude = msg.data
        self.alti_age = 0

    def cb_pressure(self, msg):
        self.pressure = msg.data

    def print_status(self):
        self.depth_age += 1
        self.temp_age += 1
        self.alti_age += 1

        # 构建状态行
        depth_str = f'{self.depth:8.2f} m' if self.depth is not None else '    ---   '
        temp_str = f'{self.temp:7.2f} C' if self.temp is not None else '   ---   '
        alti_str = f'{self.altitude:8.2f} m' if self.altitude is not None else '    ---   '
        pres_str = f'{self.pressure:7.4f} MPa' if self.pressure is not None else '   ---    '

        # 超时标记
        d_flag = '!' if self.depth_age > 5 else ' '
        t_flag = '!' if self.temp_age > 5 else ' '
        a_flag = '!' if self.alti_age > 5 else ' '

        # 彩色输出
        GREEN = '\033[92m'
        YELLOW = '\033[93m'
        CYAN = '\033[96m'
        WHITE = '\033[97m'
        RESET = '\033[0m'
        RED = '\033[91m'

        # 深度行
        d_color = RED if d_flag == '!' else GREEN
        t_color = RED if t_flag == '!' else CYAN
        a_color = RED if a_flag == '!' else YELLOW

        line = (
            f'{d_color}[{d_flag}] 深度:{depth_str}{RESET}  |  '
            f'{t_color}[{t_flag}] 水温:{temp_str}{RESET}  |  '
            f'{a_color}[{a_flag}] 高度:{alti_str}{RESET}  |  '
            f'{WHITE}压力:{pres_str}{RESET}'
        )
        print(f'\r{line}', end='', flush=True)

        # 每 10 行换行
        if int(self.depth_age) % 10 == 0:
            print()


def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
