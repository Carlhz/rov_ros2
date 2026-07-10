#!/usr/bin/env python3
"""
全向声纳监控节点 — 运行于 VM Ubuntu (上位机)
订阅 RK3588 发布的 PointCloud2 声纳数据，彩色终端显示实时状态。

话题订阅:
  /sonar/omni/original  — 原始回波点云
  /sonar/omni/rigidity  — 差分刚性检测点云
  /sonar/omni/boundary  — 剖面边界点

运行方式:
  ros2 run rov_sonar_monitor sonar_monitor_node
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rov_sonar_interface.srv import SonarConfig
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import struct
import sys
import os
import time


class SonarMonitorNode(Node):
    """全向声纳监控节点"""

    def __init__(self):
        super().__init__('sonar_monitor_node')

        # QoS (适配 UDP 传输)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 订阅
        self.sub_orig = self.create_subscription(
            PointCloud2, '/sonar/omni/original', self.cb_original, qos)
        self.sub_rig = self.create_subscription(
            PointCloud2, '/sonar/omni/rigidity', self.cb_rigidity, qos)
        self.sub_bnd = self.create_subscription(
            PointCloud2, '/sonar/omni/boundary', self.cb_boundary, qos)

        # 配置服务客户端
        self.config_cli = self.create_client(SonarConfig, '/sonar/omni/config')

        # 统计
        self.orig_count = 0
        self.rig_count = 0
        self.bnd_count = 0
        self.last_angle = 0.0
        self.last_range = 0.0
        self.orig_total = 0
        self.rig_total = 0
        self.orig_age = 0
        self.rig_age = 0
        self.bnd_age = 0

        # 点云字段解析
        self.fields_orig = None
        self.fields_rig = None
        self.fields_bnd = None

        # 控制状态
        self.sonar_on = True

        # 定时器
        self.timer = self.create_timer(1.0, self.print_status)

        self.get_logger().info('=' * 60)
        self.get_logger().info('全向声纳监控节点已启动')
        self.get_logger().info('等待声纳数据...')
        self.get_logger().info('按 h 显示帮助')
        self.get_logger().info('=' * 60)

        # 启动键盘控制
        self.start_keyboard_listener()

    def _parse_fields(self, cloud):
        """记录点云字段偏移"""
        fields = {}
        for f in cloud.fields:
            fields[f.name] = (f.offset, f.datatype)
        return fields

    def _get_point_count(self, cloud):
        return cloud.width * cloud.height

    def _extract_angle_range(self, cloud, fields):
        """从点云数据中估算扫描角度和量程"""
        if self._get_point_count(cloud) == 0:
            return 0.0, 0.0

        off_x = fields.get('x', (0, 0))[0]
        off_y = fields.get('y', (0, 0))[0]
        step = cloud.point_step

        if len(cloud.data) < step:
            return 0.0, 0.0

        # 取第一个和最后一个有效点计算
        try:
            x1 = struct.unpack_from('f', cloud.data, off_x)[0]
            y1 = struct.unpack_from('f', cloud.data, off_y)[0]
            angle = 0.0
            if abs(x1) > 0.001 or abs(y1) > 0.001:
                import math
                angle = math.degrees(math.atan2(-y1, x1))
                if angle < 0:
                    angle += 360

            # 最后一个点
            last_off = (self._get_point_count(cloud) - 1) * step
            if last_off + step <= len(cloud.data):
                x2 = struct.unpack_from('f', cloud.data, last_off + off_x)[0]
                y2 = struct.unpack_from('f', cloud.data, last_off + off_y)[0]
                range_val = math.sqrt(x2 * x2 + y2 * y2)
            else:
                range_val = 0.0

            return angle, range_val
        except Exception:
            return 0.0, 0.0

    def cb_original(self, msg):
        if self.fields_orig is None:
            self.fields_orig = self._parse_fields(msg)
        self.orig_count = self._get_point_count(msg)
        self.orig_total += 1
        self.orig_age = 0
        self.last_angle, self.last_range = self._extract_angle_range(msg, self.fields_orig)

    def cb_rigidity(self, msg):
        if self.fields_rig is None:
            self.fields_rig = self._parse_fields(msg)
        self.rig_count = self._get_point_count(msg)
        self.rig_total += 1
        self.rig_age = 0

    def cb_boundary(self, msg):
        if self.fields_bnd is None:
            self.fields_bnd = self._parse_fields(msg)
        self.bnd_count = self._get_point_count(msg)
        self.bnd_age = 0

    def print_status(self):
        self.orig_age += 1
        self.rig_age += 1
        self.bnd_age += 1

        # 颜色
        G = '\033[92m'   # 绿
        Y = '\033[93m'   # 黄
        C = '\033[96m'   # 青
        W = '\033[97m'   # 白
        R = '\033[91m'   # 红
        RESET = '\033[0m'
        BOLD = '\033[1m'

        o_flag = '!' if self.orig_age > 3 else ' '
        r_flag = '!' if self.rig_age > 3 else ' '
        b_flag = '!' if self.bnd_age > 5 else ' '

        o_color = R if o_flag == '!' else G
        r_color = R if r_flag == '!' else Y
        b_color = R if b_flag == '!' else C

        orig_str = f'{self.orig_count:>5d} 点' if self.orig_age <= 3 else '   ---   '
        rig_str = f'{self.rig_count:>5d} 点' if self.rig_age <= 3 else '   ---   '
        angle_str = f'{self.last_angle:6.1f}°' if self.last_angle > -1 else '  ---  '
        range_str = f'{self.last_range:6.2f}m' if self.last_range > 0 else '  ---  '

        state = f'{G}扫描中{RESET}' if self.sonar_on else f'{R}已停止{RESET}'

        line = (
            f'{o_color}[{o_flag}] 回波:{orig_str}{RESET}  |  '
            f'{r_color}[{r_flag}] 刚性:{rig_str}{RESET}  |  '
            f'{b_color}[{b_flag}] 边界{RESET}  |  '
            f'{W}角度:{angle_str}{RESET}  |  '
            f'{W}量程:{range_str}{RESET}  |  '
            f'状态:{state}'
        )
        print(f'\r{line}', end='', flush=True)

        # 每 10 行换行显示标题
        if self.orig_age % 10 == 0:
            print()
            print(f'{BOLD}{" 来源":>8}  {"点数":>10}  {"角度":>8}  {"量程":>8}{RESET}')

    def config_sonar(self, on_off=None, range_val=None, gain=None,
                     gate=None, sector=None):
        """调用声纳配置服务"""
        if not self.config_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('配置服务不可用')
            return False

        req = SonarConfig.Request()
        req.on_off = on_off if on_off is not None else self.sonar_on
        req.range = range_val if range_val is not None else 0
        req.start_gain = gain if gain is not None else 0
        req.gate = gate if gate is not None else 0
        req.sector_width = sector if sector is not None else 0

        future = self.config_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.result() is not None:
            res = future.result()
            if res.success:
                self.sonar_on = req.on_off
                self.get_logger().info(f'配置成功: {res.message}')
                return True

        self.get_logger().error('配置失败')
        return False

    def start_keyboard_listener(self):
        """启动键盘输入监听（非阻塞线程）"""
        import threading
        self._kb_thread = threading.Thread(target=self._kb_loop, daemon=True)
        self._kb_thread.start()

    def _kb_loop(self):
        """键盘控制循环"""
        if os.name == 'nt':
            import msvcrt
            _getch = lambda: msvcrt.getch().decode('utf-8', errors='ignore')
        else:
            import termios
            import tty
            _getch = self._unix_getch

        help_text = """
        ┌──────────────────────────────────────────┐
        │ 全向声纳控制面板                          │
        ├──────────────────────────────────────────┤
        │ s = 开始扫描    t = 停止扫描              │
        │ 1 = 量程  4m    2 = 量程 10m              │
        │ 3 = 量程 20m    4 = 量程 60m              │
        │ +/= 增益+5dB    - 增益-5dB                │
        │ ] = 扇扫+30°    [ = 扇扫-30°              │
        │ o = 全向(360°)  d = 定向(180°)            │
        │ q = 退出        h = 显示此帮助           │
        └──────────────────────────────────────────┘
        """

        self._gain = 20
        self._sector = 3600

        while rclpy.ok():
            try:
                ch = _getch()
            except Exception:
                time.sleep(0.5)
                continue

            if ch == 'h':
                print(help_text)
            elif ch == 's':
                self.config_sonar(on_off=True)
            elif ch == 't':
                self.config_sonar(on_off=False)
            elif ch == '1':
                self.config_sonar(range_val=4)
                self.get_logger().info('量程设置为 4m')
            elif ch == '2':
                self.config_sonar(range_val=10)
                self.get_logger().info('量程设置为 10m')
            elif ch == '3':
                self.config_sonar(range_val=20)
                self.get_logger().info('量程设置为 20m')
            elif ch == '4':
                self.config_sonar(range_val=60)
                self.get_logger().info('量程设置为 60m')
            elif ch in ('+', '='):
                self._gain = min(40, self._gain + 5)
                self.config_sonar(gain=self._gain)
                self.get_logger().info(f'增益设置为 {self._gain}dB')
            elif ch == '-':
                self._gain = max(0, self._gain - 5)
                self.config_sonar(gain=self._gain)
                self.get_logger().info(f'增益设置为 {self._gain}dB')
            elif ch == ']':
                self._sector = min(3600, self._sector + 300)
                self.config_sonar(sector=self._sector)
                self.get_logger().info(f'扇扫角度设置为 {self._sector // 10}°')
            elif ch == '[':
                self._sector = max(0, self._sector - 300)
                self.config_sonar(sector=self._sector)
                self.get_logger().info(f'扇扫角度设置为 {self._sector // 10}°')
            elif ch == 'o':
                self._sector = 3600
                self.config_sonar(sector=self._sector)
                self.get_logger().info('扇扫角度设置为 360° (全向 PPI)')
            elif ch == 'd':
                self._sector = 1800
                self.config_sonar(sector=self._sector)
                self.get_logger().info('扇扫角度设置为 180° (定向扫描)')
            elif ch == 'q':
                self.get_logger().info('退出监控')
                rclpy.shutdown()
                break

            time.sleep(0.05)

    def _unix_getch(self):
        """Unix 终端单字符读取"""
        import termios
        import tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch


def main(args=None):
    rclpy.init(args=args)
    node = SonarMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
