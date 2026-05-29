#!/usr/bin/env python3
"""
INS Full Monitor Node
Displays comprehensive INS status in terminal
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32, String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
import time
import sys


ALIGN_STATUS = {
    0: ('监控状态',   '\033[33m●\033[0m'),
    1: ('粗对准中',   '\033[33m◐\033[0m'),
    2: ('精对准中',   '\033[36m◑\033[0m'),
    3: ('INS导航',    '\033[32m●\033[0m'),
}

GNSS_FIX = {
    0: ('无定位',     '\033[31m✗\033[0m'),
    1: ('单点定位',   '\033[33m△\033[0m'),
    2: ('差分定位',   '\033[36m◇\033[0m'),
    4: ('RTK固定',    '\033[32m★\033[0m'),
    5: ('RTK浮动',    '\033[33m☆\033[0m'),
}

DVL_CALIB = {
    0: '禁用',
    1: '全姿态标定',
    2: '方位角标定',
}


class INSMonitorFull(Node):
    def __init__(self):
        super().__init__('ins_monitor_full')
        self.d = {}   # data dict

        subs = [
            (Float64, '/ins/latitude',          'lat'),
            (Float64, '/ins/longitude',         'lon'),
            (Float64, '/ins/altitude',          'alt'),
            (Vector3, '/ins/pose',              'pose'),
            (Vector3, '/ins/twist',             'twist'),
            (Imu,     '/ins/imu',               'imu'),
            (Int32,   '/ins/status',            'status'),
            (Int32,   '/ins/work_status',       'work_status'),
            (Int32,   '/ins/align_status',      'align_status'),
            (Int32,   '/ins/gnss_fix_type',     'gnss_fix'),
            (Int32,   '/ins/gnss_satellites',   'gnss_sats'),
            (Float64, '/ins/gnss_hdop',         'gnss_hdop'),
            (Float64, '/ins/gnss_heading',      'gnss_hdg'),
            (Float64, '/ins/gnss_latitude',     'gnss_lat'),
            (Float64, '/ins/gnss_longitude',    'gnss_lon'),
            (Float64, '/ins/gnss_altitude',     'gnss_alt'),
            (Float64, '/ins/gnss_speed',        'gnss_speed'),
            (Float64, '/ins/gnss_track_angle',  'gnss_track'),
            (Float64, '/ins/track_angle',       'track_angle'),
            (Vector3, '/ins/heave',             'heave'),
            (Vector3, '/ins/dvl_velocity',      'dvl_vel'),
            (Float64, '/ins/dvl_depth',         'dvl_depth'),
            (Int32,   '/ins/temperature',       'temp'),
            (Int32,   '/ins/combined_status',   'comb_status'),
            (Int32,   '/ins/calib_sequence',    'calib_seq'),
            (Vector3, '/ins/gnss_std',          'gnss_std'),
            (String,  '/ins/raw',               'raw'),
        ]

        for msg_type, topic, key in subs:
            self.create_subscription(
                msg_type, topic,
                lambda msg, k=key: self._cb(k, msg),
                10
            )

        self.create_timer(0.5, self.display)
        self.t_start = time.time()
        self.frame_count = 0
        self.last_seq = -1
        self.get_logger().info('INS Full Monitor started')

    def _cb(self, key, msg):
        self.d[key] = msg
        if key == 'raw':
            self.frame_count += 1

    def _f(self, key, sub=None, fmt='.4f', default='--'):
        """Get float value from data dict"""
        if key not in self.d:
            return default
        msg = self.d[key]
        if sub:
            val = getattr(msg, sub, None)
        else:
            val = getattr(msg, 'data', msg)
        if val is None:
            return default
        try:
            return f'{float(val):{fmt}}'
        except Exception:
            return str(val)

    def display(self):
        # Clear screen
        sys.stdout.write('\033[2J\033[H')
        sys.stdout.flush()

        W = 70
        elapsed = time.time() - self.t_start
        rate = self.frame_count / elapsed if elapsed > 0 else 0

        def bar(title): print(f'\033[34m{"─"*W}\033[0m  {title}')
        def hdr(title): print(f'\033[1;34m{"═"*W}\033[0m'); print(f'  {title}')
        def row(label, value, unit=''):
            label_str = f'  {label:<22}'
            val_str   = f'\033[1;37m{value}\033[0m'
            unit_str  = f' \033[90m{unit}\033[0m' if unit else ''
            print(f'{label_str}{val_str}{unit_str}')

        # ===== HEADER =====
        hdr(f'\033[1;36m ★ ROV INS 综合状态监控\033[0m')
        ts = self.get_clock().now().to_msg().sec
        print(f'  时间戳: {ts}  |  帧率: \033[32m{rate:.1f} Hz\033[0m  |  总帧: {self.frame_count}')

        # ===== SYSTEM STATUS =====
        bar('【系统状态】')
        work_byte    = self.d.get('work_status')
        align_status = self.d.get('align_status')

        if align_status is not None:
            al = align_status.data
            al_name, al_icon = ALIGN_STATUS.get(al, (str(al), '?'))
            row('导航状态', f'{al_icon} {al_name}')
        else:
            row('导航状态', '--')

        if work_byte is not None:
            wb = work_byte.data
            flags = []
            if wb & 0x80: flags.append('\033[31m陀螺超量程\033[0m')
            if wb & 0x40: flags.append('\033[31m陀螺温度异常\033[0m')
            if wb & 0x20: flags.append('\033[31m加表超量程\033[0m')
            if wb & 0x10: flags.append('\033[31m加表温度异常\033[0m')
            if wb & 0x08: flags.append('\033[31mINS参数失败\033[0m')
            if wb & 0x04: flags.append('\033[31mIMU标定失败\033[0m')
            fault_str = '  '.join(flags) if flags else '\033[32m正常\033[0m'
            row('故障标志', fault_str)
            row('状态字节', f'0x{wb:02X}')
        else:
            row('故障标志', '--')

        comb = self.d.get('comb_status')
        if comb is not None:
            cb = comb.data
            fused = []
            if cb & 0x01: fused.append('\033[32mGNSS\033[0m')
            if cb & 0x02: fused.append('\033[32m里程计\033[0m')
            if cb & 0x04: fused.append('\033[32mDVL\033[0m')
            if cb & 0x08: fused.append('\033[33m零速修正\033[0m')
            row('融合状态', '  '.join(fused) if fused else '\033[90m无融合\033[0m')
        else:
            row('融合状态', '--')

        temp = self.d.get('temp')
        row('内部温度', f'{temp.data if temp else "--"}', '°C')

        # ===== GNSS STATUS =====
        bar('【GNSS状态】')
        gnss_fix  = self.d.get('gnss_fix')
        gnss_sats = self.d.get('gnss_sats')
        gnss_hdop = self.d.get('gnss_hdop')
        gnss_std  = self.d.get('gnss_std')

        if gnss_fix is not None:
            fx = gnss_fix.data
            fx_name, fx_icon = GNSS_FIX.get(fx, (str(fx), '?'))
            row('定位状态', f'{fx_icon} {fx_name}')
        else:
            row('定位状态', '--')

        row('卫星数',   f'{gnss_sats.data if gnss_sats else "--"}', '颗')
        row('HDOP',     self._f('gnss_hdop', fmt='.2f'))
        if gnss_std is not None:
            row('位置精度(σ)', f'纬:{gnss_std.x:.1f}m  经:{gnss_std.y:.1f}m  高:{gnss_std.z:.1f}m')
        else:
            row('位置精度(σ)', '--')
        row('双天线航向', self._f('gnss_hdg', fmt='.2f'), '°')

        # ===== POSITION =====
        bar('【位置信息】')
        row('INS 纬度',  self._f('lat', fmt='.7f'),   '°')
        row('INS 经度',  self._f('lon', fmt='.7f'),   '°')
        row('GNSS 纬度', self._f('gnss_lat', fmt='.7f'), '°')
        row('GNSS 经度', self._f('gnss_lon', fmt='.7f'), '°')
        row('海拔高度',  self._f('alt', fmt='.2f'),   'm')
        row('DVL 深度',  self._f('dvl_depth', fmt='.2f'), 'm')

        # ===== ATTITUDE =====
        bar('【姿态信息】')
        pose = self.d.get('pose')
        if pose:
            row('横滚角 Roll',  f'{pose.x:.3f}', '°')
            row('俯仰角 Pitch', f'{pose.y:.3f}', '°')
            row('航向角 Yaw',   f'{pose.z:.3f}', '°  (北偏东为负)')
        else:
            row('Roll / Pitch / Yaw', '--')
        row('GNSS 航向角', self._f('gnss_track', fmt='.2f'), '°')
        row('INS 航迹角',  self._f('track_angle', fmt='.2f'), '°')

        # ===== VELOCITY =====
        bar('【速度信息】')
        twist = self.d.get('twist')
        if twist:
            spd = (twist.x**2 + twist.y**2)**0.5
            row('北向速度 Vn',  f'{twist.x:.4f}', 'm/s')
            row('东向速度 Ve',  f'{twist.y:.4f}', 'm/s')
            row('天向速度 Vd',  f'{twist.z:.4f}', 'm/s')
            row('水平速度',     f'{spd:.4f}',     'm/s')
        else:
            row('速度', '--')

        heave = self.d.get('heave')
        if heave:
            row('横荡 Sway',  f'{heave.x:.3f}', 'm')
            row('纵荡 Surge', f'{heave.y:.3f}', 'm')
            row('升沉 Heave', f'{heave.z:.3f}', 'm')

        dvl_vel = self.d.get('dvl_vel')
        if dvl_vel:
            row('DVL 纵向',  f'{dvl_vel.x:.4f}', 'm/s')
            row('DVL 横向',  f'{dvl_vel.y:.4f}', 'm/s')
            row('DVL 天向',  f'{dvl_vel.z:.4f}', 'm/s')

        # ===== IMU =====
        bar('【IMU数据】')
        imu = self.d.get('imu')
        if imu:
            row('角速率 Wx', f'{imu.angular_velocity.x:.4f}', 'deg/s')
            row('角速率 Wy', f'{imu.angular_velocity.y:.4f}', 'deg/s')
            row('角速率 Wz', f'{imu.angular_velocity.z:.4f}', 'deg/s')
            row('加速度 Ax', f'{imu.linear_acceleration.x:.4f}', 'm/s²')
            row('加速度 Ay', f'{imu.linear_acceleration.y:.4f}', 'm/s²')
            row('加速度 Az', f'{imu.linear_acceleration.z:.4f}', 'm/s²')
        else:
            row('IMU', '--')

        # ===== FOOTER =====
        print(f'\033[34m{"═"*W}\033[0m')
        print('  按 Ctrl+C 退出')
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = INSMonitorFull()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
