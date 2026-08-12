#!/usr/bin/env python3
"""
ROV 手柄控制器 v5.2 (VM 端) — 简化为纯指令转发
深度PID + 姿态PID 已全部迁移至 motor_controller (RK3588 本地)

v5.0 重大简化:
  - 删除所有PID代码（深度/roll/pitch/yaw → 已移入 motor_controller v4.0）
  - 删除INS订阅（姿态/速度 → motor_controller 本地桥接）
  - 删除深度传感器订阅 → motor_controller 本地桥接
  - dive模式下: linear.z = target_depth(米) 直接传给 motor_controller
  - 保留: 手柄读取、档位管理、按键逻辑、终端显示、CSV记录

数据流:
  VM → /rov/cmd_vel (Twist):
    linear.x  = move         前进/后退 (-1~+1)
    linear.y  = dive_flag    定深标志 (0或1)
    linear.z  = target_depth 目标深度(米) / 手动up_norm
    angular.z = yaw_trim     手动偏航 (-1~+1)

设备: Logitech F710 (D模式 / DirectInput)
  左摇杆 Y (axis[5])  → 上浮/下潜
  右摇杆 X (axis[2])  → 左转/右转
  右摇杆 Y (axis[3])  → 前进/后退
  Y键: 4档内开关悬停 (开启时用当前深度作为目标)
"""

import os
os.environ['ROS_DOMAIN_ID'] = '42'

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int8
import json
import time
import sys
import struct
import fcntl
import threading
import unicodedata
import csv
from datetime import datetime

# ── Joystick 轴映射（F710 D 模式 / DirectInput）─────────────────
AXIS_LX = 4
AXIS_LY = 5      # 左摇杆Y → 浮潜
AXIS_RX = 2      # 右摇杆X → 转向
AXIS_RY = 3      # 右摇杆Y → 前进/后退

AUTO_DETECT = False
AUTO_DETECT_SECS = 5

# ── 按键映射 (F710 D模式实测: X=0, A=1, B=2) ────────────────
BTN_A     = 1
BTN_B     = 2
BTN_X     = 0
BTN_Y     = 3
BTN_LB    = 4
BTN_RB    = 5
BTN_LT    = 6   # 图里 [6] 对应 LT（暂未使用）
BTN_RT    = 7   # 图里 [7] 对应 RT，切换水下灯

# ── 速度档位 ───────────────────────────────────────────────
SPEED_GEARS  = [1200, 1400, 1600]
GEAR_DIVE    = 3
DIVE_FLAG_VAL = 1.0            # 定深标志 (linear.y)
DEFAULT_GEAR = 0
DEADZONE     = 0.08
JS_DEVICE    = '/dev/input/js0'
AXIS_MAX     = 32767.0
MAX_RPM      = 1800
MIN_RPM      = 1100

# ── 深度悬停参数 ───────────────────────────────────────────
DEFAULT_TARGET   = 0.5          # 默认目标深度
DEPTH_STEP       = 0.1          # LB/RB 调整步长
DEPTH_SMOOTH     = 0.5          # 显示用深度平滑
LOG_DIR          = os.path.expanduser('~/rov_ros2_ws/logs')

# ── 定航向参数 ─────────────────────────────────────────────
YAW_STEP         = 5.0          # LB/RB 调整航向步长 (度)


def _str_width(s):
    w = 0
    for c in s:
        e = unicodedata.east_asian_width(c)
        if e in ('F', 'W', 'A'):
            w += 2
        else:
            w += 1
    return w


def _pad_to(s, width, fill=' '):
    pad = width - _str_width(s)
    return s + fill * pad if pad > 0 else s


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class JoystickReader(threading.Thread):
    """后台线程：读取 /dev/input/js0 原始事件"""

    def __init__(self, device=JS_DEVICE):
        super().__init__(daemon=True)
        self.device = device
        self.axes = {}
        self.buttons = {}
        self.lock = threading.Lock()
        self.running = True
        self.connected = False

    def run(self):
        try:
            fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
            self.connected = True
        except Exception as e:
            print('[FATAL] 无法打开 {}: {}'.format(self.device, e))
            return

        flags = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)

        buf = b''
        while self.running:
            try:
                chunk = os.read(fd, 64)
                if chunk:
                    buf += chunk
                else:
                    time.sleep(0.01)
                    continue
            except BlockingIOError:
                time.sleep(0.01)
                continue
            except OSError:
                break

            while len(buf) >= 8:
                event = buf[:8]
                buf = buf[8:]
                t_ms, value, etype, number = struct.unpack('IhBB', event)
                etype &= ~0x80

                with self.lock:
                    if etype == 2:
                        self.axes[number] = max(-1.0, min(1.0, value / AXIS_MAX))
                    elif etype == 1:
                        self.buttons[number] = bool(value)

        os.close(fd)
        self.connected = False

    def get_state(self):
        with self.lock:
            return dict(self.axes), dict(self.buttons)

    def stop(self):
        self.running = False


class JoyController(Node):
    def __init__(self):
        super().__init__('rov_joy_controller')

        self.cmd_pub   = self.create_publisher(Twist,  '/rov/cmd_vel',   10)
        self.state_pub = self.create_publisher(String, '/rov/joy_state', 10)
        self.light_pub = self.create_publisher(String, '/rov/light_cmd', 10)

        # ── v5.0: 订阅 motor_controller 状态 (用于显示电机转速和深度) ──
        self.sub_motor = self.create_subscription(
            String, '/rov/motor_state', self._cb_motor, 10)

        self.gear       = DEFAULT_GEAR
        self.e_stopped  = False
        self.last_axes  = [0.0, 0.0, 0.0]
        self.debug_mode = '--debug' in sys.argv
        self.scan_mode  = '--scan' in sys.argv

        # 轴映射
        self.lx = AXIS_LX; self.ly = AXIS_LY
        self.rx = AXIS_RX; self.ry = AXIS_RY
        try:
            for ax_name in ('lx', 'ly', 'rx', 'ry'):
                flag = '--{}'.format(ax_name)
                if flag in sys.argv:
                    idx = sys.argv.index(flag)
                    setattr(self, ax_name, int(sys.argv[idx + 1]))
        except (ValueError, IndexError):
            pass

        # 显示
        self._display_lines = 0
        self._status_msg = ''
        self._status_until = 0.0

        # ── v5.0: 深度状态 (从 motor_state 获取, 仅显示) ──
        self.current_depth   = 0.0
        self.depth_valid     = False
        self.filtered_depth  = 0.0
        self.depth_hold_on   = False
        self.target_depth    = DEFAULT_TARGET

        # ── v5.1: 定航向 (独立于定深) ──
        self.yaw_hold_on     = False
        self.yaw_hold_target = 0.0  # 目标航向 (度)

        # ── v5.2: 水下灯状态 ──
        self.sub_light = self.create_subscription(
            Int8, '/rov/light_state', self._cb_light, 10)
        self.light_state = 0       # 0=关, 1=半亮, 2=全亮
        self.light_age   = 999.0   # 状态回显超时
        self.light_names = {0: '关', 1: '半亮', 2: '全亮'}

        # ── v5.0: 姿态+电机状态 (从 motor_state 获取, 仅显示) ──
        self.ins_yaw      = 0.0
        self.ins_pitch    = 0.0
        self.ins_roll     = 0.0
        self.ins_att_valid = False
        # v8.1: INS 加速度 + 角速度 (CSV 记录用)
        self.ins_ax = 0.0
        self.ins_ay = 0.0
        self.ins_az = 0.0
        self.ins_wx = 0.0
        self.ins_wy = 0.0
        self.ins_wz = 0.0
        self.ins_ve = 0.0
        self.ins_vn = 0.0
        self.ins_vd = 0.0
        self.depth_pid_out = 0.0
        self.roll_pid_out  = 0.0
        self.pitch_pid_out = 0.0
        self.yaw_pid_out   = 0.0
        self.yaw_target    = 0.0
        self.yaw_captured  = False
        self.last_motor_rpms = {0: 0, 1: 0, 2: 0, 3: 0, 5: 0, 6: 0, 7: 0}
        self.motor_rpm_age = 999.0

        # ── CSV 记录 ──
        self.csv_file       = None
        self.csv_path       = None
        self.log_start_time = 0.0

        # ── 定航向 CSV 记录 ──
        self.yaw_csv_file       = None
        self.yaw_csv_path       = None
        self.yaw_log_start_time = 0.0

        # ── 启动 joystick ──
        self.js = JoystickReader(JS_DEVICE)
        self.js.start()
        time.sleep(0.5)
        if not self.js.connected:
            self.get_logger().fatal('无法连接手柄 {}'.format(JS_DEVICE))
            raise RuntimeError('Joystick not found')

        if AUTO_DETECT:
            self._auto_detect_axes()
        if self.scan_mode:
            self._run_axis_scanner()
            raise RuntimeError('扫描完成')

        # 主循环 20Hz
        self.create_timer(0.05, self.heartbeat_timer)

        self.get_logger().info('=' * 60)
        self.get_logger().info('  ROV 手柄控制器 v5.2 — 纯指令转发 (PID在RK3588本地)')
        self.get_logger().info('  定深: linear.z=target_depth(米), linear.y=flag')
        self.get_logger().info('  档位: 4档=定深, 1~3档=手动  RT:切换水下灯')
        self.get_logger().info('  设备: {}'.format(JS_DEVICE))
        self.get_logger().info('=' * 60)
        self._publish_state('ready', 'v5.2 纯指令转发就绪')

    # ═══════════════════════════════════════════════════
    # 回调: 从 motor_state 获取所有状态
    # ═══════════════════════════════════════════════════

    def _cb_motor(self, msg: String):
        try:
            s = json.loads(msg.data)
            now = time.time()

            # 电机转速
            if 'motors' in s:
                m = s['motors']
                for k in [0, 1, 2, 3, 5, 6, 7]:
                    self.last_motor_rpms[k] = m.get(str(k), m.get(k, 0))
            self.motor_rpm_age = 0.0

            # 深度 (v4.0 motor_state 包含)
            if 'current_depth' in s and s.get('depth_valid'):
                raw = float(s['current_depth'])
                if self.depth_valid:
                    self.filtered_depth = DEPTH_SMOOTH * raw + (1.0 - DEPTH_SMOOTH) * self.filtered_depth
                else:
                    self.filtered_depth = raw
                self.current_depth = self.filtered_depth
                self.depth_valid = True
            elif 'current_depth' not in s:
                self.depth_valid = False

            # 姿态
            if 'ins_yaw' in s and s.get('ins_att_valid'):
                self.ins_yaw = float(s['ins_yaw'])
                self.ins_pitch = float(s.get('ins_pitch', 0))
                self.ins_roll = float(s.get('ins_roll', 0))
                self.ins_att_valid = True

            # v8.1: INS 加速度 + 角速度
            self.ins_ax = float(s.get('ins_ax', 0))
            self.ins_ay = float(s.get('ins_ay', 0))
            self.ins_az = float(s.get('ins_az', 0))
            self.ins_wx = float(s.get('ins_wx', 0))
            self.ins_wy = float(s.get('ins_wy', 0))
            self.ins_wz = float(s.get('ins_wz', 0))
            self.ins_ve = float(s.get('ins_ve', 0))
            self.ins_vn = float(s.get('ins_vn', 0))
            self.ins_vd = float(s.get('ins_vd', 0))

            # PID 输出
            self.depth_pid_out = float(s.get('depth_pid_out', 0))
            self.roll_pid_out = float(s.get('roll_pid_out', 0))
            self.pitch_pid_out = float(s.get('pitch_pid_out', 0))
            self.yaw_pid_out = float(s.get('yaw_pid_out', 0))
            self.yaw_target = float(s.get('yaw_target', 0))
            self.yaw_captured = bool(s.get('yaw_captured', False))
            # 注意: yaw_hold_target 不从 motor_state 读取, 由本地管理 (避免覆盖用户调整)

        except Exception:
            pass

    def _cb_light(self, msg: Int8):
        """接收 ttyS5_modbus_hub 回显的灯状态"""
        code = int(msg.data)
        if code in self.light_names:
            self.light_state = code
            self.light_age = 0.0

    # ═══════════════════════════════════════════════════
    # 深度悬停开关
    # ═══════════════════════════════════════════════════

    def _toggle_depth_hold(self):
        if not self.depth_hold_on:
            self.depth_hold_on = True
            # 用当前显示深度作为目标 (motor_controller 会通过motor_state反馈)
            self.target_depth = self.current_depth if self.depth_valid else DEFAULT_TARGET
            self._start_csv_log()
            self._set_status('悬停开启 目标:{:.2f}m (PID由RK3588本地执行)'.format(
                self.target_depth), 3.0)
            self._publish_state('depth_hold', 'on_depth_{:.2f}'.format(self.target_depth))
        else:
            self.depth_hold_on = False
            self._stop_csv_log()
            self._set_status('悬停关闭', 3.0)
            self._publish_state('depth_hold', 'off')

    def _toggle_yaw_hold(self):
        """切换定航向 (独立于定深)"""
        if not self.yaw_hold_on:
            self.yaw_hold_on = True
            # 捕获当前航向作为目标
            self.yaw_hold_target = self.ins_yaw if self.ins_att_valid else 0.0
            self._start_yaw_csv_log()
            self._set_status('定航向已开启 目标:{:.0f}deg'.format(self.yaw_hold_target), 3.0)
            self._publish_state('yaw_hold', 'on_{:.0f}'.format(self.yaw_hold_target))
        else:
            self.yaw_hold_on = False
            self._stop_yaw_csv_log()
            self._set_status('定航向已关闭', 3.0)
            self._publish_state('yaw_hold', 'off')

    def _cycle_light(self):
        """RT 按下时循环切换水下灯: 关→半亮→全亮→关"""
        cycle = {0: 'half', 1: 'full', 2: 'off'}
        next_state = cycle.get(self.light_state, 'half')
        self.light_state = {0: 1, 1: 2, 2: 0}.get(self.light_state, 1)
        m = String()
        m.data = next_state
        self.light_pub.publish(m)
        self._set_status('水下灯 -> {}'.format(self.light_names[self.light_state]), 2.0)

    def _adjust_yaw_target(self, delta):
        """调整目标航向 (±5度)"""
        self.yaw_hold_target += delta
        # 归一化到 [-180, 180]
        while self.yaw_hold_target > 180.0:
            self.yaw_hold_target -= 360.0
        while self.yaw_hold_target < -180.0:
            self.yaw_hold_target += 360.0
        self._set_status('目标航向: {:.0f}deg'.format(self.yaw_hold_target), 2.0)

    def _adjust_target_depth(self, delta):
        if not self.depth_hold_on:
            return
        self.target_depth = _clamp(self.target_depth + delta, 0.0, 100.0)
        self._set_status('目标深度: {:.2f}m'.format(self.target_depth), 2.0)
        self._publish_state('depth_target', '{:.2f}'.format(self.target_depth))

    # ═══════════════════════════════════════════════════
    # CSV (v5.0: 记录 motor_state 回传的实际数据)
    # ═══════════════════════════════════════════════════

    def _start_csv_log(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(LOG_DIR, 'depth_hold_{}.csv'.format(ts))
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'elapsed_sec', 'target_depth_m', 'actual_depth_m',
            'depth_pid', 'roll_pid', 'pitch_pid', 'yaw_pid',
            'ins_roll_deg', 'ins_pitch_deg', 'ins_yaw_deg',
            'ins_ax', 'ins_ay', 'ins_az',
            'ins_wx', 'ins_wy', 'ins_wz',
            'ins_ve', 'ins_vn', 'ins_vd',
            'id0_rpm', 'id1_rpm', 'id2_rpm', 'id3_rpm',
            'id5_rpm', 'id6_rpm', 'id7_rpm'
        ])
        self.log_start_time = time.time()
        self.get_logger().info('CSV 记录开始: {}'.format(self.csv_path))

    def _stop_csv_log(self):
        if self.csv_file is None:
            return
        self.csv_file.close()
        self.csv_file = None
        self.get_logger().info('CSV 记录停止: {}'.format(self.csv_path))
        self._set_status('CSV已保存: {}'.format(os.path.basename(self.csv_path)), 5.0)

    def _write_csv_row(self, elapsed, target_d, actual_d,
                       dp, rp, pp, yp, v_roll, v_pitch, v_yaw,
                       ax, ay, az, wx, wy, wz, ve, vn, vd, rpms):
        if self.csv_file is None:
            return
        try:
            self.csv_writer.writerow(
                ['{:.3f}'.format(elapsed),
                 '{:.3f}'.format(target_d), '{:.3f}'.format(actual_d),
                 '{:.4f}'.format(dp), '{:.4f}'.format(rp),
                 '{:.4f}'.format(pp), '{:.4f}'.format(yp),
                 '{:.2f}'.format(v_roll), '{:.2f}'.format(v_pitch),
                 '{:.2f}'.format(v_yaw),
                 '{:.4f}'.format(ax), '{:.4f}'.format(ay), '{:.4f}'.format(az),
                 '{:.4f}'.format(wx), '{:.4f}'.format(wy), '{:.4f}'.format(wz),
                 '{:.4f}'.format(ve), '{:.4f}'.format(vn), '{:.4f}'.format(vd)]
                + [str(r) for r in rpms])
            self.csv_file.flush()
        except Exception:
            pass

    # ── 定航向 CSV ──

    def _start_yaw_csv_log(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.yaw_csv_path = os.path.join(LOG_DIR, 'yaw_hold_{}.csv'.format(ts))
        self.yaw_csv_file = open(self.yaw_csv_path, 'w', newline='', encoding='utf-8')
        self.yaw_csv_writer = csv.writer(self.yaw_csv_file)
        self.yaw_csv_writer.writerow([
            'elapsed_sec', 'target_yaw_deg', 'actual_yaw_deg', 'yaw_error_deg',
            'yaw_pid', 'roll_pid', 'pitch_pid', 'depth_pid',
            'ins_roll_deg', 'ins_pitch_deg', 'ins_yaw_deg',
            'ins_ax', 'ins_ay', 'ins_az',
            'ins_wx', 'ins_wy', 'ins_wz',
            'ins_ve', 'ins_vn', 'ins_vd',
            'id0_rpm', 'id1_rpm', 'id2_rpm', 'id3_rpm',
            'id5_rpm', 'id6_rpm', 'id7_rpm'
        ])
        self.yaw_log_start_time = time.time()
        self.get_logger().info('定航向CSV 记录开始: {}'.format(self.yaw_csv_path))

    def _stop_yaw_csv_log(self):
        if self.yaw_csv_file is None:
            return
        self.yaw_csv_file.close()
        self.yaw_csv_file = None
        self.get_logger().info('定航向CSV 记录停止: {}'.format(self.yaw_csv_path))
        self._set_status('定航向CSV已保存: {}'.format(os.path.basename(self.yaw_csv_path)), 5.0)

    def _write_yaw_csv_row(self, elapsed, target_y, actual_y, yaw_err,
                           yp, rp, pp, dp, v_roll, v_pitch, v_yaw,
                           ax, ay, az, wx, wy, wz, ve, vn, vd, rpms):
        if self.yaw_csv_file is None:
            return
        try:
            self.yaw_csv_writer.writerow(
                ['{:.3f}'.format(elapsed),
                 '{:.1f}'.format(target_y), '{:.1f}'.format(actual_y),
                 '{:.1f}'.format(yaw_err),
                 '{:.4f}'.format(yp), '{:.4f}'.format(rp),
                 '{:.4f}'.format(pp), '{:.4f}'.format(dp),
                 '{:.2f}'.format(v_roll), '{:.2f}'.format(v_pitch),
                 '{:.2f}'.format(v_yaw),
                 '{:.4f}'.format(ax), '{:.4f}'.format(ay), '{:.4f}'.format(az),
                 '{:.4f}'.format(wx), '{:.4f}'.format(wy), '{:.4f}'.format(wz),
                 '{:.4f}'.format(ve), '{:.4f}'.format(vn), '{:.4f}'.format(vd)]
                + [str(r) for r in rpms])
            self.yaw_csv_file.flush()
        except Exception:
            pass

    # ═══════════════════════════════════════════════════
    # 辅助
    # ═══════════════════════════════════════════════════

    def _set_status(self, msg, duration=3.0):
        self._status_msg = msg
        self._status_until = time.time() + duration

    def apply_deadzone(self, v):
        if abs(v) < DEADZONE:
            return 0.0
        sign = 1.0 if v > 0 else -1.0
        return sign * (abs(v) - DEADZONE) / (1.0 - DEADZONE)

    def _bar(self, val, width=30):
        half = width // 2
        blocks = int(abs(val) * half + 0.5)
        if val > 0.01:
            return ' ' * half + '|' + '\u2588' * blocks + '\u2591' * (half - blocks)
        elif val < -0.01:
            return '\u2591' * (half - blocks) + '\u2588' * blocks + '|' + ' ' * half
        else:
            return '\u2591' * half + '|' + '\u2591' * half

    def _motor_bar(self, rpm_val, width=6):
        h = width // 2
        ratio = min(abs(rpm_val) / MAX_RPM, 1.0)
        blocks = int(ratio * h + 0.5)
        if rpm_val > 0:
            return ' ' * (h - blocks) + '\u2588' * blocks + '|' + '\u2591' * h
        elif rpm_val < 0:
            return '\u2591' * h + '|' + '\u2588' * blocks + ' ' * (h - blocks)
        else:
            return '\u2591' * h + '|' + '\u2591' * h

    def _mrpm(self, k):
        if self.motor_rpm_age > 2.0:
            return ' ID{:}:  ---'.format(k)
        v = self.last_motor_rpms.get(k, 0)
        return ' ID{:}:{:+5d} {}'.format(k, v, self._motor_bar(v))

    def _display_status(self, move, yaw, target_d, active_btns, is_dive_gear):
        INNER = 60
        now = time.time()

        # 标题行
        estop_mark = ' !!急停!!' if self.e_stopped else ''
        if is_dive_gear:
            gear_txt = 'ROV-JOY v5.0 [4档/定深] PID@RK3588  {}'.format(estop_mark)
        else:
            max_r = SPEED_GEARS[self.gear]
            gear_txt = 'ROV-JOY v5.0 档位{}/3  尾{:4d}  垂{:4d}  {}'.format(
                self.gear + 1, max_r, min(max_r + 600, MAX_RPM), estop_mark)

        # 深度行
        d_cur = '{:5.2f}'.format(self.current_depth) if self.depth_valid else '--.--'
        d_tar = '{:5.2f}'.format(self.target_depth)
        hold_str = '悬停:{}  深:{}m  目标:{}m'.format(
            'ON ' if self.depth_hold_on else 'OFF', d_cur, d_tar if self.depth_hold_on else '--')
        err_str = ''
        if self.depth_hold_on and self.depth_valid:
            err_str = '  err:{:+.2f}m'.format(self.target_depth - self.current_depth)

        # 姿态行
        if self.ins_att_valid:
            r_str = '{:+6.1f}'.format(self.ins_roll)
            p_str = '{:+6.1f}'.format(self.ins_pitch)
            y_str = '{:+6.1f}'.format(self.ins_yaw)
        else:
            r_str = '  --.-'; p_str = '  --.-'; y_str = '  --.-'

        att_line = '姿态: Roll{}deg Pitch{}deg Yaw{}deg'.format(r_str, p_str, y_str)
        if self.yaw_hold_on:
            att_line += ' [定航:{:.0f}deg]'.format(self.yaw_hold_target)

        # PID 行
        pid_line = 'PID: D{:+5.2f} R{:+5.2f} P{:+5.2f} Y{:+5.2f}  yaw目标:{:.0f}deg'.format(
            self.depth_pid_out, self.roll_pid_out, self.pitch_pid_out, self.yaw_pid_out,
            self.yaw_target)

        # 推力行
        bar_area = INNER - 18
        bar = self._bar(move, width=bar_area)
        m_line = '前进/后退  [--] {}'.format(bar)
        bar = self._bar(yaw, width=bar_area)
        y_line = '左/右转向  [--] {}'.format(bar)

        u_line = '上浮/下潜  [--] (PID自动)'

        # 电机行
        motor_row1 = self._mrpm(0) + self._mrpm(1) + self._mrpm(2) + self._mrpm(3)
        motor_row2 = self._mrpm(5) + self._mrpm(6) + self._mrpm(7)

        # 按键
        btn_str = ', '.join(active_btns) if active_btns else '(无)'
        btn_line = '按键: {}'.format(btn_str)
        yh_status = 'ON' if self.yaw_hold_on else 'OFF'
        if self.yaw_hold_on:
            lb_rb_hint = '调航向±{}deg'.format(int(YAW_STEP))
        elif self.gear == GEAR_DIVE and self.depth_hold_on:
            lb_rb_hint = '调目标(悬停)'
        else:
            lb_rb_hint = '降/升档'
        hint_line = 'A:急停 B:恢复 X:定航({}) LB/RB:{} Y:{}'.format(
            yh_status, lb_rb_hint,
            '开关悬停' if self.gear == GEAR_DIVE else '需升到4档')
        light_line = '水下灯: [{}]  (RT切换)'.format(
            self.light_names.get(self.light_state, '未知'))

        lines = [
            '+' + '-' * INNER + '+',
            '| ' + _pad_to(gear_txt, INNER - 2) + ' |',
            '| ' + _pad_to(hold_str + err_str, INNER - 2) + ' |',
            '| ' + _pad_to(att_line, INNER - 2) + ' |',
            '| ' + _pad_to(pid_line, INNER - 2) + ' |',
            '+' + '-' * INNER + '+',
            '| ' + _pad_to(m_line, INNER - 2) + ' |',
            '| ' + _pad_to(y_line, INNER - 2) + ' |',
            '| ' + _pad_to(u_line, INNER - 2) + ' |',
            '+' + '-' * INNER + '+',
            '| ' + _pad_to('电机: ' + motor_row1.lstrip(), INNER - 2) + ' |',
            '| ' + _pad_to('      ' + motor_row2.lstrip(), INNER - 2) + ' |',
            '+' + '-' * INNER + '+',
            '| ' + _pad_to(btn_line, INNER - 2) + ' |',
            '| ' + _pad_to(hint_line, INNER - 2) + ' |',
            '| ' + _pad_to(light_line, INNER - 2) + ' |',
        ]

        if self._status_msg and time.time() < self._status_until:
            lines.append('+' + '-' * INNER + '+')
            lines.append('| *' + _pad_to(self._status_msg, INNER - 3) + '|')
        elif self._status_msg:
            self._status_msg = ''

        lines.append('+' + '-' * INNER + '+')

        if self._display_lines > 0:
            sys.stdout.write('\033[{}A'.format(self._display_lines))
        for line in lines:
            sys.stdout.write('\033[K{}\n'.format(line))
        sys.stdout.flush()
        self._display_lines = len(lines)

    # ═══════════════════════════════════════════════════
    # 按键处理
    # ═══════════════════════════════════════════════════

    def _debounce_btn(self, current, last, idx):
        if idx >= len(current) or idx >= len(last):
            return False
        return current[idx] == 1 and last[idx] == 0

    def _auto_detect_axes(self):
        self.get_logger().info('轴自动检测中...')
        axis_ranges = {}
        start = time.time()
        while time.time() - start < AUTO_DETECT_SECS:
            axes, _ = self.js.get_state()
            for num, val in axes.items():
                if num not in axis_ranges:
                    axis_ranges[num] = [val, val, 0.0]
                else:
                    r = axis_ranges[num]
                    r[0] = min(r[0], val); r[1] = max(r[1], val); r[2] += abs(val)
            time.sleep(0.05)
        active = [(n, vmax-vmin) for n, (vmin, vmax, _) in sorted(axis_ranges.items())
                  if vmax - vmin > 0.25]
        active.sort(key=lambda x: x[1], reverse=True)
        top4 = sorted([a[0] for a in active[:4]])
        if len(top4) >= 4:
            self.lx = top4[0]; self.ly = top4[1]
            self.rx = top4[2]; self.ry = top4[3]
        self.get_logger().info('  LX={} LY={} RX={} RY={}'.format(
            self.lx, self.ly, self.rx, self.ry))

    def _run_axis_scanner(self):
        btn_names = ['X', 'A', 'B', 'Y', 'LB', 'RB', 'LT', 'RT', 'Logo', 'L3', 'R3']
        print('=== F710 按键/轴扫描 (Ctrl+C 退出) ===')
        print()
        try:
            while True:
                axes, btns = self.js.get_state()
                lines = []

                # ── 轴 ──
                lines.append('-- 轴 (AXIS) ' + '-' * 24)
                for i in range(8):
                    val = axes.get(i, 0.0)
                    marker = ''
                    if i == self.lx: marker = '  LX'
                    elif i == self.ly: marker = '  LY'
                    elif i == self.rx: marker = '  RX'
                    elif i == self.ry: marker = '  RY'
                    bar = '\u2588' * int(abs(val) * 15) if abs(val) > 0.01 else ''
                    d = '\u2192' if val > 0.01 else ('\u2190' if val < -0.01 else '\u00b7')
                    lines.append('  axis[{}] {} {:+.3f} {}{}'.format(i, d, val, bar, marker))

                lines.append('')

                # ── 按键 ──
                lines.append('-- 按键 (BUTTON) ' + '-' * 20)
                for i in range(0, 11, 2):
                    row = ''
                    for j in range(2):
                        idx = i + j
                        if idx >= 11:
                            break
                        name = btn_names[idx] if idx < len(btn_names) else '---'
                        pressed = btns.get(idx, False)
                        tag = '[ON] ' if pressed else ' off '
                        row += '  [{:2d}] {:5s} {} '.format(idx, name, tag)
                    lines.append(row)

                n = len(lines)
                for line in lines:
                    sys.stdout.write('\033[K{}\n'.format(line))
                sys.stdout.write('\033[{}A'.format(n))
                sys.stdout.flush()
                time.sleep(0.1)
        except KeyboardInterrupt:
            sys.stdout.write('\033[{}B\033[J'.format(len(lines)))
            sys.stdout.flush()

    # ═══════════════════════════════════════════════════
    # 主循环 (20Hz)
    # ═══════════════════════════════════════════════════

    def heartbeat_timer(self):
        axes, btns = self.js.get_state()
        now = time.time()

        # 摇杆
        raw_up   = axes.get(self.ly, 0.0)
        raw_move = axes.get(self.ry, 0.0)   # v5.1: 去掉取反, 修正电机前进方向
        raw_yaw  = axes.get(self.rx, 0.0)

        move = self.apply_deadzone(raw_move)
        yaw  = self.apply_deadzone(raw_yaw)
        up   = self.apply_deadzone(raw_up)

        btn_arr = [1 if btns.get(i, 0) else 0 for i in range(11)]

        # ── 按键 ──
        # 急停 (仅 A 键)
        if btn_arr[BTN_A]:
            if not self.e_stopped:
                self.e_stopped = True
                was_holding = self.depth_hold_on
                self.depth_hold_on = False
                was_yaw_hold = self.yaw_hold_on
                self.yaw_hold_on = False
                if was_holding:
                    self._stop_csv_log()
                if was_yaw_hold:
                    self._stop_yaw_csv_log()
                self._set_status('急停! 所有电机停止 (按B恢复)', 5.0)
                self._publish_state('e_stop', '急停')

        # 恢复
        if btn_arr[BTN_B] and self.e_stopped:
            self.e_stopped = False
            self._set_status('急停解除', 3.0)
            self._publish_state('resume', '控制已恢复')

        # LB/RB — 优先级: 定航向 > 定深目标 > 档位
        if self.yaw_hold_on:
            # 定航向开启时: LB/RB 调整目标航向
            if self._debounce_btn(btn_arr, self._last_btns if hasattr(self, '_last_btns') else [0]*11, BTN_LB):
                self._adjust_yaw_target(-YAW_STEP)
            if self._debounce_btn(btn_arr, self._last_btns if hasattr(self, '_last_btns') else [0]*11, BTN_RB):
                self._adjust_yaw_target(+YAW_STEP)
        elif self.gear == GEAR_DIVE:
            if self.depth_hold_on:
                if self._debounce_btn(btn_arr, self._last_btns if hasattr(self, '_last_btns') else [0]*11, BTN_LB):
                    self._adjust_target_depth(-DEPTH_STEP)
                if self._debounce_btn(btn_arr, self._last_btns if hasattr(self, '_last_btns') else [0]*11, BTN_RB):
                    self._adjust_target_depth(+DEPTH_STEP)
        else:
            if self._debounce_btn(btn_arr, self._last_btns if hasattr(self, '_last_btns') else [0]*11, BTN_RB):
                if self.gear < GEAR_DIVE:
                    self.gear += 1
                    msg = '升档 -> 4档[定深] 按Y开关悬停' if self.gear == GEAR_DIVE else \
                          '升档 -> {}/3档 ({}rpm)'.format(self.gear+1, SPEED_GEARS[self.gear])
                    self._set_status(msg, 3.0)
                    self._publish_state('gear', '{}档'.format(self.gear+1))
            if self._debounce_btn(btn_arr, self._last_btns if hasattr(self, '_last_btns') else [0]*11, BTN_LB):
                if self.gear > 0:
                    was_dive = (self.gear == GEAR_DIVE)
                    self.gear -= 1
                    if was_dive:
                        if self.depth_hold_on:
                            self.depth_hold_on = False
                            self._stop_csv_log()
                        self._set_status('退出定深档 -> 3档', 3.0)
                    self._publish_state('gear', '{}档'.format(self.gear+1))

        # Y键: 开关悬停
        if self._debounce_btn(btn_arr, self._last_btns if hasattr(self, '_last_btns') else [0]*11, BTN_Y):
            if self.gear == GEAR_DIVE:
                self._toggle_depth_hold()

        # X键: 开关定航向 (独立于定深, 任何档位可用)
        if self._debounce_btn(btn_arr, self._last_btns if hasattr(self, '_last_btns') else [0]*11, BTN_X):
            self._toggle_yaw_hold()

        # RT键 (按钮7): 循环切换水下灯
        if self._debounce_btn(btn_arr, self._last_btns if hasattr(self, '_last_btns') else [0]*11, BTN_RT):
            self._cycle_light()

        self._last_btns = btn_arr

        # 急停
        if self.e_stopped:
            move, yaw, up = 0.0, 0.0, 0.0

        is_dive_gear = (self.gear == GEAR_DIVE)

        # ═══════════════════════════════════════════════════
        # v5.0: 构建 Twist — 纯指令转发
        # ═══════════════════════════════════════════════════
        twist = Twist()
        twist.linear.x  = float(move)        # 前进/后退

        # 修正: 4档仅当悬停开启时才发送 dive_flag=1
        # 防止用户切换到4档但未按Y时, motor_controller 进入定深模式(target_depth=0)
        if is_dive_gear and self.depth_hold_on:
            twist.linear.y = DIVE_FLAG_VAL     # 定深标志
            twist.linear.z = float(self.target_depth)  # 目标深度(米)
        elif is_dive_gear:
            # 4档但未开启悬停: 当作手动模式, 允许手柄垂直控制
            twist.linear.y = 0.0
            twist.linear.z = float(up)       # 手动垂直推力
        else:
            twist.linear.y = 0.0             # 手动模式
            twist.linear.z = float(up)       # 手动垂直推力

        twist.angular.x = 1.0 if self.yaw_hold_on else 0.0  # 定航向标志
        twist.angular.y = float(self.yaw_hold_target) if self.yaw_hold_on else 0.0  # 目标航向(度)
        twist.angular.z = float(yaw)          # 手动偏航偏置
        self.cmd_pub.publish(twist)

        # ═══════════════════════════════════════════════════
        # CSV 记录
        # ═══════════════════════════════════════════════════
        if self.depth_hold_on and self.csv_file is not None:
            elapsed = now - self.log_start_time
            rpms = [self.last_motor_rpms.get(i, 0) for i in [0, 1, 2, 3, 5, 6, 7]]
            self._write_csv_row(
                elapsed, self.target_depth, self.current_depth,
                self.depth_pid_out, self.roll_pid_out, self.pitch_pid_out, self.yaw_pid_out,
                self.ins_roll, self.ins_pitch, self.ins_yaw,
                self.ins_ax, self.ins_ay, self.ins_az,
                self.ins_wx, self.ins_wy, self.ins_wz,
                self.ins_ve, self.ins_vn, self.ins_vd,
                rpms)

        # 定航向 CSV 记录
        if self.yaw_hold_on and self.yaw_csv_file is not None:
            elapsed = now - self.yaw_log_start_time
            rpms = [self.last_motor_rpms.get(i, 0) for i in [0, 1, 2, 3, 5, 6, 7]]
            # 计算角差 (归一化到 [-180, 180])
            raw_err = self.yaw_hold_target - self.ins_yaw
            if raw_err > 180.0:
                yaw_err = raw_err - 360.0
            elif raw_err < -180.0:
                yaw_err = raw_err + 360.0
            else:
                yaw_err = raw_err
            self._write_yaw_csv_row(
                elapsed, self.yaw_hold_target, self.ins_yaw, yaw_err,
                self.yaw_pid_out, self.roll_pid_out, self.pitch_pid_out, self.depth_pid_out,
                self.ins_roll, self.ins_pitch, self.ins_yaw,
                self.ins_ax, self.ins_ay, self.ins_az,
                self.ins_wx, self.ins_wy, self.ins_wz,
                self.ins_ve, self.ins_vn, self.ins_vd,
                rpms)

        # ═══════════════════════════════════════════════════
        # 显示
        # ═══════════════════════════════════════════════════
        btn_names = ['X', 'A', 'B', 'Y', 'LB', 'RB', 'LT', 'RT']
        active = [btn_names[i] for i in range(8) if btns.get(i, 0)]
        self._display_status(-move, yaw,   # v5.1: 取反使前进时条形向右填充
                             self.target_depth if self.depth_hold_on else 0.0,
                             active, is_dive_gear=is_dive_gear)

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    def _publish_state(self, event, msg):
        is_dg = (self.gear == GEAR_DIVE)
        data = json.dumps({
            'event': event,
            'msg': msg,
            'gear': self.gear + 1,
            'e_stopped': self.e_stopped,
            'depth_hold': self.depth_hold_on,
            'yaw_hold': self.yaw_hold_on,
            'light': self.light_state,
            'target_depth': round(self.target_depth, 2),
            'ts': time.time()
        }, ensure_ascii=False)
        self.state_pub.publish(String(data=data))

    def destroy_node(self):
        self._publish_zero()
        self.js.stop()
        if self._display_lines > 0:
            sys.stdout.write('\033[{}A\033[J'.format(self._display_lines))
            sys.stdout.flush()
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = JoyController()
    except RuntimeError as e:
        print('\n[ERROR] {}'.format(e))
        rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
