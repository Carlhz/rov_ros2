#!/usr/bin/env python3
"""
ROV 自动定深测试 v1.3 — 无手柄依赖，可远程启停

v1.3 改动:
  - 配合 motor_controller v3.17 (尾部100%跟随PID), 降低默认 KP 3.0→2.0
v1.2 改动:
  - 修复 descend_slow 步进太快问题: 新增 --descend_interval 控制步进间隔(默认2s)
  - 防止 PID 瞬间全功率饱和导致 pitch 失控
v1.1 改动:
  - 新增 --no_yaw 禁用ID7侧推, 专注垂推+尾推定深
  - 优化默认参数配合 motor_controller v3.15 (低尾耦合+pitch修正)

用法:
  python3 auto_depth_test.py [--duration 60] [--target 0.5]
      [--kp 3.0] [--ki 0.15] [--i_max 0.30] [--dband 0.01] [--i_gate 0.05]
      [--kv 2.0] [--roll_kp 1.0] [--pitch_kp 1.5] [--yaw_kp 0.5]
      [--vert_base 1380] [--vert_half 200] [--descend_slow] [--no_yaw]
      [--descend_target 0.5] [--descend_step 0.05] [--descend_interval 2.0]

模式:
  默认: 直接在当前位置开启悬停, 运行 --duration 秒后停止
  --descend_slow: 缓慢下潜到 --descend_target (逐步减小目标深度, step=--descend_step)
  --descend_target: 缓慢下潜目标深度 (默认同 --target)

设计理念:
  - 完全独立, 不依赖物理手柄
  - PID 逻辑与 joy_controller v4.11 一致
  - 所有参数可通过 CLI 覆盖, 方便快速调参迭代
  - 自动 CSV 记录, 文件名含参数便于区分
"""

import os
os.environ['ROS_DOMAIN_ID'] = '42'

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String, Float32
import time
import sys
import csv
import json
import math
import argparse
from datetime import datetime

# ── 默认 PID 参数 (与 joy_controller v4.11 对齐) ───────
DEFAULT_KP         = 2.0   # v1.3: 配合 motor v3.17 尾部全耦合, 降低KP避免过冲
DEFAULT_KI         = 0.15
DEFAULT_KD         = 0.0
DEFAULT_I_MAX      = 0.30
DEFAULT_DBAND      = 0.01
DEFAULT_I_GATE     = 0.05
DEFAULT_I_DECAY    = 0.80
DEFAULT_KV         = 2.0

DEFAULT_ROLL_KP    = 1.0
DEFAULT_ROLL_KI    = 0.05
DEFAULT_ROLL_I_MAX = 0.05
DEFAULT_ROLL_DBAND = 0.3

DEFAULT_PITCH_KP    = 1.5    # v1.1: 增强pitch修正响应
DEFAULT_PITCH_KI    = 0.05
DEFAULT_PITCH_I_MAX = 0.05
DEFAULT_PITCH_DBAND = 0.3

DEFAULT_YAW_KP    = 0.5
DEFAULT_YAW_KI    = 0.03
DEFAULT_YAW_I_MAX = 0.10
DEFAULT_YAW_DBAND = 1.0

DEFAULT_VE2ROLL_K  = 3.0
DEFAULT_VN2PITCH_K = 3.0
VEL_ATT_MAX        = 5.0

DEFAULT_VERT_BASE  = 1380   # v1.1: 配合 motor_controller v3.15
DEFAULT_VERT_HALF  = 200
DEFAULT_TAIL_RPM   = 1170
DIVE_FLAG          = 0.65  # 定深档标志 (DIVE_TAIL_RPM/MAX_RPM)

MAX_RPM   = 1800
MIN_RPM   = 1100

DEPTH_TIMEOUT    = 3.0
VEL_TIMEOUT      = 2.0
ATT_TIMEOUT      = 2.0
DEPTH_SMOOTH     = 0.5

LOG_DIR = os.path.expanduser('~/rov_ros2_ws/logs')


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _angle_diff(target, current):
    d = target - current
    while d > 180: d -= 360
    while d < -180: d += 360
    return d


def pid_step(err, dt, err_i, kp, ki, kd, i_max, deadband, i_gate, i_decay,
             err_last=0.0, output_saturated=False):
    p_error = 0.0 if abs(err) < deadband else err
    p = kp * p_error
    new_err_i = err_i
    if dt > 0 and dt < 2.0:
        if abs(err) > i_gate:
            pass
        elif (err * err_i) < -0.01:
            new_err_i = 0.0
        elif abs(err) < deadband:
            new_err_i *= i_decay
        elif not output_saturated or (err * err_i) < 0:
            new_err_i = _clamp(err_i + ki * err * dt, -i_max, i_max)
    d = 0.0
    if kd > 0 and dt > 0.001:
        d = kd * (err - err_last) / dt
    out = _clamp(p + new_err_i + d, -1.0, 1.0)
    return out, new_err_i, err


class AutoDepthTest(Node):
    def __init__(self, args):
        super().__init__('auto_depth_test')

        # ── 保存配置 ──
        self.args = args
        self.duration = args.duration
        self.target_depth = args.target
        self.descend_slow = args.descend_slow
        self.descend_target = args.descend_target if args.descend_target else args.target
        self.descend_step = args.descend_step
        self.descend_interval = args.descend_interval
        self._last_step_time = 0.0  # 上次步进时间

        # ── PID 参数 ──
        self.depth_kp     = args.kp
        self.depth_ki     = args.ki
        self.depth_kd     = args.kd
        self.depth_i_max  = args.i_max
        self.depth_dband  = args.dband
        self.depth_i_gate = args.i_gate
        self.depth_i_decay = DEFAULT_I_DECAY
        self.ins_kv       = args.kv

        self.roll_kp     = args.roll_kp
        self.roll_ki     = args.roll_ki
        self.roll_i_max  = args.roll_i_max
        self.roll_dband  = DEFAULT_ROLL_DBAND
        self.roll_i_gate = 2.0
        self.roll_i_decay = 0.80

        self.pitch_kp     = args.pitch_kp
        self.pitch_ki     = args.pitch_ki
        self.pitch_i_max  = args.pitch_i_max
        self.pitch_dband  = DEFAULT_PITCH_DBAND
        self.pitch_i_gate = 2.0
        self.pitch_i_decay = 0.80

        self.yaw_kp     = args.yaw_kp
        self.yaw_ki     = args.yaw_ki
        self.yaw_i_max  = args.yaw_i_max
        self.yaw_dband  = DEFAULT_YAW_DBAND
        self.yaw_i_gate = 5.0
        self.yaw_i_decay = 0.85

        self.ve2roll_k   = DEFAULT_VE2ROLL_K
        self.vn2pitch_k  = DEFAULT_VN2PITCH_K

        self.vert_base   = args.vert_base
        self.vert_half   = args.vert_half
        self.no_yaw      = args.no_yaw

        # ── 传感器状态 ──
        self.current_depth   = 0.0
        self.filtered_depth  = 0.0
        self.depth_valid     = False
        self.last_depth_time = 0.0

        self.ins_vd = 0.0; self.ins_ve = 0.0; self.ins_vn = 0.0
        self.ins_vel_valid = False
        self.last_vel_time = 0.0

        self.ins_roll = 0.0; self.ins_pitch = 0.0; self.ins_yaw = 0.0
        self.ins_att_valid = False
        self.last_att_time = 0.0

        # ── PID 状态 ──
        self.depth_err_i = 0.0; self.depth_err_last = 0.0
        self.depth_pid_out = 0.0

        self.roll_err_i = 0.0; self.roll_err_last = 0.0
        self.roll_pid_out = 0.0

        self.pitch_err_i = 0.0; self.pitch_err_last = 0.0
        self.pitch_pid_out = 0.0

        self.yaw_err_i = 0.0; self.yaw_err_last = 0.0
        self.yaw_pid_out = 0.0

        self.yaw_target = 0.0

        # ── 控制状态 ──
        self.hold_active   = False
        self.start_time    = 0.0
        self.last_pid_time = 0.0
        self.phase         = 'waiting'  # waiting/descending/holding/done
        self.init_depth    = 0.0
        self.depth_samples = []  # 用于计算初始深度

        # ── 状态保活 ──
        self.last_published_rpms = {0: 0, 1: 0, 2: 0, 3: 0, 5: 0, 6: 0, 7: 0}

        # ── 话题 ──
        self.cmd_pub  = self.create_publisher(Twist, '/rov/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/rov/joy_state', 10)

        self.depth_sub = self.create_subscription(Float32, '/rov/depth', self._depth_cb, 10)
        self.vel_sub   = self.create_subscription(Vector3, '/ins/velocity', self._vel_cb, 10)
        self.att_sub   = self.create_subscription(Vector3, '/ins/attitude', self._att_cb, 10)
        self.motor_sub = self.create_subscription(String, '/rov/motor_state', self._motor_cb, 10)

        # ── CSV ──
        os.makedirs(LOG_DIR, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        tag = 'descend' if self.descend_slow else 'hold'
        self.csv_path = os.path.join(LOG_DIR,
            'auto_test_{}_kp{:.0f}_ki{:.2f}_{}.csv'.format(
                tag, self.depth_kp * 10, self.depth_ki * 100, ts))
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'elapsed', 'phase', 'target', 'actual', 'est',
            'depth_pid', 'roll_pid', 'pitch_pid', 'yaw_pid',
            'depth_err_i', 'ins_roll', 'ins_pitch', 'ins_yaw',
            've', 'vn', 'vd', 'err_mm',
            'id0', 'id1', 'id2', 'id3', 'id5', 'id6', 'id7'
        ])

        # ── 定时器: 20Hz PID ──
        self.create_timer(0.05, self.pid_timer)

        # ── 启动信息 ──
        self.get_logger().info('='*60)
        self.get_logger().info('  ROV 自动定深测试 v1.0')
        self.get_logger().info('  时长: {}s  目标: {:.2f}m'.format(self.duration, self.target_depth))
        if self.descend_slow:
            self.get_logger().info('  模式: 缓慢下潜 (step={:.2f}m/{}s) → {:.2f}m'.format(
                self.descend_step, self.descend_interval, self.descend_target))
        self.get_logger().info('  Kp={:.1f}  Ki={:.2f}  I_MAX={:.2f}  DBAND={:.3f}'.format(
            self.depth_kp, self.depth_ki, self.depth_i_max, self.depth_dband))
        self.get_logger().info('  Kv={:.1f}  VERT: {}±{}rpm  NO_YAW:{}'.format(
            self.ins_kv, self.vert_base, self.vert_half, self.no_yaw))
        self.get_logger().info('  CSV: {}'.format(self.csv_path))
        self.get_logger().info('='*60)
        print('\n等待传感器数据...')

    # ── 订阅回调 ──
    def _depth_cb(self, msg):
        raw = float(msg.data)
        if self.depth_valid:
            self.filtered_depth = DEPTH_SMOOTH * raw + (1 - DEPTH_SMOOTH) * self.filtered_depth
        else:
            self.filtered_depth = raw
        self.current_depth = self.filtered_depth
        self.depth_valid = True
        self.last_depth_time = time.time()

    def _vel_cb(self, msg):
        self.ins_ve = float(msg.x)
        self.ins_vn = float(msg.y)
        self.ins_vd = float(msg.z)
        self.ins_vel_valid = True
        self.last_vel_time = time.time()

    def _att_cb(self, msg):
        self.ins_pitch = float(msg.x)
        self.ins_roll  = float(msg.y)
        self.ins_yaw   = float(msg.z)
        self.ins_att_valid = True
        self.last_att_time = time.time()

    def _motor_cb(self, msg):
        try:
            s = json.loads(msg.data)
            if 'motors' in s:
                m = s['motors']
                for k in [0, 1, 2, 3, 5, 6, 7]:
                    self.last_published_rpms[k] = m.get(str(k), m.get(k, 0))
        except Exception:
            pass

    # ── 预估深度 ──
    def _est_depth(self):
        if not self.depth_valid:
            return 0.0
        dt = time.time() - self.last_depth_time
        if dt < 0.05:
            return self.current_depth
        if self.ins_vel_valid and (time.time() - self.last_vel_time) < VEL_TIMEOUT:
            est = self.current_depth + self.ins_vd * dt
            return _clamp(est, self.current_depth - 0.3, self.current_depth + 0.3)
        return self.current_depth

    # ── 20Hz PID 定时器 ──
    def pid_timer(self):
        now = time.time()
        dt = 0.05

        # ── 阶段管理 ──
        if self.phase == 'waiting':
            # 等待传感器数据就绪
            if not self.depth_valid:
                return
            # 采集初始深度
            self.depth_samples.append(self.current_depth)
            elapsed_wait = now - self.last_pid_time if self.last_pid_time > 0 else 0
            if len(self.depth_samples) >= 20 or elapsed_wait > 2.0:
                self.init_depth = (sum(self.depth_samples) / len(self.depth_samples)
                                   if self.depth_samples else self.current_depth)
                if self.descend_slow:
                    self.target_depth = self.init_depth  # 缓慢下潜: 从当前位置开始
                    self.phase = 'descending'
                    self._last_step_time = now - self.descend_interval  # 首次立即步进
                    self.get_logger().info('开始缓慢下潜: {:.3f}m → {:.3f}m (间隔{}s)'.format(
                        self.init_depth, self.descend_target, self.descend_interval))
                else:
                    self.target_depth = self.init_depth  # 就在当前位置悬停
                    self.phase = 'holding'
                    self.get_logger().info('开始悬停测试: 目标={:.3f}m'.format(self.target_depth))
                self.hold_active = True
                self.start_time = now
                self.last_pid_time = now
                self.last_depth_time = now  # 初始化预估基准
                # 捕获航向
                if self.ins_att_valid:
                    self.yaw_target = self.ins_yaw
                # 重置 PID
                self.depth_err_i = 0.0; self.roll_err_i = 0.0
                self.pitch_err_i = 0.0; self.yaw_err_i = 0.0
            return

        if self.phase == 'descending':
            # v1.2: 真正缓慢下潜 — 每隔 descend_interval 秒步进一步
            elapsed = now - self.start_time
            time_since_step = now - self._last_step_time
            if (time_since_step >= self.descend_interval and
                    self.target_depth < self.descend_target + 0.01):
                new_target = min(self.target_depth + self.descend_step, self.descend_target)
                if new_target != self.target_depth:
                    self.target_depth = new_target
                    self._last_step_time = now
                    # 重置积分避免累积
                    self.depth_err_i *= 0.5
                    self.get_logger().info('  ↘ 目标深度: {:.2f}m  (已用时 {:.0f}s, PID={:.2f})'.format(
                        self.target_depth, elapsed, self.depth_pid_out))
                if self.target_depth >= self.descend_target - 0.001:
                    self.phase = 'holding'
                    self.get_logger().info('到达目标, 进入悬停阶段 (PID={:.2f})'.format(self.depth_pid_out))
            elif elapsed > self.duration * 1.5:
                self.phase = 'done'

        if self.phase == 'holding':
            elapsed = now - self.start_time
            if elapsed >= self.duration:
                self.phase = 'done'
                self.get_logger().info('测试时间到, 停止')

        if self.phase == 'done':
            self.hold_active = False
            self._send_stop()
            self._finish()
            return

        if not self.hold_active:
            return
        if self.phase != 'holding' and self.phase != 'descending':
            return

        # ═══════════════════════════════════════════════
        # PID 计算 (与 joy_controller v4.11 一致)
        # ═══════════════════════════════════════════════

        # ── 速度→姿态 级联 ──
        self._vel_valid = (self.ins_vel_valid and
                     (now - self.last_vel_time) < VEL_TIMEOUT)
        vel_roll_bias  = 0.0
        vel_pitch_bias = 0.0
        if self._vel_valid:
            vel_roll_bias  = _clamp(-self.ve2roll_k * self.ins_ve, -VEL_ATT_MAX, VEL_ATT_MAX)
            vel_pitch_bias = _clamp(-self.vn2pitch_k * self.ins_vn, -VEL_ATT_MAX, VEL_ATT_MAX)

        att_valid = (self.ins_att_valid and
                     (now - self.last_att_time) < ATT_TIMEOUT)
        self._att_valid = att_valid

        # ── Roll PID ──
        if self._att_valid:
            roll_target = vel_roll_bias  # 基础目标=0, 速度偏置
            roll_err = roll_target - self.ins_roll
            self.roll_pid_out, self.roll_err_i, self.roll_err_last = pid_step(
                roll_err, dt, self.roll_err_i,
                self.roll_kp, self.roll_ki, 0.0, self.roll_i_max,
                self.roll_dband, self.roll_i_gate, self.roll_i_decay, self.roll_err_last)
        else:
            self.roll_pid_out = 0.0; self.roll_err_i = 0.0

        # ── Pitch PID ──
        if self._att_valid:
            pitch_target = vel_pitch_bias
            pitch_err = pitch_target - self.ins_pitch
            self.pitch_pid_out, self.pitch_err_i, self.pitch_err_last = pid_step(
                pitch_err, dt, self.pitch_err_i,
                self.pitch_kp, self.pitch_ki, 0.0, self.pitch_i_max,
                self.pitch_dband, self.pitch_i_gate, self.pitch_i_decay, self.pitch_err_last)
        else:
            self.pitch_pid_out = 0.0; self.pitch_err_i = 0.0

        # ── Yaw PID ──
        if self._att_valid and not self.no_yaw:
            yaw_err = _angle_diff(self.yaw_target, self.ins_yaw)
            self.yaw_pid_out, self.yaw_err_i, self.yaw_err_last = pid_step(
                yaw_err, dt, self.yaw_err_i,
                self.yaw_kp, self.yaw_ki, 0.0, self.yaw_i_max,
                self.yaw_dband, self.yaw_i_gate, self.yaw_i_decay, self.yaw_err_last)
        else:
            self.yaw_pid_out = 0.0; self.yaw_err_i = 0.0

        # ── Depth PID ──
        est_depth = self._est_depth()
        raw_error = self.target_depth - est_depth

        p_error = 0.0 if abs(raw_error) < self.depth_dband else raw_error
        p = self.depth_kp * p_error

        # 积分: 与 joy_controller v4.11 一致
        if dt > 0 and dt < 2.0:
            if abs(raw_error) > self.depth_i_gate:
                pass  # 远端冻结I
            elif (raw_error * self.depth_err_i) < -0.01:
                self.depth_err_i = 0.0
            elif abs(raw_error) < self.depth_dband:
                self.depth_err_i *= self.depth_i_decay
            elif abs(self.depth_pid_out) < 0.95 or (raw_error * self.depth_err_i) < 0:
                self.depth_err_i = _clamp(
                    self.depth_err_i + self.depth_ki * raw_error * dt,
                    -self.depth_i_max, self.depth_i_max)

        vel_term = 0.0
        if self._vel_valid:
            vel_term = -self.ins_kv * self.ins_vd

        self.depth_pid_out = _clamp(p + self.depth_err_i + vel_term, -1.0, 1.0)
        self.depth_err_last = raw_error
        self.last_pid_time = now

        # ── 发送命令 ──
        self._publish_cmd()

    def _publish_cmd(self):
        twist = Twist()
        twist.linear.x  = 0.0  # 悬停时不前进
        twist.linear.y  = float(DIVE_FLAG)
        twist.linear.z  = float(self.depth_pid_out)
        twist.angular.x = float(self.roll_pid_out)
        twist.angular.y = float(self.pitch_pid_out)
        twist.angular.z = float(self.yaw_pid_out)
        self.cmd_pub.publish(twist)

        # ── CSV 记录 ──
        now = time.time()
        elapsed = now - self.start_time
        err_mm = (self.target_depth - self.current_depth) * 1000
        rpms = [self.last_published_rpms.get(i, 0) for i in [0, 1, 2, 3, 5, 6, 7]]
        self.csv_writer.writerow([
            '{:.3f}'.format(elapsed),
            self.phase,
            '{:.3f}'.format(self.target_depth),
            '{:.3f}'.format(self.current_depth),
            '{:.3f}'.format(self._est_depth()),
            '{:.4f}'.format(self.depth_pid_out),
            '{:.4f}'.format(self.roll_pid_out),
            '{:.4f}'.format(self.pitch_pid_out),
            '{:.4f}'.format(self.yaw_pid_out),
            '{:.4f}'.format(self.depth_err_i),
            '{:.2f}'.format(self.ins_roll),
            '{:.2f}'.format(self.ins_pitch),
            '{:.2f}'.format(self.ins_yaw),
            '{:.3f}'.format(self.ins_ve),
            '{:.3f}'.format(self.ins_vn),
            '{:.3f}'.format(self.ins_vd),
            '{:.1f}'.format(err_mm),
        ] + [str(r) for r in rpms])

        # ── 控制台输出 ──
        elapsed_since_start = now - self.start_time
        if self.phase == 'descending':
            phase_info = '↘ DROP 目标:{:.2f}'.format(self.target_depth)
        else:
            phase_info = '◎ HOLD'
        err_str = '{:+.3f}'.format(self.target_depth - self.current_depth)
        pid_str = '{:+.2f}'.format(self.depth_pid_out)
        i_str   = '{:+.3f}'.format(self.depth_err_i)
        vd_str  = '{:+.2f}'.format(self.ins_vd) if self._vel_valid else '--'
        r_str   = '{:.1f}'.format(self.ins_roll) if self._att_valid else '--'
        p_str   = '{:.1f}'.format(self.ins_pitch) if self._att_valid else '--'
        print('\r[{}s] {} | 深度:{:.3f} 目标:{:.3f} err:{} | PID:{} I:{} | vd:{} roll:{} pitch:{}'.format(
            int(elapsed_since_start), phase_info,
            self.current_depth, self.target_depth, err_str,
            pid_str, i_str, vd_str, r_str, p_str),
            end='', flush=True)

    def _send_stop(self):
        self.cmd_pub.publish(Twist())
        time.sleep(0.1)
        self.cmd_pub.publish(Twist())

    def _finish(self):
        print()  # newline
        self.csv_file.close()

        # ── 分析结果 ──
        self._analyze()

        self.get_logger().info('CSV: {}'.format(self.csv_path))
        self.get_logger().info('测试完成, 3s后退出...')
        time.sleep(0.5)

        # 发送停止命令确保安全
        for _ in range(5):
            self._send_stop()
            time.sleep(0.1)

        # 创建完成标记文件
        result_file = LOG_DIR + '/last_test_result.txt'
        with open(result_file, 'w') as f:
            f.write('csv={}\n'.format(self.csv_path))
            f.write('time={}\n'.format(datetime.now().isoformat()))

        # 安排退出
        self.create_timer(1.0, self._shutdown)

    def _shutdown(self):
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    def _analyze(self):
        """分析 CSV 数据并输出统计"""
        if not os.path.exists(self.csv_path):
            return
        errors_mm = []
        with open(self.csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row.get('phase') == 'holding':
                    try:
                        errors_mm.append(float(row.get('err_mm', 0)))
                    except Exception:
                        pass

        if not errors_mm:
            self.get_logger().warn('无悬停阶段数据可分析')
            return

        n = len(errors_mm)
        rms = math.sqrt(sum(e*e for e in errors_mm) / n)
        max_e = max(abs(e) for e in errors_mm)
        mean_e = sum(errors_mm) / n
        std_e = math.sqrt(sum((e - mean_e)**2 for e in errors_mm) / n)
        in_band = sum(1 for e in errors_mm if abs(e) <= 10) / n * 100  # ±10mm

        self.get_logger().info('='*60)
        self.get_logger().info('  测试结果分析 (悬停阶段 {} 个采样点)'.format(n))
        self.get_logger().info('  RMS:  {:.1f}mm  ({:.3f}m)'.format(rms, rms/1000))
        self.get_logger().info('  STD:  {:.1f}mm  ({:.3f}m)'.format(std_e, std_e/1000))
        self.get_logger().info('  MAX:  {:.1f}mm  ({:.3f}m)'.format(max_e, max_e/1000))
        self.get_logger().info('  MEAN: {:.1f}mm  ({:.3f}m)'.format(mean_e, mean_e/1000))
        self.get_logger().info('  ±10mm内: {:.1f}%'.format(in_band))
        self.get_logger().info('  PASS: {} (RMS={:.1f}mm)'.format(
            '✅' if rms <= 10 else '❌ 需继续调参', rms))
        self.get_logger().info('='*60)

        # 打印一行可直接复制的调参命令
        if rms > 10:
            # 分析误差特征, 给出建议
            if abs(mean_e) > 10:
                # 稳态偏置大 → 增大 KI
                new_ki = min(self.depth_ki * 1.5, 0.50)
                self.get_logger().info('  建议: 稳态偏置大, 增大KI')
                self.get_logger().info('  重测: --ki {:.3f} --i_max {:.3f}'.format(new_ki, new_ki*2))
            elif std_e > rms * 0.7:
                # 振荡大 → 减小 KP, 增大死区
                new_kp = max(self.depth_kp * 0.7, 0.5)
                self.get_logger().info('  建议: 振荡较大, 减小KP')
                self.get_logger().info('  重测: --kp {:.1f}'.format(new_kp))
            else:
                # 一般情况 → 微调 KP + KI
                new_kp = self.depth_kp * 0.8
                new_ki = self.depth_ki * 1.2
                self.get_logger().info('  建议: 微调 KP↓ KI↑')
                self.get_logger().info('  重测: --kp {:.1f} --ki {:.3f}'.format(new_kp, new_ki))


def main():
    parser = argparse.ArgumentParser(description='ROV 自动定深测试')
    parser.add_argument('--duration', type=float, default=60,
                        help='测试时长 (秒), 默认60')
    parser.add_argument('--target', type=float, default=0.5,
                        help='悬停目标深度 (米), 默认0.5')
    # PID 参数
    parser.add_argument('--kp', type=float, default=DEFAULT_KP)
    parser.add_argument('--ki', type=float, default=DEFAULT_KI)
    parser.add_argument('--kd', type=float, default=DEFAULT_KD)
    parser.add_argument('--i_max', type=float, default=DEFAULT_I_MAX)
    parser.add_argument('--dband', type=float, default=DEFAULT_DBAND)
    parser.add_argument('--i_gate', type=float, default=DEFAULT_I_GATE)
    parser.add_argument('--kv', type=float, default=DEFAULT_KV)
    # 姿态 PID
    parser.add_argument('--roll_kp', type=float, default=DEFAULT_ROLL_KP)
    parser.add_argument('--roll_ki', type=float, default=DEFAULT_ROLL_KI)
    parser.add_argument('--roll_i_max', type=float, default=DEFAULT_ROLL_I_MAX)
    parser.add_argument('--pitch_kp', type=float, default=DEFAULT_PITCH_KP)
    parser.add_argument('--pitch_ki', type=float, default=DEFAULT_PITCH_KI)
    parser.add_argument('--pitch_i_max', type=float, default=DEFAULT_PITCH_I_MAX)
    parser.add_argument('--yaw_kp', type=float, default=DEFAULT_YAW_KP)
    parser.add_argument('--yaw_ki', type=float, default=DEFAULT_YAW_KI)
    parser.add_argument('--yaw_i_max', type=float, default=DEFAULT_YAW_I_MAX)
    # 垂直范围
    parser.add_argument('--vert_base', type=int, default=DEFAULT_VERT_BASE)
    parser.add_argument('--vert_half', type=int, default=DEFAULT_VERT_HALF)
    # 缓慢下潜
    parser.add_argument('--descend_slow', action='store_true',
                        help='缓慢下潜模式')
    parser.add_argument('--descend_target', type=float, default=None,
                        help='缓慢下潜目标深度')
    parser.add_argument('--descend_step', type=float, default=0.05,
                        help='缓慢下潜步长 (米), 默认0.05')
    parser.add_argument('--descend_interval', type=float, default=2.0,
                        help='缓慢下潜步进间隔 (秒), 默认2.0')
    parser.add_argument('--no_yaw', action='store_true',
                        help='禁用ID7侧推, 仅垂推+尾推定深')

    args = parser.parse_args()

    rclpy.init()
    node = AutoDepthTest(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n中断')
        node._send_stop()
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
