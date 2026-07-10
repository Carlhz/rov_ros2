#!/usr/bin/env python3
"""
自动化定深测试 v1.0 — 多轮定深测试 + 数据采集

用法 (RK3588):
  source /opt/ros/setup.bash
  export ROS_DOMAIN_ID=42
  python3 /opt/ros/rov_ros2_ws/auto_depth_test.py

功能:
  - 依次测试 0.4 / 0.5 / 0.6 / 0.7 / 0.8 米深度
  - 每轮先等 8 秒让 ROV 稳定, 再记录 60 秒数据
  - 以 10Hz 持续发布 cmd_vel (防止 motor_controller 5s 超时)
  - 订阅 /rov/motor_state (2Hz), 记录完整状态到 CSV
"""

import os
import sys
import time
import json
import csv
import signal
import math
from datetime import datetime

os.environ['ROS_DOMAIN_ID'] = '42'

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ── 测试参数 ──
TARGETS        = [0.4, 0.5, 0.6, 0.7, 0.8]   # 目标深度 (米)
RECORD_SECONDS = 60      # 每轮记录时长
SETTLE_SECONDS = 8       # 切换深度后稳定等待时间 (等PID收敛)
CMD_HZ         = 10      # cmd_vel 发布频率 (防超时)

# ── 列定义 ──
CSV_HEADER = [
    'round', 'target_depth', 'elapsed_sec',
    'current_depth', 'depth_error', 'depth_pid_out', 'depth_err_i',
    'pitch_deg', 'roll_deg', 'yaw_deg',
    'id0', 'id1', 'id2', 'id3', 'id5', 'id6', 'id7',
    'fz_vert_avg', 'fz_tail_avg'
]


class AutoDepthTest(Node):
    def __init__(self):
        super().__init__('auto_depth_test')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_sub = self.create_subscription(
            String, '/rov/motor_state', self._state_cb, 10)

        # ── 状态机 ──
        self._round_idx        = -1                    # 当前轮次索引
        self._phase            = 'idle'                # idle | settle | record | done
        self._phase_start_ts   = 0.0
        self._round_record_cnt = 0
        self._running          = True
        self._final_csv        = None

        # ── CSV 输出 ──
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = '/tmp/auto_depth_test_{}.csv'.format(ts)
        self._csv_fh = open(self._csv_path, 'w', newline='')
        self._csv_w = csv.writer(self._csv_fh)
        self._csv_w.writerow(CSV_HEADER)

        # ── 定时器 ──
        self.create_timer(1.0 / CMD_HZ, self._publish_cmd)
        self.create_timer(1.0, self._status_tick)

        self.get_logger().info('=' * 55)
        self.get_logger().info('  自动化定深测试 v1.0')
        self.get_logger().info('  目标: {}'.format(TARGETS))
        self.get_logger().info('  每轮: {}s 稳定 + {}s 记录'.format(SETTLE_SECONDS, RECORD_SECONDS))
        self.get_logger().info('  CSV:  {}'.format(self._csv_path))
        self.get_logger().info('  3 秒后开始第 1 轮...')
        self.get_logger().info('=' * 55)

        # 延迟启动第一轮
        self.create_timer(3.0, self._start_first_round)

    # ═══════════════════════════════════════
    # 状态机
    # ═══════════════════════════════════════

    def _start_first_round(self):
        self._next_round()

    def _next_round(self):
        self._flush_csv()
        self._round_idx += 1
        if self._round_idx >= len(TARGETS):
            self._phase = 'done'
            self._running = False
            self.get_logger().info('')
            self.get_logger().info('=' * 55)
            self.get_logger().info('  全部 {} 轮测试完成!'.format(len(TARGETS)))
            self.get_logger().info('  数据文件: {}'.format(self._csv_path))
            self.get_logger().info('=' * 55)
            return

        self._phase            = 'settle'
        self._phase_start_ts   = time.time()
        self._round_record_cnt = 0
        self.get_logger().info('')
        self.get_logger().info('┌─ 第 {}/{} 轮 ─ 目标深度: {:.1f}m ────────'.format(
            self._round_idx + 1, len(TARGETS), TARGETS[self._round_idx]))
        self.get_logger().info('│  稳定等待 {}s ...'.format(SETTLE_SECONDS))

    def _status_tick(self):
        """ 每秒状态更新 """
        if self._phase == 'settle':
            elapsed = time.time() - self._phase_start_ts
            remain  = max(0, SETTLE_SECONDS - elapsed)
            self.get_logger().info('│  等待中... 剩余 {:.0f}s'.format(remain))
        elif self._phase == 'record':
            elapsed = time.time() - self._phase_start_ts
            remain  = max(0, RECORD_SECONDS - elapsed)
            if self._round_record_cnt % 4 == 0:  # 每2秒
                self.get_logger().info(
                    '│  记录中... 已记录 {} 条, 剩余 {:.0f}s'.format(
                        self._round_record_cnt, remain))

    def _state_cb(self, msg: String):
        if not self._running:
            return
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        # 在 settle 阶段: 检查是否到时候开始记录
        if self._phase == 'settle':
            elapsed = time.time() - self._phase_start_ts
            if elapsed >= SETTLE_SECONDS:
                self._phase = 'settle→record'
                self._phase_start_ts = time.time()
                self._round_record_cnt = 0
                self.get_logger().info('│  稳定完成, 开始记录 60s 数据...')
            return

        if self._phase not in ('settle→record', 'record'):
            return
        # settle→record 只持续一次回调, 用于重置计时器
        if self._phase == 'settle→record':
            self._phase = 'record'
            self._phase_start_ts = time.time()
            self._round_record_cnt = 0

        # 记录阶段
        elapsed = time.time() - self._phase_start_ts
        if elapsed > RECORD_SECONDS:
            self.get_logger().info(
                '│  第 {} 轮完成 (记录 {} 条)'.format(
                    self._round_idx + 1, self._round_record_cnt))
            self._next_round()
            return

        # ── 写 CSV ──
        motors = data.get('motors', [0] * 8)
        n_motors = len(motors)

        row = [
            self._round_idx + 1,
            round(data.get('target_depth', 0), 2),
            round(elapsed, 1),
            round(data.get('current_depth', 0), 3),
            round(data.get('target_depth', 0) - data.get('current_depth', 0), 4),
            round(data.get('depth_pid_out', 0), 5),
            round(data.get('depth_err_i', 0), 5),
            round(data.get('ins_pitch', 0), 1),
            round(data.get('ins_roll', 0), 1),
            round(data.get('ins_yaw', 0), 1),
            motors[0] if n_motors > 0 else 0,
            motors[1] if n_motors > 1 else 0,
            motors[2] if n_motors > 2 else 0,
            motors[3] if n_motors > 3 else 0,
            motors[5] if n_motors > 5 else 0,
            motors[6] if n_motors > 6 else 0,
            motors[7] if n_motors > 7 else 0,
            round((motors[5] + motors[6]) / 2.0, 1) if n_motors > 6 else 0,
            round(sum(motors[i] for i in [0,1,2,3] if i < n_motors) / max(1, sum(1 for i in [0,1,2,3] if i < n_motors)), 1),
        ]
        self._csv_w.writerow(row)
        self._round_record_cnt += 1

    # ═══════════════════════════════════════
    # cmd_vel 发布 (10Hz, 防止超时)
    # ═══════════════════════════════════════

    def _publish_cmd(self):
        if self._phase == 'done':
            t = Twist()
            t.linear.x = t.linear.y = t.linear.z = 0.0
            t.angular.x = t.angular.y = t.angular.z = 0.0
            self.cmd_pub.publish(t)
            return

        if self._round_idx < len(TARGETS):
            t = Twist()
            t.linear.y  = 1.0       # dive_flag = 1 (定深模式)
            t.linear.z  = float(TARGETS[self._round_idx])
            t.linear.x  = 0.0       # 不前进后退
            t.angular.x = 0.0       # 不定航向
            t.angular.y = 0.0
            t.angular.z = 0.0       # 不手动转向
            self.cmd_pub.publish(t)

    def _flush_csv(self):
        if self._csv_fh:
            self._csv_fh.flush()

    def shutdown(self):
        self._running = False
        # 发送停止命令
        t = Twist()
        self.cmd_pub.publish(t)
        if self._csv_fh:
            self._csv_fh.close()
        self.get_logger().info('已关闭, 数据: {}'.format(self._csv_path))


def main():
    rclpy.init()
    node = AutoDepthTest()

    # 注册信号处理
    def _sig(sig, frame):
        node.shutdown()
        rclpy.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)

    try:
        while rclpy.ok() and node._running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
