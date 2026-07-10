#!/usr/bin/env python3
"""
ROV 定深悬停测试脚本 v1.0
阶段1: target=0.5m, 30s
阶段2: target=0.8m, 30s
阶段3: 停止

用法 (RK3588):
  source /opt/ros/setup.bash
  export ROS_DOMAIN_ID=42
  python3 test_depth_hold.py
"""
import os
os.environ['ROS_DOMAIN_ID'] = '42'
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import json
import sys

class DepthHoldTester(Node):
    def __init__(self):
        super().__init__('depth_hold_tester')
        self.pub = self.create_publisher(Twist, '/rov/cmd_vel', 10)

        # 订阅 motor_state 获取实时数据
        self.depth = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw_target = 0.0
        self.yaw_pid = 0.0
        self.motors = {}
        self.state_sub = self.create_subscription(
            String, '/rov/motor_state', self.state_cb, 10)

        # 日志记录
        self.records = []
        self.t0 = time.time()

        self.get_logger().info('定深悬停测试启动!')
        self.get_logger().info('=' * 80)

    def state_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.depth = data.get('current_depth', 0.0)
            self.yaw = data.get('ins_yaw', 0.0)
            self.pitch = data.get('ins_pitch', 0.0)
            self.roll = data.get('ins_roll', 0.0)
            self.yaw_target = data.get('yaw_target', 0.0)
            self.yaw_pid = data.get('yaw_pid_out', 0.0)
            self.motors = data.get('motors', {})
        except:
            pass

    def send_cmd(self, dive_flag, target_depth):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = float(dive_flag)
        msg.linear.z = float(target_depth)
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def log_state(self, phase):
        elapsed = time.time() - self.t0
        yaw_err = self.yaw_target - self.yaw if self.yaw_target != 0 else 0.0
        m = self.motors
        rec = {
            't': round(elapsed, 1),
            'phase': phase,
            'depth': round(self.depth, 3),
            'yaw': round(self.yaw, 1),
            'pitch': round(self.pitch, 1),
            'roll': round(self.roll, 1),
            'yaw_target': round(self.yaw_target, 1),
            'yaw_err': round(yaw_err, 1),
            'yaw_pid': round(self.yaw_pid, 3),
            'motors': {k: v for k, v in m.items()},
        }
        self.records.append(rec)

        motor_str = ' '.join(f'ID{k}={v:+4d}' for k, v in sorted(m.items()))
        s = (f'[{elapsed:5.1f}s] {phase:6s} | '
             f'深={self.depth:.3f}m | '
             f'yaw={self.yaw:+.1f}°(目标{self.yaw_target:+.1f} 误{yaw_err:+.1f}°) | '
             f'pitch={self.pitch:+.1f}° roll={self.roll:+.1f}° | '
             f'yaw_pid={self.yaw_pid:+.3f} | '
             f'{motor_str}')
        self.get_logger().info(s)

    def run(self):
        t0 = time.time()

        # ── 预热: 发送10次确保 motor_controller 收到 ──
        self.get_logger().info('发送悬停指令 target=0.50m...')
        for _ in range(10):
            self.send_cmd(1.0, 0.5)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        # ── 阶段1: 0.50m 悬停 ──
        self.get_logger().info('=' * 80)
        self.get_logger().info('阶段1: 目标深度 0.50m (30秒)')
        self.get_logger().info('=' * 80)
        phase_start = time.time()
        last_log = 0
        last_cmd = 0
        while time.time() - phase_start < 30.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.time()
            # 10Hz 发布指令
            if now - last_cmd >= 0.1:
                self.send_cmd(1.0, 0.5)
                last_cmd = now
            if now - last_log >= 1.0:
                self.log_state('P1-0.5m')
                last_log = now
            time.sleep(0.03)

        # ── 阶段2: 0.80m 悬停 ──
        self.get_logger().info('=' * 80)
        self.get_logger().info('阶段2: 目标深度 0.80m (30秒)')
        self.get_logger().info('=' * 80)
        phase_start = time.time()
        last_log = 0
        last_cmd = 0
        while time.time() - phase_start < 30.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.time()
            if now - last_cmd >= 0.1:
                self.send_cmd(1.0, 0.8)
                last_cmd = now
            if now - last_log >= 1.0:
                self.log_state('P2-0.8m')
                last_log = now
            time.sleep(0.03)

        # ── 停止 ──
        self.get_logger().info('=' * 80)
        self.get_logger().info('测试完成, 停止电机')
        self.get_logger().info('=' * 80)
        for _ in range(10):
            self.send_cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        # ── 打印总结 ──
        self._print_summary()

    def _print_summary(self):
        if not self.records:
            self.get_logger().info('无记录')
            return

        self.get_logger().info('=' * 80)
        self.get_logger().info('                   测 试 总 结')
        self.get_logger().info('=' * 80)

        for phase_name, target in [('P1-0.5m', 0.5), ('P2-0.8m', 0.8)]:
            recs = [r for r in self.records if r['phase'] == phase_name]
            if not recs:
                continue

            depths = [r['depth'] for r in recs if r['depth'] > 0.01]
            yaws = [r['yaw'] for r in recs]
            yaw_errs = [r['yaw_err'] for r in recs if r['yaw_target'] != 0]
            pitches = [r['pitch'] for r in recs]
            rolls = [r['roll'] for r in recs]

            if depths:
                depth_avg = sum(depths) / len(depths)
                depth_err = depth_avg - target
                depth_min = min(depths)
                depth_max = max(depths)
                depth_std = (sum((d-depth_avg)**2 for d in depths)/len(depths))**0.5
            else:
                depth_avg = depth_err = depth_min = depth_max = depth_std = 0

            if yaw_errs:
                yaw_abs_max = max(abs(e) for e in yaw_errs)
                yaw_abs_avg = sum(abs(e) for e in yaw_errs) / len(yaw_errs)
            else:
                yaw_abs_max = yaw_abs_avg = 0

            pitch_abs_max = max(abs(p) for p in pitches) if pitches else 0
            roll_abs_max = max(abs(r) for r in rolls) if rolls else 0

            yaw_ok = '✓' if yaw_abs_max <= 5.0 else '✗'
            pitch_ok = '✓' if pitch_abs_max <= 5.0 else '✗'

            self.get_logger().info(
                f'  深度{target:.2f}m: 均值{depth_avg:.3f}m 误差{depth_err:+.3f}m '
                f'范围[{depth_min:.3f},{depth_max:.3f}]m std={depth_std:.3f}m')
            self.get_logger().info(
                f'  Yaw({yaw_ok}): 最大偏航={yaw_abs_max:.1f}° 平均={yaw_abs_avg:.1f}°')
            self.get_logger().info(
                f'  Pitch({pitch_ok}): 最大={pitch_abs_max:.1f}°')
            self.get_logger().info(
                f'  Roll: 最大={roll_abs_max:.1f}°')

        self.get_logger().info('=' * 80)

        # 保存到文件
        json_path = '/tmp/depth_hold_test_{}.json'.format(
            time.strftime('%Y%m%d_%H%M%S'))
        with open(json_path, 'w') as f:
            json.dump(self.records, f, indent=2)
        self.get_logger().info('详细记录保存至: {}'.format(json_path))


def main():
    rclpy.init()
    tester = DepthHoldTester()
    try:
        tester.run()
    except KeyboardInterrupt:
        tester.send_cmd(0.0, 0.0)
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
