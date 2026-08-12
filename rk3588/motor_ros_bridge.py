#!/usr/bin/env python3
"""
motor_ros_bridge.py — ROS2 ↔ C motor_controller 共享内存桥 (v9.0)

架构 (双进程, 与 motor_controller.c 配套):
  [C motor_controller]  CAN + B+伪逆 + PID + 档位增益 + 安全
  [本桥 Python]         ROS2 订阅 → mmap 共享内存; 读 mmap → 发布状态

职责:
  1. 订阅 /rov/cmd_vel (Twist)     → 处理定航向捕获/模式切换 → 写 mmap INPUT
  2. 订阅 /rov/joy_state (String)  → 解析 gear → 写 mmap INPUT
  3. 订阅 /ins/attitude|acceleration|angular_rate|velocity, /rov/depth → 写 mmap INPUT
  4. 10Hz 读 mmap OUTPUT           → 发布 /rov/motor_state (String JSON)

共享内存布局 (与 motor_controller.c 完全一致, 全 double, 4096B):
  [  0..215]  INPUT  (桥→C): 27 doubles
  [216..351]  OUTPUT (C→桥): 17 doubles

编译/运行:
  RK3588:  /opt/ros/rov_ros2_ws/motor_controller  (C 二进制, 由 start_all.sh 启动)
  桥:      python3 motor_ros_bridge.py

环境: ROS_DOMAIN_ID=42
"""

import os
import sys
import json
import time
import struct
import mmap
import signal

os.environ.setdefault('ROS_DOMAIN_ID', '42')

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, String


# ── 共享内存布局 (必须与 motor_controller.c 一致) ──────────────────────
SHM_PATH = '/dev/shm/rov_motor_shm'
SHM_SIZE = 4096

# INPUT offsets (double, 8B each)
IN_MOVE             = 0 * 8
IN_UP               = 1 * 8
IN_YAW              = 2 * 8
IN_DIVE_FLAG        = 3 * 8
IN_TARGET_DEPTH     = 4 * 8
IN_YAW_HOLD_ACTIVE  = 5 * 8
IN_YAW_HOLD_TARGET  = 6 * 8
IN_LAST_CMD_TIME    = 7 * 8
IN_E_STOPPED        = 8 * 8
IN_GEAR             = 9 * 8
IN_INS_YAW          = 10 * 8
IN_INS_PITCH        = 11 * 8
IN_INS_ROLL         = 12 * 8
IN_INS_ATT_VALID    = 13 * 8
IN_INS_AX           = 14 * 8
IN_INS_AY           = 15 * 8
IN_INS_AZ           = 16 * 8
IN_INS_WX           = 17 * 8
IN_INS_WY           = 18 * 8
IN_INS_WZ           = 19 * 8
IN_INS_VE           = 20 * 8
IN_INS_VN           = 21 * 8
IN_INS_VD           = 22 * 8
IN_CURRENT_DEPTH    = 23 * 8
IN_DEPTH_VALID      = 24 * 8
IN_LAST_ATT_TIME    = 25 * 8
IN_LAST_DEPTH_TIME  = 26 * 8
INPUT_END           = 27 * 8   # 216

# OUTPUT offsets
OUT_MOTOR0          = INPUT_END + 0 * 8
OUT_MOTOR1          = INPUT_END + 1 * 8
OUT_MOTOR2          = INPUT_END + 2 * 8
OUT_MOTOR3          = INPUT_END + 3 * 8
OUT_MOTOR4          = INPUT_END + 4 * 8
OUT_MOTOR5          = INPUT_END + 5 * 8
OUT_MOTOR6          = INPUT_END + 6 * 8
OUT_MOTOR7          = INPUT_END + 7 * 8
OUT_DEPTH_PID       = INPUT_END + 8 * 8
OUT_ROLL_PID        = INPUT_END + 9 * 8
OUT_PITCH_PID       = INPUT_END + 10 * 8
OUT_YAW_PID         = INPUT_END + 11 * 8
OUT_FZ_FF           = INPUT_END + 12 * 8
OUT_INITIALIZED     = INPUT_END + 13 * 8
OUT_FIXED_STAGE     = INPUT_END + 14 * 8
OUT_TS              = INPUT_END + 15 * 8
OUT_MODE            = INPUT_END + 16 * 8

# 死区 (与 motor_controller.py apply_deadzone 一致)
DEADZONE = 0.08

MOTOR_IDS = [0, 1, 2, 3, 5, 6, 7]


def apply_deadzone(v):
    if abs(v) < DEADZONE:
        return 0.0
    return float(v)


class MotorRosBridge(Node):
    """ROS2 ↔ C 共享内存桥"""

    def __init__(self):
        super().__init__('motor_ros_bridge')

        # ── 打开共享内存 (C 程序已创建并 truncate) ──
        self._shm_fd = None
        self._shm = None
        self._open_shm()

        # ── 桥接状态 (与原 motor_controller.py cmd_callback 逻辑一致) ──
        self.last_move = 0.0
        self.last_up = 0.0
        self.last_yaw = 0.0
        self.last_dive_flag = 0.0
        self.target_depth = 0.5
        self.yaw_hold_active = False
        self.yaw_hold_target = 0.0
        self.yaw_captured = False
        self.yaw_first_msg = False
        self.e_stopped = False
        self.gear = 1

        # 传感器缓存
        self.ins_yaw = 0.0
        self.ins_pitch = 0.0
        self.ins_roll = 0.0
        self.ins_att_valid = False
        self.ins_ax = self.ins_ay = self.ins_az = 0.0
        self.ins_wx = self.ins_wy = self.ins_wz = 0.0
        self.ins_ve = self.ins_vn = self.ins_vd = 0.0
        self.current_depth = 0.0
        self.depth_valid = False
        self.last_att_time = 0.0
        self.last_depth_time = 0.0

        # ── 订阅 ──
        self._cmd_sub = self.create_subscription(
            Twist, '/rov/cmd_vel', self.cmd_callback, 10)
        self._joy_sub = self.create_subscription(
            String, '/rov/joy_state', self.joy_state_callback, 10)
        self._att_sub = self.create_subscription(
            Vector3, '/ins/attitude', self.att_callback, 10)
        self._depth_sub = self.create_subscription(
            Float32, '/rov/depth', self.depth_callback, 10)
        self._acc_sub = self.create_subscription(
            Vector3, '/ins/acceleration', self.acc_callback, 10)
        self._gyro_sub = self.create_subscription(
            Vector3, '/ins/angular_rate', self.gyro_callback, 10)
        self._vel_sub = self.create_subscription(
            Vector3, '/ins/velocity', self.vel_callback, 10)

        # ── 状态发布 ──
        self._state_pub = self.create_publisher(String, '/rov/motor_state', 10)

        # ── 10Hz 读 OUTPUT 发布状态 ──
        self._state_timer = self.create_timer(0.1, self._publish_state)

        self.get_logger().info(
            'motor_ros_bridge v9.0 启动 (C+mmap 架构), SHM={}'.format(SHM_PATH))

    # ─────────────────────────────────────────────────────────
    # 共享内存
    # ─────────────────────────────────────────────────────────
    def _open_shm(self):
        """打开/创建共享内存 (C 程序可能尚未启动, 这里确保文件存在)"""
        try:
            fd = os.open(SHM_PATH, os.O_RDWR | os.O_CREAT, 0o666)
            os.ftruncate(fd, SHM_SIZE)
            self._shm = mmap.mmap(fd, SHM_SIZE, mmap.MAP_SHARED,
                                  mmap.PROT_READ | mmap.PROT_WRITE)
            os.close(fd)
            self._shm_fd = None
        except Exception as e:
            self.get_logger().error('共享内存打开失败: {}'.format(e))
            self._shm = None

    def _write_d(self, offset, value):
        if self._shm is None:
            return
        try:
            self._shm[offset:offset + 8] = struct.pack('<d', float(value))
        except Exception:
            pass

    def _read_d(self, offset):
        if self._shm is None:
            return 0.0
        try:
            return struct.unpack('<d', bytes(self._shm[offset:offset + 8]))[0]
        except Exception:
            return 0.0

    def _flush_cmd_to_shm(self):
        """把当前指令状态写入共享内存 INPUT 区"""
        now = time.time()
        self._write_d(IN_MOVE, self.last_move)
        self._write_d(IN_UP, self.last_up)
        self._write_d(IN_YAW, self.last_yaw)
        self._write_d(IN_DIVE_FLAG, self.last_dive_flag)
        self._write_d(IN_TARGET_DEPTH, self.target_depth)
        self._write_d(IN_YAW_HOLD_ACTIVE, 1.0 if self.yaw_hold_active else 0.0)
        self._write_d(IN_YAW_HOLD_TARGET, self.yaw_hold_target)
        self._write_d(IN_LAST_CMD_TIME, now)
        self._write_d(IN_E_STOPPED, 1.0 if self.e_stopped else 0.0)
        self._write_d(IN_GEAR, self.gear)

    def _flush_sensor_to_shm(self):
        """把传感器状态写入共享内存 INPUT 区"""
        self._write_d(IN_INS_YAW, self.ins_yaw)
        self._write_d(IN_INS_PITCH, self.ins_pitch)
        self._write_d(IN_INS_ROLL, self.ins_roll)
        self._write_d(IN_INS_ATT_VALID, 1.0 if self.ins_att_valid else 0.0)
        self._write_d(IN_INS_AX, self.ins_ax)
        self._write_d(IN_INS_AY, self.ins_ay)
        self._write_d(IN_INS_AZ, self.ins_az)
        self._write_d(IN_INS_WX, self.ins_wx)
        self._write_d(IN_INS_WY, self.ins_wy)
        self._write_d(IN_INS_WZ, self.ins_wz)
        self._write_d(IN_INS_VE, self.ins_ve)
        self._write_d(IN_INS_VN, self.ins_vn)
        self._write_d(IN_INS_VD, self.ins_vd)
        self._write_d(IN_CURRENT_DEPTH, self.current_depth)
        self._write_d(IN_DEPTH_VALID, 1.0 if self.depth_valid else 0.0)
        self._write_d(IN_LAST_ATT_TIME, self.last_att_time)
        self._write_d(IN_LAST_DEPTH_TIME, self.last_depth_time)

    # ─────────────────────────────────────────────────────────
    # cmd_vel 回调 (与原 motor_controller.py cmd_callback 逻辑一致)
    # ─────────────────────────────────────────────────────────
    def cmd_callback(self, msg: Twist):
        dive_flag = msg.linear.y
        mv = apply_deadzone(msg.linear.x)
        yw = apply_deadzone(msg.angular.z)

        # ── 定航向 (angular.x > 0.1 = 开启, angular.y = 目标航向度) ──
        yaw_hold_new = msg.angular.x > 0.1
        if yaw_hold_new:
            if not self.yaw_hold_active:
                self.yaw_hold_active = True
                # 首次激活: 强制用 INS 当前 yaw 作为目标
                if self.ins_att_valid:
                    self.yaw_hold_target = self.ins_yaw
                    self.get_logger().info(
                        '  定航向已开启 (INS捕获: {:.1f}deg)'.format(self.yaw_hold_target))
                else:
                    self.yaw_hold_target = msg.angular.y
                    self.get_logger().warn(
                        '  定航向已开启 (无INS数据, 使用手柄值: {:.1f}deg)'.format(
                            self.yaw_hold_target))
                self.yaw_first_msg = True
            else:
                if self.yaw_first_msg:
                    if self.ins_att_valid and abs(msg.angular.y) < 0.5:
                        self.get_logger().info(
                            '  定航向: 忽略 joy_controller 的 0.0, 保持目标={:.1f}deg'.format(
                                self.yaw_hold_target))
                    else:
                        self.yaw_hold_target = msg.angular.y
                    self.yaw_first_msg = False
                else:
                    self.yaw_hold_target = msg.angular.y
            self.yaw_captured = True
        elif not yaw_hold_new and self.yaw_hold_active:
            self.yaw_hold_active = False
            self.yaw_captured = False
            self.yaw_first_msg = False
            self.get_logger().info('  定航向已关闭')

        # ── 模式判定 ──
        if dive_flag > 0.1:
            target_depth = float(msg.linear.z)
            up = 0.0
        else:
            up = apply_deadzone(msg.linear.z)
            target_depth = 0.0

        prev_was_depth = self.last_dive_flag > 0.1

        self.last_move = mv
        self.last_yaw = yw
        self.last_dive_flag = dive_flag

        if dive_flag > 0.1 and target_depth >= 0.01:
            # ── 定深模式 (悬停已开启) ──
            old_target = self.target_depth
            self.target_depth = max(0.0, target_depth)
            if abs(self.target_depth - old_target) > 0.001:
                self.get_logger().info(
                    '  目标深度更新: {:.2f}m → {:.2f}m'.format(
                        old_target, self.target_depth))
            self.last_up = 0.0
        else:
            # ── 手动模式 (包括定深档但悬停未开启) ──
            if dive_flag > 0.1:
                # 定深档但悬停未开启: 清空PID状态
                self.target_depth = 0.0
                self.yaw_captured = False
            self.last_up = up

        self._flush_cmd_to_shm()

    # ─────────────────────────────────────────────────────────
    # joy_state 回调 (解析 gear)
    # ─────────────────────────────────────────────────────────
    def joy_state_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            gear = int(data.get('gear', 1))
            if gear < 1:
                gear = 1
            if gear > 4:
                gear = 4
            if gear != self.gear:
                self.gear = gear
                self.get_logger().info('  档位切换 → {}档'.format(gear))
            estop = data.get('e_stopped', False)
            if estop != self.e_stopped:
                self.e_stopped = bool(estop)
                self.get_logger().info('  急停状态: {}'.format(self.e_stopped))
            self._flush_cmd_to_shm()
        except Exception as e:
            self.get_logger().warn('joy_state 解析失败: {}'.format(e))

    # ─────────────────────────────────────────────────────────
    # 传感器回调
    # ─────────────────────────────────────────────────────────
    def att_callback(self, msg: Vector3):
        self.ins_yaw = msg.z
        self.ins_pitch = msg.y
        self.ins_roll = msg.x
        self.ins_att_valid = True
        self.last_att_time = time.time()
        self._flush_sensor_to_shm()

    def depth_callback(self, msg: Float32):
        self.current_depth = float(msg.data)
        self.depth_valid = True
        self.last_depth_time = time.time()
        self._flush_sensor_to_shm()

    def acc_callback(self, msg: Vector3):
        self.ins_ax = msg.x
        self.ins_ay = msg.y
        self.ins_az = msg.z
        self._flush_sensor_to_shm()

    def gyro_callback(self, msg: Vector3):
        self.ins_wx = msg.x
        self.ins_wy = msg.y
        self.ins_wz = msg.z
        self._flush_sensor_to_shm()

    def vel_callback(self, msg: Vector3):
        self.ins_ve = msg.x
        self.ins_vn = msg.y
        self.ins_vd = msg.z
        self._flush_sensor_to_shm()

    # ─────────────────────────────────────────────────────────
    # 10Hz 读 OUTPUT → 发布 /rov/motor_state
    # ─────────────────────────────────────────────────────────
    def _publish_state(self):
        motors = {
            0: int(self._read_d(OUT_MOTOR0)),
            1: int(self._read_d(OUT_MOTOR1)),
            2: int(self._read_d(OUT_MOTOR2)),
            3: int(self._read_d(OUT_MOTOR3)),
            5: int(self._read_d(OUT_MOTOR5)),
            6: int(self._read_d(OUT_MOTOR6)),
            7: int(self._read_d(OUT_MOTOR7)),
        }
        mode_int = int(self._read_d(OUT_MODE))
        mode_names = {0: 'MANUAL', 1: 'FIXED', 2: 'FIXED_UP', 3: 'PID', 4: 'YAW_HOLD'}
        data = json.dumps({
            'move_norm': round(self.last_move, 3),
            'up_norm': round(self.last_up, 3),
            'yaw_norm': round(self.last_yaw, 3),
            'dive_flag': round(self.last_dive_flag, 3),
            'target_depth': round(self.target_depth, 2),
            'current_depth': round(self.current_depth, 3),
            'depth_valid': self.depth_valid,
            'depth_pid_out': round(self._read_d(OUT_DEPTH_PID), 3),
            'fz_ff': round(self._read_d(OUT_FZ_FF), 4),
            'roll_pid_out': round(self._read_d(OUT_ROLL_PID), 3),
            'pitch_pid_out': round(self._read_d(OUT_PITCH_PID), 3),
            'yaw_pid_out': round(self._read_d(OUT_YAW_PID), 3),
            'yaw_hold_active': self.yaw_hold_active,
            'yaw_hold_target': round(self.yaw_hold_target, 1),
            'ins_yaw': round(self.ins_yaw, 1),
            'ins_pitch': round(self.ins_pitch, 1),
            'ins_roll': round(self.ins_roll, 1),
            'ins_att_valid': self.ins_att_valid,
            'ins_ax': round(self.ins_ax, 3),
            'ins_ay': round(self.ins_ay, 3),
            'ins_az': round(self.ins_az, 3),
            'ins_wx': round(self.ins_wx, 3),
            'ins_wy': round(self.ins_wy, 3),
            'ins_wz': round(self.ins_wz, 3),
            'ins_ve': round(self.ins_ve, 3),
            'ins_vn': round(self.ins_vn, 3),
            'ins_vd': round(self.ins_vd, 3),
            'motors': motors,
            'initialized': self._read_d(OUT_INITIALIZED) > 0.5,
            'fixed_stage': self._read_d(OUT_FIXED_STAGE) > 0.5,
            'gear': self.gear,
            'e_stopped': self.e_stopped,
            'mode': mode_names.get(mode_int, 'UNKNOWN'),
            'ts': time.time(),
        })
        self._state_pub.publish(String(data=data))

    def shutdown(self):
        self.get_logger().info('桥关闭')


def main():
    rclpy.init()
    node = MotorRosBridge()

    def on_signal(sig, frame):
        node.get_logger().info('收到信号 {}, 退出'.format(sig))
        rclpy.shutdown()
        os._exit(0)

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGHUP, on_signal)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
