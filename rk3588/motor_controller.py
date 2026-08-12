#!/usr/bin/env python3
"""
ROV 电机控制器 v8.6 (RK3588 端) — 手动模式动力重做 + 增强下潜推力 + 可调Yaw PD + CAN恢复

v8.6 改进 (手动遥控动力分配重做):
  - 转向增强: 手动模式 YAW_MANUAL_TRIM 0.10→0.60 (ID7 满量程 1130→1280 RPM),
    尾推Yaw参与比例 0.5→0.6 (手动专属), 转向力矩大幅提升
  - 手动下潜同步增强: 手动垂直增益与定深阶段1对齐 (尾推 1180→1250, 垂推 1400→1480 RPM),
    上浮推力保持不变 (方向不对称增益)
  - 手动/自动增益分离: 定深/定航向逻辑完全不受影响
    YAW_MANUAL_TRIM_AUTO=0.10 / TAIL_YAW_RATIO_AUTO=0.5 与 v8.5 完全一致

v8.5 改进:
  - 机器人重量减轻, 浮力增大, 原推力不足以实现下潜
  - 下潜固定推力阶段: 尾推 1180→1250 RPM, 垂推 1405→1480 RPM
  - PID阶段保底推力: 尾推 1140→1200 RPM, 垂推 1250→1350 RPM
  - 深度PID: Kp 2.0→2.5, I_MAX 0.30→0.40 (适应更大浮力补偿需求)
  - 推力平衡: 尾推/垂推比值保持~0.84, 尾推垂直分量占比39%不变, 无倾斜/偏航风险
  - 上浮推力保持不变 (机器人变轻, 上浮更容易)

v7.7 改进:
  - YAW_DIRECTION 只应用于 PID 输出 (定航向 + 定深阶段), 手动 steering 不翻转发方向
  - 修复: 手柄左转→机器人右转的问题 (YAW_DIRECTION 全局应用导致手动方向反转)

v7.6 改进:
  - Yaw PID 大幅提升增益: KP 0.06→0.15, KI 0.02→0.06, I_MAX 0.30→0.50
  - 死区收紧: 0.3°→0.15°, 门控降低: 3.0°→2.0°
  - 稳态1°误差时 ID7 RPM: 1208→1295, 抵抗恒定外部偏转力矩

v7.5 改进:
  - YAW_DIRECTION=-1: ID7物理方向修正 (电机正转=左转, 需要取反才能实现+ =右转)
  - 解决定航向时 ROV 往目标反方向转的问题 (目标yaw与实际yaw相差~180°)

v7.3 改进:
  - 尾推(ID0-3)重新参与Yaw转向, 但按 TAIL_YAW_RATIO=0.5 缩放 (通过 B+ 伪逆自动分配)
  - ID7 仍为主控 (100% mz), 尾推辅助 (50% mz), 保证 ID7 RPM > 尾推 RPM
  - 力平衡分析: Fx=0 (前后抵消), Fy≈0.3*mz (小侧漂), Mz=0.79*mz (尾推77%+ID7 100%归一化叠加)
  - 定航向首次激活时, 如果 joy_controller 因 yaw_captured=False 发送 0.0,
    motor_controller 忽略该值, 保持 INS 捕获的当前航向作为目标
  - 解决"开启定航后 ROV 自动往 0° 猛转"的问题

v7.2 改进:
  - Yaw力矩完全由ID7独立控制, 尾推(ID0-3)不再参与Yaw (避免4尾推合力>ID7单电机)
  - 定航向首次激活强制以INS当前yaw为目标 (不受joy_controller可能发来的0值影响)

v7.1 改进:
  - 定航向两阶段控制: |误差|>10°大转速回正(1400RPM), |误差|≤10°PID微调
  - YAW_RPM_MAX 提升至 1400

v7.0 改进:
  - 两阶段深度控制: 固定推力阶段 + PID精细控制阶段
  - 阶段1 (误差 > 0.10m): 下潜尾推1250/垂推1480, 上浮尾推1180/垂推1400
  - 阶段2 (误差 ≤ 0.10m): PID 精细控制, 稳定悬浮
  - 尾推绝对值不超过垂推, 支持正负双向

6-DOF 控制:
  linear.x  = move      前进/后退 (-1~+1)
  linear.y  = dive_flag  定深档标志 (>0.1 表示定深模式)
  linear.z  = target_depth / up_norm (定深模式: 目标深度米; 手动模式: 垂直推力)
  angular.x = yaw_hold_flag  定航向开关 (>0.1=开启)
  angular.y = yaw_hold_target 目标航向 (度, -180~+180)
  angular.z = yaw        手动偏航修正 (-1~+1)

推力分配:
  tau = [Fx, Fy, Fz, Mx, My, Mz]  (PID 输出 + 手柄输入)
  u = B+ * tau                     (7 电机归一化命令)
  rpm_i = norm_to_rpm(u_i)         (转 RPM)
  CAN 反相 (ID1/3/6) 由 build_ctrl 处理

CAN 帧格式 (与 can_motor_v1.0.c 的 build_ctrl_200/201 完全一致):
  帧 0x200 (电机 0~3): ID0直接, ID1取反, ID2直接, ID3取反
  帧 0x201 (电机 4~7): ID4未用, ID5直接, ID6取反, ID7直接
  rpm_to_cmd: bit[0:10]=abs(rpm), bit[11]=0(正转)/1(反转)

安全机制:
  - 超过 TIMEOUT_SEC 秒未收到新命令 → 自动全停
  - pitch 超阈值线性降推, 防翻覆
  - 初始化时发送两轮零帧使能电机
  - 退出时发送全零停机帧
"""

import os
os.environ['ROS_DOMAIN_ID'] = '42'

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String, Float32
import socket
import struct
import threading
import time
import json
import signal
import sys

# 推力分配矩阵
from thrust_allocator import allocate, MOTOR_IDS

# ── CAN 常量 ─────────────────────────────────────────────────────
CAN_INTERFACE = 'can0'
CAN_FRAME_200 = 0x200   # 电机 0~3
CAN_FRAME_201 = 0x201   # 电机 4~7

# ── 配置 ─────────────────────────────────────────────────────────
MAX_RPM       = 2000    # 绝对上限
MIN_RPM       = 1100    # 最小启动转速 (电机<1100不转)
TIMEOUT_SEC   = 5.0     # 命令超时

# ── RPM 范围 (各电机类型) ────────────────────────────────────────
TAIL_RPM_MIN  = 1100
TAIL_RPM_MAX  = 1550
VERT_RPM_MIN  = 1100
VERT_RPM_MAX  = 1550
YAW_RPM_MIN   = 1100
YAW_RPM_MAX   = 1400  # v7.1 定航向大误差回正

# ── 推力增益 ─────────────────────────────────────────────────────
FZ_GAIN_TAIL  = 0.178  # 尾推垂直增益 (fz=1.0→1180RPM, 倾角22.5deg) — 定深PID阶段2使用
FZ_GAIN_VERT  = 0.667  # 垂推增益 (fz=1.0→1400RPM, 纯垂直) — 定深PID阶段2使用
TAIL_DIVE_MIN = 0.223  # v8.5: 尾推下潜最低norm (→1200RPM, 克服浮力)
VERT_DIVE_MIN = 0.556  # v8.5: 垂推下潜最低norm (→1350RPM, 克服浮力)

# v8.6: 手动模式垂直增益 — 方向不对称 (下潜增强, 上浮保持v8.5)
# 下潜满量程与定深阶段1一致 (尾推1250/垂推1480 RPM), 上浮沿用 FZ_GAIN_* (1180/1400 RPM)
MANUAL_DIVE_FZ_TAIL = 0.334   # 手动下潜尾推 → 1250 RPM
MANUAL_DIVE_FZ_VERT = 0.845   # 手动下潜垂推 → 1480 RPM

# ── v7.0: 两阶段深度控制 ─────────────────────────────────────────
DEPTH_FIXED_THRESHOLD = 0.10  # 误差超过此值 → 固定推力阶段
DIVE_TAIL_NORM = 0.334   # v8.5: 下潜尾推 → 1250 RPM (增推克服浮力)
DIVE_VERT_NORM = 0.845   # v8.5: 下潜垂推 → 1480 RPM
SURF_TAIL_NORM = 0.178   # 上浮尾推 → 1180 RPM (不变, 上浮更容易)
SURF_VERT_NORM = 0.667   # 上浮垂推 → 1400 RPM (不变)

# ── 深度 PID ─────────────────────────────────────────────────────
DEPTH_KP       = 2.5    # v8.5: 2.0→2.5 (更快响应, 适应增大浮力)
DEPTH_KI       = 0.10
DEPTH_I_MAX    = 0.40   # v8.5: 0.30→0.40 (更大积分裕度补偿稳态浮力)
DEPTH_I_GATE   = 0.50   # 超过此误差不积分
DEPTH_I_DECAY  = 0.85
DEPTH_DEADBAND = 0.05   # 5cm 死区
DEPTH_TIMEOUT  = 3.0

# ── Roll PID ─────────────────────────────────────────────────────
ROLL_KP       = 0.10
ROLL_KI       = 0.02
ROLL_I_MAX    = 0.20
ROLL_DBAND    = 1.0     # 度 (增大死区)
ROLL_I_GATE   = 3.0     # 度
ROLL_I_DECAY  = 0.80

# ── Pitch PID (v6.0 新增) ────────────────────────────────────────
PITCH_KP      = 0.10
PITCH_KI      = 0.02
PITCH_I_MAX   = 0.20
PITCH_DBAND   = 1.5     # 度 (增大死区)
PITCH_I_GATE  = 5.0     # 度
PITCH_I_DECAY = 0.85

# ── Yaw PD 控制器 (v8.4: 可调PD, I=0起调) ──────────────────────────
# PD 公式: mz = KP * err + KD * (err - last_err)/dt
# 映射: mz→ID7 RPM = norm_to_rpm(mz, 1100, 1400)
#   mz=0    → ID7=1100 (停转)
#   mz=0.5  → ID7=1250
#   mz=1.0  → ID7=1400
YAW_KP         = 0.5   # v8.4: 大幅提升比例增益 (1°误差→mz=0.5→ID7=1250)
YAW_KD         = 0.3   # v8.4: 微分增益 (阻尼震荡, 先设KD/KP≈0.6)
YAW_KI         = 0.0   # v8.4: 积分置零, 先用纯PD验证
YAW_I_MAX      = 0.50  # 积分限幅 (当前KI=0不使用)
YAW_DEADBAND   = 0.15  # 死区 ±0.15° (目标±1°)
YAW_I_GATE     = 2.0   # 积分门控 (当前KI=0不使用)
YAW_I_DECAY    = 0.85
YAW_MANUAL_TRIM_MANUAL = 0.60  # v8.6: 手动模式转向增益 (满量程→ID7 1280RPM, 灵敏响应)
YAW_MANUAL_TRIM_AUTO   = 0.10  # v8.6: 定深模式偏置微调 (与v8.5一致, 不影响定深)
YAW_ATT_TIMEOUT = 1.0
YAW_HOLD_THRESHOLD = 10.0  # v7.1: 定航向大误差阈值(度), 超此值大转速回正
YAW_DIRECTION = -1  # v7.4: ID7物理方向修正 (+1=正转右转, -1=正转左转)
TAIL_YAW_RATIO_AUTO   = 0.5    # v7.3: 尾推Yaw参与比例 (定深/定航向, 力平衡最优=0.707, RPM约束选0.5)
TAIL_YAW_RATIO_MANUAL = 0.6    # v8.6: 尾推Yaw参与比例 (手动模式, 增强转向)

# ── pitch 安全 (防翻覆) ──────────────────────────────────────────
PITCH_SAFE     = 30     # 开始降推的角度
PITCH_KILL     = 55     # 推力归零的角度

# ── 传感器超时 ───────────────────────────────────────────────────
ATT_TIMEOUT       = 2.0
DEPTH_TIMEOUT_SENSOR = 3.0

# ── 手动模式推力混合 (mix_thrust_manual) ─────────────────────────
MANUAL_VERT_BASE   = 1600   # 全量程基数
TAIL_VERTICAL_RATIO = 0.5   # 尾部承担比例
UP_VERT_OFF        = 200    # 垂推偏移 (下潜更多)
PITCH_GAIN         = 0.05   # pitch 调节增益
ROLL_GAIN_M        = 300    # roll 差分增益 (手动)

# ── 深度前馈补偿 (v8.0: 数据驱动, 辅助PID) ─────────────────────────
# 模型: fz_ff = FF_BIAS + FF_DEPTH_COEFF*target_depth + FF_SIN_PITCH*sin(pitch) + FF_SIN_ROLL*sin(roll)
# 训练后更新系数, 设置 FF_GAIN > 0 启用
FF_GAIN            = 0.0    # 前馈增益 (0=禁用, 建议 0.5~1.0)
FF_BIAS            = 0.0    # w0
FF_DEPTH_COEFF     = 0.0    # w1
FF_SIN_PITCH_COEFF = 0.0    # w2
FF_SIN_ROLL_COEFF  = 0.0    # w3


def apply_deadzone(v):
    """去除微小抖动"""
    if abs(v) < 0.003:
        return 0.0
    return v


def _angle_diff(target, current):
    """最小角度差 (-180..180), 用于 yaw"""
    d = target - current
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def norm_to_rpm(norm, min_rpm, max_rpm):
    """归一化值 [-1,+1] → RPM [min_rpm, max_rpm]
    零输入→零输出, 非零>0 → min_rpm+(max-min)*|norm|
    """
    if abs(norm) < 0.001:
        return 0
    sign = 1 if norm > 0 else -1
    rpm = int(min_rpm + abs(norm) * (max_rpm - min_rpm))
    rpm = _clamp(rpm, min_rpm, max_rpm)
    return sign * rpm


def rpm_clamp_raw(v, min_rpm=MIN_RPM):
    """钳位 RPM 到 ±MAX_RPM，非零时 ≥ min_rpm"""
    if abs(v) < 0.5:
        return 0
    sign = 1 if v > 0 else -1
    rpm = int(round(abs(v)))
    rpm = max(0, min(MAX_RPM, rpm))
    if rpm > 0 and rpm < min_rpm:
        rpm = min_rpm
    return sign * rpm


def rpm_to_cmd(rpm):
    """RPM → CAN 编码: bit[0:10]=abs, bit[11]=方向(1=反转)"""
    if rpm == 0:
        return 0
    abs_v = -rpm if rpm < 0 else rpm
    cmd = abs_v & 0x07FF
    if rpm < 0:
        cmd |= 0x0800
    return cmd


def write_cmd_le(buf, offset, cmd):
    """写入 2 字节 LE 到缓冲区"""
    buf[offset]     = cmd & 0xFF
    buf[offset + 1] = (cmd >> 8) & 0xFF


def build_ctrl_200(g):
    """帧 0x200: 电机 0~3"""
    buf = bytearray(8)
    write_cmd_le(buf, 0, rpm_to_cmd( g[0]))
    write_cmd_le(buf, 2, rpm_to_cmd(-g[1]))
    write_cmd_le(buf, 4, rpm_to_cmd( g[2]))
    write_cmd_le(buf, 6, rpm_to_cmd(-g[3]))
    return bytes(buf)


def build_ctrl_201(g):
    """帧 0x201: 电机 4~7"""
    buf = bytearray(8)
    write_cmd_le(buf, 0, 0)
    write_cmd_le(buf, 2, rpm_to_cmd( g[5]))
    write_cmd_le(buf, 4, rpm_to_cmd(-g[6]))
    write_cmd_le(buf, 6, rpm_to_cmd( g[7]))
    return bytes(buf)


def mix_thrust_manual(move_norm, up_norm, yaw_norm, roll_norm=0.0, pitch_norm=0.0):
    """手动模式推力混合 (v4.0: 保持原逻辑不变)"""
    m = move_norm * MANUAL_VERT_BASE
    u = up_norm   * MANUAL_VERT_BASE
    y = yaw_norm  * MANUAL_VERT_BASE

    u_tail = u * TAIL_VERTICAL_RATIO
    if u > 0:
        u_vert = min(u + UP_VERT_OFF, MAX_RPM)
    elif u < 0:
        u_vert = max(u - UP_VERT_OFF, -MAX_RPM)
    else:
        u_vert = 0.0

    if u != 0:
        pitch_adjust = pitch_norm * PITCH_GAIN
        p_ratio = TAIL_VERTICAL_RATIO + pitch_adjust
        p_ratio = max(0.1, min(0.7, p_ratio))
        u_tail = u * p_ratio

    g0 = (+m - u_tail)
    g1 = (+m + u_tail)
    g2 = (+m + u_tail)
    g3 = (+m - u_tail)
    g5 = (+u_vert)
    g6 = (+u_vert)
    g7 = (+y)

    roll_delta = roll_norm * ROLL_GAIN_M
    g0 -= roll_delta; g1 += roll_delta
    g2 += roll_delta; g3 -= roll_delta

    max_abs = max(abs(g0), abs(g1), abs(g2), abs(g3))
    if max_abs > MAX_RPM:
        scale = MAX_RPM / max_abs
        g0 *= scale; g1 *= scale; g2 *= scale; g3 *= scale

    return {
        0: rpm_clamp_raw(g0),
        1: rpm_clamp_raw(g1),
        2: rpm_clamp_raw(g2),
        3: rpm_clamp_raw(g3),
        5: rpm_clamp_raw(g5),
        6: rpm_clamp_raw(g6),
        7: rpm_clamp_raw(g7),
    }


# ── 全局 CAN socket ──────────────────────────────────────────────
_can_sock = None
_can_lock = threading.Lock()
_can_fail_count = 0       # 连续发送失败计数
_can_recovering = False   # 是否正在恢复中
_CAN_MAX_FAILS = 3        # 连续失败多少次后重建 socket


def can_init():
    """初始化 CAN socket, 发送使能帧"""
    global _can_sock, _can_fail_count, _can_recovering
    _can_fail_count = 0
    _can_recovering = False
    try:
        if _can_sock is not None:
            try:
                _can_sock.close()
            except Exception:
                pass
        _can_sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        _can_sock.bind((CAN_INTERFACE,))
    except Exception as e:
        print('[FATAL] CAN socket 失败: {}'.format(e))
        return False

    zero = bytes(8)
    try:
        send_can_frame(CAN_FRAME_200, zero)
        time.sleep(0.05)
        send_can_frame(CAN_FRAME_201, zero)
        time.sleep(0.05)
        send_can_frame(CAN_FRAME_200, zero)
        time.sleep(0.05)
        send_can_frame(CAN_FRAME_201, zero)
    except Exception as e:
        print('[WARN] CAN init 发送失败: {}'.format(e))
        return False
    return True


def can_recover():
    """CAN 通信恢复: 关闭旧 socket, 重新初始化"""
    global _can_recovering
    with _can_lock:
        if _can_recovering:
            return False  # 已有恢复在进行
        _can_recovering = True
    
    print('[CAN] 尝试恢复 CAN 通信...')
    # 关闭旧 socket
    global _can_sock
    if _can_sock is not None:
        try:
            _can_sock.close()
        except Exception:
            pass
        _can_sock = None
    
    # 尝试重启 CAN 接口 (bus-off 恢复)
    import subprocess
    try:
        subprocess.run(['ip', 'link', 'set', CAN_INTERFACE, 'restart'],
                       timeout=2, capture_output=True)
        time.sleep(0.2)
    except Exception:
        pass
    
    # 重新初始化 socket
    ok = can_init()
    
    with _can_lock:
        _can_recovering = False
    return ok


def can_close():
    global _can_sock
    with _can_lock:
        if _can_sock is not None:
            try:
                zero = bytes(8)
                _can_sock.send(struct.pack('=IB3x8s', CAN_FRAME_200 & 0x1FFFFFFF, 8, zero))
                _can_sock.send(struct.pack('=IB3x8s', CAN_FRAME_201 & 0x1FFFFFFF, 8, zero))
            except Exception:
                pass
            try:
                _can_sock.close()
            except Exception:
                pass
            _can_sock = None


def send_can_frame(can_id, data):
    """发送 CAN 帧, 带错误恢复"""
    global _can_sock, _can_fail_count
    with _can_lock:
        if _can_sock is None:
            _can_fail_count += 1  # socket 不存在也算失败
            raise OSError('CAN socket is None')
        frame = struct.pack('=IB3x8s', can_id & 0x1FFFFFFF, 8, data)
        try:
            _can_sock.send(frame)
            _can_fail_count = 0  # 发送成功, 重置计数
        except OSError:
            _can_fail_count += 1
            raise  # 重新抛出, 让调用方知道


def send_motor_rpm(g_motor):
    """发送所有电机 RPM, 带 CAN 错误恢复"""
    global _can_fail_count
    try:
        send_can_frame(CAN_FRAME_200, build_ctrl_200(g_motor))
        send_can_frame(CAN_FRAME_201, build_ctrl_201(g_motor))
    except (OSError, Exception):
        # CAN 发送失败或 socket 为 None, 尝试恢复
        if _can_fail_count >= _CAN_MAX_FAILS and not _can_recovering:
            # 在新线程中恢复, 避免阻塞心跳
            import threading
            t = threading.Thread(target=can_recover, daemon=True)
            t.start()


# ═══════════════════════════════════════════════════════════════════
#  ROS2 节点
# ═══════════════════════════════════════════════════════════════════

class MotorController(Node):
    def __init__(self):
        super().__init__('rov_motor_controller')

        self.state_pub = self.create_publisher(String, '/rov/motor_state', 10)
        self.echo_pub  = self.create_publisher(Twist, '/rov/cmd_vel_echo', 10)

        self.sub = self.create_subscription(
            Twist, '/rov/cmd_vel', self.cmd_callback, 10)

        # ═══════════════════════════════════════════════════
        # v4.0: 传感器状态 (由 subprocess 桥接进程填充)
        # ═══════════════════════════════════════════════════
        self.ins_yaw        = 0.0
        self.ins_pitch      = 0.0
        self.ins_roll       = 0.0
        self.ins_att_valid  = False
        self.last_att_time  = 0.0

        # v8.1: INS 加速度 + 角速度 (用于 CSV 记录)
        self.ins_ax = 0.0
        self.ins_ay = 0.0
        self.ins_az = 0.0
        self.ins_wx = 0.0
        self.ins_wy = 0.0
        self.ins_wz = 0.0
        self.ins_ve = 0.0
        self.ins_vn = 0.0
        self.ins_vd = 0.0

        self.current_depth     = 0.0
        self.filtered_depth    = 0.0
        self.depth_valid       = False
        self.last_depth_time   = 0.0
        self._sensor_first     = False

        # ═══════════════════════════════════════════════════
        # v4.0: 深度 PID 状态
        # ═══════════════════════════════════════════════════
        self.target_depth    = 0.5
        self.depth_err_i     = 0.0
        self.depth_pid_out   = 0.0   # 归一化 [-1,+1]
        self.fz_ff           = 0.0   # v8.0: 前馈补偿输出

        # ═══════════════════════════════════════════════════
        # v5.5: Roll PID 状态
        # ═══════════════════════════════════════════════════
        self.roll_err_i      = 0.0
        self.roll_pid_out    = 0.0

        # ═══════════════════════════════════════════════════
        # v6.0: Pitch PID 状态
        # ═══════════════════════════════════════════════════
        self.pitch_err_i     = 0.0
        self.pitch_pid_out   = 0.0

        # ═══════════════════════════════════════════════════
        # v4.0: Yaw PD 状态 (v8.4: PID→PD, KI=0)
        # ═══════════════════════════════════════════════════
        self.yaw_target      = 0.0
        self.yaw_captured    = False
        self.yaw_err_i       = 0.0
        self.yaw_pid_out     = 0.0   # 归一化 [-1,+1]
        self._last_yaw_err   = None  # v8.4: PD 微分需要上次误差
        self.last_yaw_pid_time = time.time()  # v8.4: PD 微分需要 dt

        # ── v7.0: 独立定航向 ──
        self.yaw_hold_active = False
        self.yaw_hold_target = 0.0
        self._yaw_first_msg  = False  # v7.3: 首次激活后忽略 joy_controller 的初始 0.0

        # ═══════════════════════════════════════════════════
        # 命令状态
        # ═══════════════════════════════════════════════════
        self.last_cmd_time   = time.time()
        self.last_motors     = {0: 0, 1: 0, 2: 0, 3: 0, 5: 0, 6: 0, 7: 0}
        self.last_g          = [0] * 8
        self.last_move       = 0.0
        self.last_up         = 0.0
        self.last_yaw        = 0.0
        self.last_roll       = 0.0
        self.last_pitch      = 0.0
        self.last_dive_flag  = 0.0
        self.initialized     = False
        self._hb_log_count   = 0     # 心跳计数器

        # ── 定时器 ──
        # v6.0: 不再使用定时器, 直接在main()的while循环中调用heartbeat_tick()
        # 发布 /rov/motor_state (2Hz)
        self.create_timer(0.5, self.publish_state)
        # 安全: 检测手柄超时
        self.last_cmd_time = time.time()

        self.get_logger().info('=' * 60)
        self.get_logger().info('  ROV 电机控制器 v8.5 (增强下潜推力+Yaw PD + CAN恢复+健康监控+INS速度)')
        self.get_logger().info('=' * 60)
        self.get_logger().info('  分配: B+ (7x6) | 4 PID: depth/roll/pitch/yaw')
        self.get_logger().info('  尾推: {}-{}rpm  垂推: {}-{}rpm  横推: {}-{}rpm'.format(
            TAIL_RPM_MIN, TAIL_RPM_MAX, VERT_RPM_MIN, VERT_RPM_MAX, YAW_RPM_MIN, YAW_RPM_MAX))
        self.get_logger().info('  Depth PID: Kp={} Ki={} Imax={} deadband={:.2f}m'.format(
            DEPTH_KP, DEPTH_KI, DEPTH_I_MAX, DEPTH_DEADBAND))
        self.get_logger().info('  Dive thrust: tail={}rpm vert={}rpm | PID min: tail={}rpm vert={}rpm'.format(
            norm_to_rpm(DIVE_TAIL_NORM, TAIL_RPM_MIN, TAIL_RPM_MAX),
            norm_to_rpm(DIVE_VERT_NORM, VERT_RPM_MIN, VERT_RPM_MAX),
            norm_to_rpm(TAIL_DIVE_MIN, TAIL_RPM_MIN, TAIL_RPM_MAX),
            norm_to_rpm(VERT_DIVE_MIN, VERT_RPM_MIN, VERT_RPM_MAX)))
        self.get_logger().info('  Roll PID:  Kp={} Ki={} deadband={:.1f}deg'.format(
            ROLL_KP, ROLL_KI, ROLL_DBAND))
        self.get_logger().info('  Pitch PID: Kp={} Ki={} deadband={:.1f}deg'.format(
            PITCH_KP, PITCH_KI, PITCH_DBAND))
        self.get_logger().info('  Yaw PD:    Kp={} Kd={} Ki={} deadband={:.2f}deg'.format(
            YAW_KP, YAW_KD, YAW_KI, YAW_DEADBAND))
        self.get_logger().info('  Pitch安全: {}deg降推 -> {}deg归零'.format(
            PITCH_SAFE, PITCH_KILL))
        self.get_logger().info('  Depth FF: GAIN={} bias={:.4f} w_d={:.4f} w_p={:.4f} w_r={:.4f}'.format(
            FF_GAIN, FF_BIAS, FF_DEPTH_COEFF, FF_SIN_PITCH_COEFF, FF_SIN_ROLL_COEFF))
        self.get_logger().info('  初始化 CAN socket...')
        self.get_logger().info('=' * 60)

        if can_init():
            self.initialized = True
            self.get_logger().info('  CAN socket 初始化成功 (直接模式)')
        else:
            self.get_logger().error('  CAN socket 初始化失败!')

    # ═══════════════════════════════════════════════════
    # 命令回调
    # ═══════════════════════════════════════════════════

    def cmd_callback(self, msg: Twist):
        self.last_cmd_time = time.time()

        dive_flag = msg.linear.y
        mv = apply_deadzone(msg.linear.x)
        yw = apply_deadzone(msg.angular.z)
        # angular.y 现在用于定航向目标(度), 不再作为 pitch 输入
        pitch_norm = 0.0

        # ── v7.3: 定航向 (angular.x > 0.1 = 开启, angular.y = 目标航向度) ──
        # 修复: 首次激活时捕获 INS yaw, 但 joy_controller 可能因 yaw_captured=False 发送 0.0
        # 导致目标被覆盖。v7.3 在首次激活后忽略 joy_controller 发来的 0.0
        yaw_hold_new = msg.angular.x > 0.1
        if yaw_hold_new:
            if not self.yaw_hold_active:
                self.yaw_hold_active = True
                self.yaw_err_i = 0.0
                self.yaw_pid_out = 0.0
                # 首次激活: 强制用 INS 当前 yaw 作为目标
                if self.ins_att_valid:
                    self.yaw_hold_target = self.ins_yaw
                    self.get_logger().info(
                        '  定航向已开启 (INS捕获: {:.1f}deg)'.format(self.yaw_hold_target))
                else:
                    self.yaw_hold_target = msg.angular.y
                    self.get_logger().warn(
                        '  定航向已开启 (无INS数据, 使用手柄值: {:.1f}deg)'.format(self.yaw_hold_target))
                self._yaw_first_msg = True
            else:
                # 已活跃
                if self._yaw_first_msg:
                    # 首次激活后的第一条消息: 如果 joy_controller 发送 0.0(未捕获), 保持 INS 值
                    if self.ins_att_valid and abs(msg.angular.y) < 0.5:
                        self.get_logger().info(
                            '  定航向: 忽略 joy_controller 的 0.0, 保持目标={:.1f}deg'.format(self.yaw_hold_target))
                    else:
                        self.yaw_hold_target = msg.angular.y
                    self._yaw_first_msg = False
                else:
                    self.yaw_hold_target = msg.angular.y
            self.yaw_captured = True
        elif not yaw_hold_new and self.yaw_hold_active:
            self.yaw_hold_active = False
            self.yaw_captured = False
            self._yaw_first_msg = False
            self.yaw_err_i = 0.0
            self.yaw_pid_out = 0.0
            self.get_logger().info('  定航向已关闭')

        # Debug: print all cmd_vel messages
        self.get_logger().error('CMD: dive={:.2f} tgt={:.2f} mv={:.2f} yh={} yh_tgt={:.1f}'.format(
            dive_flag, msg.linear.z, mv, self.yaw_hold_active, self.yaw_hold_target))

        # v4.0: dive模式下 linear.z = target_depth(米)
        #       手动模式下 linear.z = up_norm(-1~+1)
        if dive_flag > 0.1:
            target_depth = float(msg.linear.z)
        else:
            up = apply_deadzone(msg.linear.z)

        # 手动模式跳过微小变动 (模式切换时强制处理, 防止PID状态残留)
        prev_was_depth = self.last_dive_flag > 0.1
        if dive_flag <= 0.1 and not prev_was_depth:
            if (abs(mv - self.last_move) < 0.0001 and
                    abs(up - self.last_up) < 0.0001 and
                    abs(yw - self.last_yaw) < 0.0001 and
                    abs(pitch_norm - self.last_pitch) < 0.0001):
                return

        self.last_move   = mv
        self.last_yaw    = yw
        self.last_pitch  = pitch_norm
        self.last_dive_flag = dive_flag

        if dive_flag > 0.1 and target_depth >= 0.01:
            # ── 定深模式 (悬停已开启) ──
            old_target = self.target_depth
            self.target_depth = max(0.0, target_depth)
            if abs(self.target_depth - old_target) > 0.001:
                self.get_logger().info(
                    '  目标深度更新: {:.2f}m → {:.2f}m'.format(old_target, self.target_depth))

            # 航向捕获
            if not self.yaw_captured and self.ins_att_valid:
                self.yaw_target = self.ins_yaw
                self.yaw_captured = True
                self.yaw_err_i = 0.0
                self.yaw_pid_out = 0.0
                self.get_logger().info(
                    '  Yaw PID 捕获航向: {:.1f}deg'.format(self.yaw_target))

            # PID 和电机值在 heartbeat_tick 中计算并发送
            # 保持 last_motors 不变（由 heartbeat_tick 更新），让日志显示真实值
            self.last_up = 0.0  # 不再用 up_norm
        else:
            # ── 手动模式 (包括定深档但悬停未开启) ──
            if dive_flag > 0.1:
                # 定深档但悬停未开启: 清空PID状态, 仍允许手动操控
                self.target_depth = 0.0
                self.yaw_captured = False
                self.yaw_err_i = 0.0
                self.yaw_pid_out = 0.0
                self.depth_err_i = 0.0
                self.depth_pid_out = 0.0
            self.last_up = up
            self.yaw_captured = False
            self.yaw_err_i = 0.0
            self.yaw_pid_out = 0.0
            self.depth_err_i = 0.0
            self.depth_pid_out = 0.0
            self.roll_err_i = 0.0
            self.roll_pid_out = 0.0

            # v7.0: 手动模式电机控制由 heartbeat_tick 统一处理
            # (与定深模式一致, 避免 cmd_callback 和 heartbeat_tick 双重发送冲突)

        # 日志输出
        self._log_cmd(mv, yw, dive_flag)

    def _log_cmd(self, mv, yw, dive_flag):
        """输出当前状态日志"""
        dive_info = ' [定深档]' if dive_flag > 0.1 else ''
        ids_str = '  '.join('ID{}={:+5d}'.format(k, v)
                            for k, v in sorted(self.last_motors.items()) if k != 7)
        ids_str += '  ID7={:+4d}'.format(self.last_motors[7])

        extra = ''
        if dive_flag > 0.1:
            d_str = '{:.2f}'.format(self.current_depth) if self.depth_valid else '--.--'
            t_str = '{:.2f}'.format(self.target_depth)
            err = self.target_depth - self.current_depth if self.depth_valid else 0.0
            extra = ' [深={}/目标={} err={:+.2f}m PID={:+.2f}]'.format(
                d_str, t_str, err, self.depth_pid_out)
            if self.ins_att_valid:
                extra += ' [yawPID={:+.2f} ins={:.1f}deg target={:.1f}deg]'.format(
                    self.yaw_pid_out, self.ins_yaw, self.yaw_target)
                extra += ' [roll={:.1f}° pid={:+.2f} pitch={:.1f}° K_couple]'.format(
                    self.ins_roll, self.roll_pid_out, self.ins_pitch)

        self.get_logger().info(
            'CMD: move={:+5.2f} yaw={:+5.2f}{}  |  {}{}'.format(
                mv, yw, dive_info, ids_str, extra))

    # ═══════════════════════════════════════════════════
    # v4.0: 传感器管道读取
    # ═══════════════════════════════════════════════════

    def _read_sensor_pipe(self):
        """从文件读取 INS + 深度数据 (替代管道)"""
        import os
        sensor_file = "/tmp/sensor_data.json"
        
        # 检查文件是否存在且有内容
        if not os.path.exists(sensor_file):
            if not hasattr(self, '_file_missing_logged'):
                self.get_logger().warn('SENSOR_FILE: {} not found'.format(sensor_file))
                self._file_missing_logged = True
            return
        
        try:
            # 读取文件 (sensor_bridge使用原子操作写入, 所以是安全的)
            with open(sensor_file, 'r') as f:
                line = f.readline().strip()
            if not line:
                return
            
            data = json.loads(line)
            now = time.time()
            
            # INS 姿态
            if 'yaw' in data:
                self.ins_yaw = float(data['yaw'])
                self.ins_pitch = float(data.get('pitch', 0.0))
                self.ins_roll = float(data.get('roll', 0.0))
                self.ins_att_valid = True
                self.last_att_time = now

            # v8.1: INS 加速度 + 角速度
            if 'ax' in data:
                self.ins_ax = float(data['ax'])
                self.ins_ay = float(data['ay'])
                self.ins_az = float(data['az'])
            if 'wx' in data:
                self.ins_wx = float(data['wx'])
                self.ins_wy = float(data['wy'])
                self.ins_wz = float(data['wz'])
            # v8.3: INS 速度 (ve/vn/vd)
            if 've' in data:
                self.ins_ve = float(data['ve'])
                self.ins_vn = float(data['vn'])
                self.ins_vd = float(data['vd'])
            
            # 深度
            if 'depth' in data:
                raw = float(data['depth'])
                if self.depth_valid:
                    self.filtered_depth = 0.5 * raw + 0.5 * self.filtered_depth
                else:
                    self.filtered_depth = raw
                self.current_depth = self.filtered_depth
                self.depth_valid = True
                self.last_depth_time = now
            
            if not self._sensor_first and self.ins_att_valid and self.depth_valid:
                self._sensor_first = True
                self.get_logger().info(
                    '  传感器首帧: yaw={:.2f}deg pitch={:.2f}deg roll={:.2f}deg depth={:.3f}m'.format(
                        self.ins_yaw, self.ins_pitch, self.ins_roll, self.current_depth))
        except Exception as e:
            if not hasattr(self, '_file_error_logged'):
                self.get_logger().error('SENSOR_FILE ERROR: {}'.format(e))
                self._file_error_logged = True

    # ═══════════════════════════════════════════════════
    # v4.0: PID 计算 (在 heartbeat_tick 中调用)
    # ═══════════════════════════════════════════════════

    def _compute_depth_pid(self):
        """深度 PID: 输出归一化 fz in [-1,+1] (+ = 下潜)"""
        if not self.depth_valid:
            self.depth_pid_out = 0.0
            self.get_logger().error('  DEPTH_PID: depth_valid=False, output=0')
            return
        if (time.time() - self.last_depth_time) > DEPTH_TIMEOUT_SENSOR:
            self.depth_pid_out = 0.0
            self.get_logger().error('  DEPTH_PID: depth timeout, output=0')
            return

        err = self.target_depth - self.current_depth
        dt = 0.1  # 10Hz

        # P
        p_error = 0.0 if abs(err) < DEPTH_DEADBAND else err
        p = DEPTH_KP * p_error

        # Debug log (every 10 ticks = 1 second)
        if hasattr(self, '_depth_debug_cnt'):
            self._depth_debug_cnt += 1
        else:
            self._depth_debug_cnt = 0
        if self._depth_debug_cnt % 10 == 0:
            self.get_logger().error(
                '  DEPTH_PID_DEBUG: valid={} err={:.3f} p_error={:.3f} p={:.3f} I={:.3f} out={:.3f}'.format(
                    self.depth_valid, err, p_error, p, self.depth_err_i, self.depth_pid_out))

        # I
        if abs(err) > DEPTH_I_GATE:
            pass
        elif (err * self.depth_err_i) < -0.01:
            self.depth_err_i = 0.0
        elif abs(err) < DEPTH_DEADBAND:
            self.depth_err_i *= DEPTH_I_DECAY
        elif abs(self.depth_pid_out) < 0.95 or (err * self.depth_err_i) < 0:
            self.depth_err_i = _clamp(
                self.depth_err_i + DEPTH_KI * err * dt, -DEPTH_I_MAX, DEPTH_I_MAX)

        self.depth_pid_out = _clamp(p + self.depth_err_i, -1.0, 1.0)

    # ═══════════════════════════════════════════════════
    # v8.0: 深度前馈补偿 (数据驱动, 辅助PID稳态)
    # ═══════════════════════════════════════════════════
    def _compute_depth_ff(self):
        """
        计算稳态平衡推力的前馈估计。
        模型: fz_ff = bias + w_d*depth + w_p*sin(pitch) + w_r*sin(roll)
        乘以 FF_GAIN 缩放, clamp 到安全范围。
        当 FF_GAIN=0 时此函数无效果。
        """
        if FF_GAIN <= 0.0:
            self.fz_ff = 0.0
            return
        import math
        pitch_rad = math.radians(self.ins_pitch if self.ins_att_valid else 0.0)
        roll_rad  = math.radians(self.ins_roll  if self.ins_att_valid else 0.0)
        raw = (FF_BIAS
               + FF_DEPTH_COEFF * self.target_depth
               + FF_SIN_PITCH_COEFF * math.sin(pitch_rad)
               + FF_SIN_ROLL_COEFF  * math.sin(roll_rad))
        self.fz_ff = _clamp(raw * FF_GAIN, -0.5, 0.5)  # 前馈不超过 ±50% 推力

    def _compute_roll_pid(self):
        """Roll PID: 维持横滚水平, 输出 mx in [-1,+1] (+ = 右滚)"""
        now = time.time()
        att_valid = (self.ins_att_valid and
                     (now - self.last_att_time) < ATT_TIMEOUT)
        if not att_valid:
            self.roll_pid_out = 0.0
            return

        dt = 0.1
        roll_err = 0.0 - self.ins_roll
        if abs(roll_err) < ROLL_DBAND:
            r_p = 0.0
        else:
            r_p = ROLL_KP * roll_err

        if abs(roll_err) > ROLL_I_GATE:
            pass
        elif (roll_err * self.roll_err_i) < -0.01:
            self.roll_err_i = 0.0
        elif abs(roll_err) < ROLL_DBAND:
            self.roll_err_i *= ROLL_I_DECAY
        else:
            self.roll_err_i = _clamp(
                self.roll_err_i + ROLL_KI * roll_err * dt, -ROLL_I_MAX, ROLL_I_MAX)

        self.roll_pid_out = _clamp(r_p + self.roll_err_i, -1.0, 1.0)

    def _compute_pitch_pid(self):
        """Pitch PID: 维持俯仰水平, 输出 my in [-1,+1] (+ = 抬头)"""
        now = time.time()
        att_valid = (self.ins_att_valid and
                     (now - self.last_att_time) < ATT_TIMEOUT)
        if not att_valid:
            self.pitch_pid_out = 0.0
            return

        dt = 0.1
        pitch_err = 0.0 - self.ins_pitch
        if abs(pitch_err) < PITCH_DBAND:
            p_p = 0.0
        else:
            p_p = PITCH_KP * pitch_err

        if abs(pitch_err) > PITCH_I_GATE:
            pass
        elif (pitch_err * self.pitch_err_i) < -0.01:
            self.pitch_err_i = 0.0
        elif abs(pitch_err) < PITCH_DBAND:
            self.pitch_err_i *= PITCH_I_DECAY
        else:
            self.pitch_err_i = _clamp(
                self.pitch_err_i + PITCH_KI * pitch_err * dt, -PITCH_I_MAX, PITCH_I_MAX)

        self.pitch_pid_out = _clamp(p_p + self.pitch_err_i, -1.0, 1.0)

    def _compute_yaw_pid(self):
        """Yaw PD: 输出 mz in [-1,+1] (+ = 右转)
        
        v8.4: 改为可调PD, 默认 KI=0. 
        mz = KP*err + KD*(err-last_err)/dt
        当 KI>0 时启用积分项 (与原PID一致).
        """
        now = time.time()
        valid = (self.ins_att_valid and
                 (now - self.last_att_time) < YAW_ATT_TIMEOUT)
        if not valid:
            self.yaw_pid_out = 0.0
            return

        yaw_err = _angle_diff(self.yaw_target, self.ins_yaw)
        dt = max(0.01, now - self.last_yaw_pid_time)
        self.last_yaw_pid_time = now

        # ── Proportional ──
        if abs(yaw_err) < YAW_DEADBAND:
            p_out = 0.0
        else:
            p_out = YAW_KP * yaw_err

        # ── Derivative (角速度阻尼, 防震荡) ──
        if self._last_yaw_err is not None:
            err_dot = (yaw_err - self._last_yaw_err) / dt
            d_out = YAW_KD * err_dot
        else:
            d_out = 0.0
        self._last_yaw_err = yaw_err

        # ── Integral (KI>0 时启用, 对抗恒定偏转) ──
        i_out = 0.0
        if YAW_KI > 0:
            if abs(yaw_err) > YAW_I_GATE:
                pass
            elif (yaw_err * self.yaw_err_i) < -0.01:
                self.yaw_err_i = 0.0
            elif abs(yaw_err) < YAW_DEADBAND:
                self.yaw_err_i *= YAW_I_DECAY
            else:
                self.yaw_err_i = _clamp(
                    self.yaw_err_i + YAW_KI * yaw_err * dt,
                    -YAW_I_MAX, YAW_I_MAX)
            i_out = self.yaw_err_i

        self.yaw_pid_out = _clamp(p_out + d_out + i_out, -1.0, 1.0)

    # ═══════════════════════════════════════════════════
    # 心跳 (10Hz)
    # ═══════════════════════════════════════════════════

    def heartbeat_tick(self):
        """10Hz 心跳: 读取传感器, 运行4PID, 推力分配, 发送CAN (v6.0)"""
        # 调试: 每次都输出，确认心跳在运行
        if not hasattr(self, '_hb_first_logged'):
            self.get_logger().error('HEARTBEAT_TICK: FIRST CALL!')
            self._hb_first_logged = True
        
        if not self.initialized:
            return

        self._hb_log_count += 1

        # 读取传感器数据 (subprocess管道)
        self._read_sensor_pipe()

        # Debug: print dive mode status every 10 seconds
        if self._hb_log_count % 100 == 0:
            self.get_logger().error(
                '  HB_DEBUG: dive_flag={:.2f} target={:.2f} depth={:.2f} valid={} att_valid={}'.format(
                    self.last_dive_flag, self.target_depth, self.current_depth,
                    self.depth_valid, self.ins_att_valid))

        # ── v6.0: 定深模式 → 推力分配矩阵 + 6-DOF PID ──
        if self.last_dive_flag > 0.1:
            # 安全: 深度传感器无效时禁用定深模式
            if not self.depth_valid:
                if any(v != 0 for v in self.last_g):
                    self.get_logger().warn('定深模式: 深度传感器无效, 停止电机')
                    g = [0] * 8
                    self.last_g = g
                    self.last_motors = {mid: 0 for mid in MOTOR_IDS}
                    send_motor_rpm(g)
                return

            # 安全: 目标深度为0或无效时禁用 (防止用户未开启悬停时target_depth=0)
            if self.target_depth <= 0.05:
                if any(v != 0 for v in self.last_g):
                    self.get_logger().warn('定深模式: 目标深度无效({}), 停止电机'.format(self.target_depth))
                    g = [0] * 8
                    self.last_g = g
                    self.last_motors = {mid: 0 for mid in MOTOR_IDS}
                    send_motor_rpm(g)
                return

            # 安全: 姿态传感器无效时限制垂直推力 (防翻覆)
            att_ok = self.ins_att_valid and (time.time() - self.last_att_time) < ATT_TIMEOUT
        else:
            # 手动模式: 也检查姿态有效性
            att_ok = self.ins_att_valid and (time.time() - self.last_att_time) < ATT_TIMEOUT

        # 定深模式: 运行所有PID; 手动模式: 只用原始手柄输入, 不运行PID
        # (定航向活跃时Yaw PID由下方yaw_hold块单独调用, 避免重复计算)
        if self.last_dive_flag > 0.1:
            self._compute_depth_pid()
            self._compute_depth_ff()   # v8.0: 前馈补偿 (FF_GAIN=0时无效果)
            self._compute_roll_pid()
            self._compute_pitch_pid()
            if not self.yaw_hold_active:
                self._compute_yaw_pid()

        # ── 构建 6-DOF 力/力矩向量 tau ──
        # Fx: 手动前进/后退 (深度保持时仍可平移)
        # v8.4: 翻转电机方向, 使 axis[3]=+1→电机后退(与监控显示一致)
        fx = -self.last_move

        # Fy: 侧移 (当前无外部需求)
        fy = 0.0

        # Fz: 定深模式=PID输出, 手动模式=手柄输入 (+ = 下潜)
        # 姿态无效时限制垂直推力, 防止无姿态反馈时翻覆
        if self.last_dive_flag > 0.1:
            fz = self.depth_pid_out + self.fz_ff  # v8.0: PID + 前馈补偿
            if not att_ok:
                fz = _clamp(fz, -0.3, 0.3)
        else:
            fz = self.last_up

        # ── 计算当前深度控制阶段 (用于 mz/ID7 阶段判断) ──
        self._fixed_stage = False
        if self.last_dive_flag > 0.1 and self.depth_valid:
            self._fixed_stage = abs(self.target_depth - self.current_depth) > DEPTH_FIXED_THRESHOLD

        # Mx: Roll PID (+ = 右滚修正) - 临时禁用
        mx = 0.0  # self.roll_pid_out

        # My: Pitch PID (+ = 抬头修正) - 临时禁用
        my = 0.0  # self.pitch_pid_out

        # Mz: 定航向优先 → 定深PID阶段 → 手动偏航
        # v7.7: YAW_DIRECTION 只修正 PID 输出, 不翻转子动手动 steering 方向
        # v8.6: 手动/自动增益分离 — 手动转向增强(0.60), 定深偏置保持(0.10)
        if self.last_dive_flag > 0.1:
            manual_yaw = self.last_yaw * YAW_MANUAL_TRIM_AUTO
        else:
            manual_yaw = self.last_yaw * YAW_MANUAL_TRIM_MANUAL
        if self.yaw_hold_active:
            self.yaw_target = self.yaw_hold_target
            if self.ins_att_valid:
                yaw_err_hold = _angle_diff(self.yaw_target, self.ins_yaw)
            else:
                yaw_err_hold = 0.0
            if self.ins_att_valid and abs(yaw_err_hold) > YAW_HOLD_THRESHOLD:
                # 阶段1: 大转速回正 (mz_id7=±1.0 → ID7=1400 RPM)
                self.yaw_pid_out = 1.0 if yaw_err_hold > 0 else -1.0
                self.yaw_err_i = 0.0  # 防积分饱和
                mz_id7 = self.yaw_pid_out
            else:
                self._compute_yaw_pid()
                mz_id7 = _clamp(self.yaw_pid_out, -1.0, 1.0)
            # 定航向PID方向修正 (YAW_DIRECTION=-1)
            mz_id7 *= YAW_DIRECTION
        elif self.last_dive_flag > 0.1:
            if self._fixed_stage:
                mz_id7 = 0.0  # 固定阶段: 完全不控制ID7
            else:
                self._compute_yaw_pid()
                # PID需要YAW_DIRECTION修正, manual_yaw不需要
                mz_id7 = _clamp(self.yaw_pid_out * YAW_DIRECTION + manual_yaw, -1.0, 1.0)
        else:
            mz_id7 = manual_yaw
            # 手动模式: 不应用 YAW_DIRECTION, 保持手柄原始方向

        # v7.3: 尾推Yaw辅助 — 通过 B+ 自动分配差速, 按尾推Yaw比例缩放
        # v8.6: 手动模式用增强比例0.6, 定深/定航向保持0.5 (不影响既有自动逻辑)
        if self.yaw_hold_active or self.last_dive_flag > 0.1:
            mz_tail = mz_id7 * TAIL_YAW_RATIO_AUTO
        else:
            mz_tail = mz_id7 * TAIL_YAW_RATIO_MANUAL

        # ── 推力分配 (伪逆矩阵 B+ 解算, 尾推含缩放Yaw, ID7独立) ──
        # Fz 不通过分配器 (避免尾推抵消垂推), 直接控制 ID5/ID6
        alloc = allocate(fx, fy, 0.0, mx, my, mz_tail)

        # ── 转换为 RPM ──
        g = [0] * 8

        # ═══════════════════════════════════════════════════════
        # v7.0: 两阶段深度控制
        #   阶段1 (|误差| > DEPTH_FIXED_THRESHOLD): 固定推力, 快速下潜/上浮
        #   阶段2 (|误差| ≤ DEPTH_FIXED_THRESHOLD): PID 精细控制, 稳定悬浮
        #
        # 尾推倾角 22.5deg, 水平力相互抵消, 垂直分量叠加:
        #   下潜: ID0/ID3后退(-) ID1/ID2前进(+) ID5/ID6向下(+)
        #   上浮: ID0/ID3前进(+) ID1/ID2后退(-) ID5/ID6向上(-)
        # ═══════════════════════════════════════════════════════

        if self.last_dive_flag > 0.1 and self.depth_valid:
            depth_error = self.target_depth - self.current_depth  # + = 需要下潜

            if abs(depth_error) > DEPTH_FIXED_THRESHOLD:
                # ── 阶段1: 固定推力 ──
                if depth_error > 0:
                    fz_tail = DIVE_TAIL_NORM   # +0.334 → 1250 RPM (v8.5)
                    fz_vert = DIVE_VERT_NORM   # +0.845 → 1480 RPM (v8.5)
                else:
                    fz_tail = -SURF_TAIL_NORM  # -0.178 → -1180 RPM
                    fz_vert = -SURF_VERT_NORM  # -0.667 → -1400 RPM
            else:
                # ── 阶段2: PID 精细控制 ──
                fz = self.depth_pid_out + self.fz_ff  # v8.0: PID + 前馈
                fz_tail = fz * FZ_GAIN_TAIL
                fz_vert = fz * FZ_GAIN_VERT
                # 下潜方向保底: PID输出小时仍需克服浮力
                if fz > 0.01:
                    fz_tail = max(fz_tail, TAIL_DIVE_MIN)  # v8.5: 保底1200RPM
                    fz_vert = max(fz_vert, VERT_DIVE_MIN)  # v8.5: 保底1350RPM
        else:
            # 手动模式或无深度数据: 方向不对称垂直增益 (v8.6)
            # 下潜 → 增强至与定深阶段1一致 (尾推1250/垂推1480 RPM)
            # 上浮 → 保持v8.5增益不变 (1180/1400 RPM, 机器人变轻上浮更容易)
            if fz >= 0:
                fz_tail = fz * MANUAL_DIVE_FZ_TAIL
                fz_vert = fz * MANUAL_DIVE_FZ_VERT
            else:
                fz_tail = fz * FZ_GAIN_TAIL
                fz_vert = fz * FZ_GAIN_VERT

        # 下潜/上浮时尾推不参与Yaw-pitch耦合, 保持4电机转速一致
        if abs(fz_tail) > 0.001:
            fx_only = allocate(fx, fy, 0.0, 0, 0, mz_tail)  # v7.3: 垂直运动时仍含Yaw
            a0, a1, a2, a3 = fx_only.get(0,0), fx_only.get(1,0), fx_only.get(2,0), fx_only.get(3,0)
        else:
            a0, a1, a2, a3 = alloc.get(0,0), alloc.get(1,0), alloc.get(2,0), alloc.get(3,0)

        g[0] = norm_to_rpm(a0 - fz_tail, TAIL_RPM_MIN, TAIL_RPM_MAX)
        g[1] = norm_to_rpm(a1 + fz_tail, TAIL_RPM_MIN, TAIL_RPM_MAX)
        g[2] = norm_to_rpm(a2 + fz_tail, TAIL_RPM_MIN, TAIL_RPM_MAX)
        g[3] = norm_to_rpm(a3 - fz_tail, TAIL_RPM_MIN, TAIL_RPM_MAX)

        g[5] = norm_to_rpm(fz_vert, VERT_RPM_MIN, VERT_RPM_MAX)
        g[6] = norm_to_rpm(fz_vert, VERT_RPM_MIN, VERT_RPM_MAX)

        # ── 尾推绝对值不超过垂推 (防止pitch摆动, v7.0: 支持正负双向) ──
        if abs(fz_tail) > 0.01:
            vert_abs = max(abs(g[5]), abs(g[6]))
            for i in range(4):
                if abs(g[i]) > vert_abs:
                    g[i] = vert_abs if g[i] > 0 else -vert_abs

        # 横推 ID7: 纯Yaw主控 (v7.3: ID7 100% mz, 尾推 50% 辅助)
        g[7] = norm_to_rpm(mz_id7, YAW_RPM_MIN, YAW_RPM_MAX)
        # 固定深度阶段ID7=0，但定航向活跃时仍然保持控制
        if self._fixed_stage and not self.yaw_hold_active:
            g[7] = 0

        # ── Pitch 安全: 超阈值线性降推, 防翻覆 ──
        if self.ins_att_valid:
            pitch_abs = abs(self.ins_pitch)
            if pitch_abs > PITCH_SAFE:
                pitch_scale = max(0.0, 1.0 - (pitch_abs - PITCH_SAFE) / (PITCH_KILL - PITCH_SAFE))
                for i in range(8):
                    g[i] = int(g[i] * pitch_scale)

        self.last_g = g
        self.last_motors = {mid: g[mid] for mid in MOTOR_IDS}

        send_motor_rpm(g)

        # ── 每秒日志 ──
        if self._hb_log_count % 10 == 0:
            m = self.last_motors
            d_str = '{:.3f}'.format(self.current_depth) if self.depth_valid else '---'
            safe_tag = ' SAFE!' if (self.ins_att_valid and abs(self.ins_pitch) > PITCH_SAFE) else ''
            att_tag = ' NO-ATT!' if not att_ok else ''
            # v7.1: 显示控制阶段 + 定航向(大转速/微调)
            if self.yaw_hold_active and self.ins_att_valid:
                yaw_err_h = _angle_diff(self.yaw_hold_target, self.ins_yaw)
                if abs(yaw_err_h) > YAW_HOLD_THRESHOLD:
                    yh_tag = ' |YH_B|{:+.0f}'.format(yaw_err_h)
                else:
                    yh_tag = ' |YH_F|{:+.0f}'.format(yaw_err_h)
            elif self.yaw_hold_active:
                yh_tag = ' |YH_N'
            else:
                yh_tag = ''
            if self.last_dive_flag > 0.1 and self.depth_valid:
                err = self.target_depth - self.current_depth
                if abs(err) > DEPTH_FIXED_THRESHOLD:
                    mode_tag = 'FIXED' if err > 0 else 'FIXED_UP'
                else:
                    mode_tag = 'PID'
            else:
                mode_tag = 'MANUAL'
                err = 0.0
            self.get_logger().info(
                'v8.6 {}{} | 深={}/tar={:.2f} err={:+.3f}m pit={:.1f}° rol={:.1f}° yaw={:.1f}° | '
                'fz={:+.3f} mx={:+.3f} my={:+.3f} mz={:+.3f} | '
                'T={:+d} {:+d} {:+d} {:+d} V={:+d} {:+d} Y={:+d}{}{}'.format(
                    mode_tag, yh_tag, d_str, self.target_depth, err,
                    self.ins_pitch if self.ins_att_valid else 0,
                    self.ins_roll if self.ins_att_valid else 0,
                    self.ins_yaw if self.ins_att_valid else 0,
                    fz, mx, my, mz_id7,
                    m[0], m[1], m[2], m[3], m[5], m[6], m[7],
                    safe_tag, att_tag))

        elif any(v != 0 for v in self.last_g):
            # 手动模式: 维持最后一次指令
            send_motor_rpm(self.last_g)

    # ═══════════════════════════════════════════════════
    # 超时检查
    # ═══════════════════════════════════════════════════

    def timeout_check(self):
        if time.time() - self.last_cmd_time > TIMEOUT_SEC:
            if any(v != 0 for v in self.last_g):
                self.get_logger().warn('命令超时，自动停止所有电机')
                self.last_move = 0.0
                self.last_up   = 0.0
                self.last_yaw  = 0.0
                self.last_roll = 0.0
                self.last_pitch = 0.0
                self.last_dive_flag = 0.0
                self.last_motors = {0: 0, 1: 0, 2: 0, 3: 0, 5: 0, 6: 0, 7: 0}
                self.last_g = [0] * 8
                self.depth_err_i = 0.0
                self.depth_pid_out = 0.0
                self.roll_err_i = 0.0
                self.roll_pid_out = 0.0
                self.pitch_err_i = 0.0
                self.pitch_pid_out = 0.0
                self.yaw_err_i = 0.0
                self.yaw_pid_out = 0.0
                self.yaw_captured = False
                self.yaw_hold_active = False
                send_motor_rpm(self.last_g)

    # ═══════════════════════════════════════════════════
    # 诊断
    # ═══════════════════════════════════════════════════

    def diagnostic_tick(self):
        """5s 诊断输出"""
        d_str = '{:.3f}'.format(self.current_depth) if self.depth_valid else '---'
        d_tar = '{:.2f}'.format(self.target_depth)
        att_age = time.time() - self.last_att_time if self.last_att_time > 0 else -1
        dep_age = time.time() - self.last_depth_time if self.last_depth_time > 0 else -1
        alive = (self.sensor_proc.poll() is None
                 if hasattr(self, 'sensor_proc') and self.sensor_proc else False)
        self.get_logger().info(
            'DIAG v8.5: depth={}m target={}m | yaw={:.1f}deg pitch={:.1f}deg roll={:.1f}deg | '
            'att_age={:.1f}s dep_age={:.1f}s | '
            'PID: fz={:+.3f} mx={:+.3f} my={:+.3f} mz_id7={:+.3f} | '
            'yaw_hold={} yaw_target={:.1f} yaw_captured={} proc_alive={}'.format(
                d_str, d_tar, self.ins_yaw, self.ins_pitch, self.ins_roll,
                att_age, dep_age,
                self.depth_pid_out, self.roll_pid_out, self.pitch_pid_out, self.yaw_pid_out,
                self.yaw_hold_active,
                self.yaw_hold_target if self.yaw_hold_active else self.yaw_target,
                self.yaw_captured, alive))

    # ═══════════════════════════════════════════════════
    # 状态发布
    # ═══════════════════════════════════════════════════

    def publish_state(self):
        now = time.time()
        data = json.dumps({
            'move_norm': round(self.last_move, 3),
            'up_norm':   round(self.last_up, 3),
            'yaw_norm':  round(self.last_yaw, 3),
            'roll_norm': round(self.last_roll, 3),
            'pitch_norm': round(self.last_pitch, 3),
            'dive_flag': round(self.last_dive_flag, 3),
            'target_depth': round(self.target_depth, 2),
            'current_depth': round(self.current_depth, 3),
            'depth_valid': self.depth_valid,
            'depth_pid_out': round(self.depth_pid_out, 3),
            'depth_err_i': round(self.depth_err_i, 4),
            'fz_ff': round(self.fz_ff, 4),   # v8.0: 前馈补偿
            'roll_pid_out': round(self.roll_pid_out, 3),
            'pitch_pid_out': round(self.pitch_pid_out, 3),
            'yaw_pid_out': round(self.yaw_pid_out, 3),
            'yaw_target': round(self.yaw_hold_target if self.yaw_hold_active else self.yaw_target, 1),
            'yaw_captured': self.yaw_captured or self.yaw_hold_active,
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
            'motors': self.last_motors,
            'initialized': self.initialized,
            'ts': now
        })
        self.state_pub.publish(String(data=data))
        # 调试日志（每5秒一次）
        if self._hb_log_count % 50 == 0:
            self.get_logger().info('publish_state: motors={} depth={:.3f}'.format(
                self.last_motors, self.current_depth))

    def shutdown(self):
        self.get_logger().info('关闭：停止所有电机...')
        send_motor_rpm([0] * 8)
        can_close()
        if hasattr(self, 'sensor_proc') and self.sensor_proc:
            try:
                self.sensor_proc.terminate()
                self.sensor_proc.wait(timeout=2)
            except Exception:
                pass


def main():
    rclpy.init()
    node = MotorController()

    def on_signal(sig, frame):
        try:
            send_motor_rpm([0] * 8)
            can_close()
        except Exception:
            pass
        os._exit(0)

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGHUP, on_signal)

    # ═══════════════════════════════════════════════════
    # v4.0: Subprocess 桥接 - 同时订阅 INS 姿态 + 深度
    # ═══════════════════════════════════════════════════
    import subprocess as _sp
    sensor_bridge_code = r'''
import os, sys, time, json
os.environ['ROS_DOMAIN_ID'] = '42'
import rclpy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

rclpy.init()
n = rclpy.create_node("sensor_bridge")

state = {"yaw": 0.0, "pitch": 0.0, "roll": 0.0, "depth": 0.0,
         "ax": 0.0, "ay": 0.0, "az": 0.0,
         "wx": 0.0, "wy": 0.0, "wz": 0.0,
         "ve": 0.0, "vn": 0.0, "vd": 0.0}
ins_count = 0
dep_count = 0
acc_valid = False
gyro_valid = False
vel_valid = False

def _write_state():
    out = {}
    if ins_count > 0:
        out["yaw"] = state["yaw"]
        out["pitch"] = state["pitch"]
        out["roll"] = state["roll"]
    if dep_count > 0:
        out["depth"] = state["depth"]
    if acc_valid:
        out["ax"] = state["ax"]
        out["ay"] = state["ay"]
        out["az"] = state["az"]
    if gyro_valid:
        out["wx"] = state["wx"]
        out["wy"] = state["wy"]
        out["wz"] = state["wz"]
    if vel_valid:
        out["ve"] = state["ve"]
        out["vn"] = state["vn"]
        out["vd"] = state["vd"]
    tmp_file = "/tmp/sensor_data.tmp"
    with open(tmp_file, 'w') as f:
        f.write(json.dumps(out) + '\n')
    os.rename(tmp_file, "/tmp/sensor_data.json")

def att_cb(msg):
    global ins_count
    state["yaw"] = float(msg.z)
    state["pitch"] = float(msg.x)
    state["roll"] = float(msg.y)
    ins_count += 1
    # INS 100Hz → 每10帧输出一次减少管道压力
    if ins_count % 10 == 0:
        _write_state()

def dep_cb(msg):
    global dep_count
    state["depth"] = float(msg.data)
    dep_count += 1
    # 深度 ~2Hz → 每次收到都输出
    _write_state()

def acc_cb(msg):
    global acc_valid
    state["ax"] = float(msg.x)
    state["ay"] = float(msg.y)
    state["az"] = float(msg.z)
    acc_valid = True

def gyro_cb(msg):
    global gyro_valid
    state["wx"] = float(msg.x)
    state["wy"] = float(msg.y)
    state["wz"] = float(msg.z)
    gyro_valid = True

def vel_cb(msg):
    global vel_valid
    state["ve"] = float(msg.x)
    state["vn"] = float(msg.y)
    state["vd"] = float(msg.z)
    vel_valid = True

n.create_subscription(Vector3, "/ins/attitude", att_cb, 10)
n.create_subscription(Float32, "/rov/depth", dep_cb, 10)
n.create_subscription(Vector3, "/ins/acceleration", acc_cb, 10)
n.create_subscription(Vector3, "/ins/angular_rate", gyro_cb, 10)
n.create_subscription(Vector3, "/ins/velocity", vel_cb, 10)
sys.stderr.write("SENSOR_BRIDGE_READY\n"); sys.stderr.flush()

# 主循环 - 添加详细异常处理
import traceback
try:
    while rclpy.ok():
        try:
            rclpy.spin_once(n, timeout_sec=0.05)
        except Exception as e:
            # 记录错误但继续运行
            with open('/tmp/sensor_bridge_error.log', 'a') as f:
                f.write('{} spin_once ERROR: {}\n'.format(time.time(), e))
                f.write(traceback.format_exc() + '\n')
            time.sleep(0.1)
except KeyboardInterrupt:
    pass
except Exception as e:
    # 致命错误 - 记录并退出
    with open('/tmp/sensor_bridge_error.log', 'a') as f:
        f.write('{} FATAL ERROR: {}\n'.format(time.time(), e))
        f.write(traceback.format_exc() + '\n')
'''
    # 启动 sensor_bridge 子进程 (输出到文件, 不再使用管道)
    sensor_proc = _sp.Popen(
        ['python3', '-u', '-c', sensor_bridge_code],
        stdout=_sp.DEVNULL,  # 不再需要stdout管道
        stderr=open('/tmp/sensor_bridge_stderr.log', 'w'),
        text=True, bufsize=1)
    node.sensor_proc = sensor_proc

    # 等待桥接就绪 (简化: 直接等待0.5秒让子进程启动)
    import time
    time.sleep(0.5)
    if sensor_proc.poll() is not None:
        # 进程已退出
        node.get_logger().error('  sensor_bridge 启动失败, 退出码={}'.format(sensor_proc.returncode))
    else:
        node.get_logger().info('  独立传感器桥接进程已启动 (PID={})'.format(sensor_proc.pid))

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    sensor_restart_count = 0
    try:
        loop_count = 0
        while rclpy.ok():
            loop_count += 1
            try:
                # 直接调用heartbeat_tick (10Hz)
                node.heartbeat_tick()
            except Exception as e:
                import traceback
                node.get_logger().error('HEARTBEAT_TICK EXCEPTION: {}'.format(e))
                node.get_logger().error(traceback.format_exc())
            
            try:
                executor.spin_once(timeout_sec=0.01)
            except Exception as e:
                node.get_logger().error('SPIN_ONCE EXCEPTION: {}'.format(e))
            
            # v8.2: 超时检查 (每10个循环≈1秒检查一次)
            if loop_count % 10 == 0:
                try:
                    node.timeout_check()
                except Exception as e:
                    node.get_logger().error('TIMEOUT_CHECK EXCEPTION: {}'.format(e))
            
            # v8.2: sensor_bridge 健康监控 (每50个循环≈5秒检查一次)
            if loop_count % 50 == 0:
                if sensor_proc.poll() is not None:
                    sensor_restart_count += 1
                    node.get_logger().error(
                        'SENSOR_BRIDGE 已退出(码={}), 第{}次重启...'.format(
                            sensor_proc.returncode, sensor_restart_count))
                    # 重启 sensor_bridge
                    sensor_proc = _sp.Popen(
                        ['python3', '-u', '-c', sensor_bridge_code],
                        stdout=_sp.DEVNULL,
                        stderr=open('/tmp/sensor_bridge_stderr.log', 'w'),
                        text=True, bufsize=1)
                    node.sensor_proc = sensor_proc
                    time.sleep(0.5)
                    if sensor_proc.poll() is None:
                        node.get_logger().info(
                            'SENSOR_BRIDGE 重启成功 (PID={})'.format(sensor_proc.pid))
                    else:
                        node.get_logger().error('SENSOR_BRIDGE 重启失败!')

                # v8.2: CAN 健康检查 — socket 为 None 时主动恢复
                if _can_sock is None and not _can_recovering:
                    node.get_logger().warn('CAN socket 为 None, 主动触发恢复...')
                    t = threading.Thread(target=can_recover, daemon=True)
                    t.start()
            
            time.sleep(0.09)  # 10Hz = 100ms = 0.1s
    except KeyboardInterrupt:
        pass
    finally:
        try:
            send_motor_rpm([0] * 8)
            can_close()
        except Exception:
            pass
        # v8.2: 清理 sensor_bridge 子进程, 防止孤儿进程
        try:
            if sensor_proc.poll() is None:
                sensor_proc.terminate()
                sensor_proc.wait(timeout=2)
        except Exception:
            try:
                sensor_proc.kill()
            except Exception:
                pass
        os._exit(0)


if __name__ == '__main__':
    main()
