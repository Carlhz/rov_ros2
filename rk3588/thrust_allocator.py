#!/usr/bin/env python3
"""
ROV 推力分配矩阵 v1.5

6-DOF 力/力矩 -> 7 电机归一化命令

坐标系: X+ = 前, Y+ = 右, Z+ = 下 (NED)

输入 tau = [Fx, Fy, Fz, Mx, My, Mz]:
  Fx: 前进力     (+ = 前进)
  Fy: 侧向力     (+ = 右移)
  Fz: 垂直力     (+ = 下潜)
  Mx: 横滚力矩   (+ = 右滚)
  My: 俯仰力矩   (+ = 抬头)
  Mz: 偏航力矩   (+ = 右转)

输出 u = {0: u0, 1: u1, 2: u2, 3: u3, 5: u5, 6: u6, 7: u7}
  所有 u_i in [-1, +1], + = 前进/主方向
  CAN 反相 (ID1/3/6) 由 motor_controller 的 build_ctrl 处理

电机布局 (后视图, 从尾部看):
  ID3(左上,反装)  ID0(右上,正装)
  ID2(左下,正装)  ID1(右下,反装)

  正装对角线: ID0 + ID2 (CW = 前进)
  反装对角线: ID1 + ID3 (CW = 后退, CAN反相补偿)

  ID5: 左垂推 (纯垂直, g>0=下潜)
  ID6: 右垂推 (纯垂直, g>0=下潜)
  ID7: 横推   (纯侧向, 前部)

矢量倾角: 22.5 deg (垂直 + 水平各 22.5 deg)
  上倾 -> 前进产生下压 (Z+)
  下倾 -> 前进产生上浮 (Z-)
  右偏 -> 前进产生右移 (Y+)
  左偏 -> 前进产生左移 (Y-)
"""

import math

# 22.5 deg 矢量倾角
_A = 22.5 * math.pi / 180
_CA = math.cos(_A)   # 0.9239
_SA = math.sin(_A)   # 0.3827

# 推力方向系数 (g_i > 0 = 前进/主方向, CAN反相已补偿)
# 格式: (fx, fy, fz)
_THRUST_DIR = {
    0: (_CA * _CA,  _CA * _SA,  _SA),    # 右上: 前, 右, 下
    1: (_CA * _CA,  _CA * _SA, -_SA),    # 右下: 前, 右, 上
    2: (_CA * _CA, -_CA * _SA, -_SA),    # 左下: 前, 左, 上
    3: (_CA * _CA, -_CA * _SA,  _SA),    # 左上: 前, 左, 下
    5: (0.0, 0.0,  1.0),                 # 左垂推: 下 (g>0→下潜)
    6: (0.0, 0.0,  1.0),                 # 右垂推: 下 (g>0→下潜)
    7: (0.0, 1.0, 0.0),                  # 横推: 右
}

# 电机位置 (相对质心, 米)
# 质心: (-15.5, 0, 12) mm
_POS = {
    0: (-0.527,  0.080, -0.062),   # 右上尾推
    1: (-0.527,  0.080,  0.073),   # 右下尾推
    2: (-0.527, -0.080,  0.073),   # 左下尾推
    3: (-0.527, -0.080, -0.062),   # 左上尾推
    5: ( 0.143, -0.170,  0.053),   # 左垂推
    6: ( 0.143,  0.170,  0.053),   # 右垂推
    7: ( 0.278, -0.050,  0.018),   # 横推
}

MOTOR_IDS = [0, 1, 2, 3, 5, 6, 7]

# 预计算伪逆 B+ (7x6)
# B+ = B^T * (B * B^T)^(-1)
# 由几何参数计算得出, B * B+ = I (6x6 单位矩阵)
#
# 分配矩阵 B (6x7) 的物理含义:
#   每列 = 一个电机对 6-DOF 力/力矩的贡献
#   力分量 = 推力方向系数
#   力矩分量 = r x f (位置叉乘力方向)
#
# 伪逆 B+ 的物理含义:
#   给定期望力/力矩, 求最小范数的电机命令
#   7 个电机 > 6 个 DOF -> 冗余驱动, 伪逆给出最优分配

BPLUS = [
    # v1.5: 修正ID5/ID6垂推方向 (g>0=下潜, 基于实测物理行为)
    [+0.285976, +0.208295, +0.179844, +0.781942, +1.257653, -0.698634],  # ID0
    [+0.299810, +0.185590, -0.179844, -0.781942, -1.257653, -0.718219],  # ID1
    [+0.299810, -0.185590, -0.179844, +0.781942, -1.257653, +0.718219],  # ID2
    [+0.285976, -0.208295, +0.179844, -0.781942, +1.257653, +0.698634],  # ID3
    [+0.005294, -0.035425, +0.362353, -2.440002, -0.962566, -0.030557],  # ID5
    [+0.005294, +0.035425, +0.362353, +2.440002, -0.962566, +0.030557],  # ID6
    [+0.000000, +0.721481, +0.000000, +0.000000, +0.000000, +1.001866],  # ID7
]

# DOF 标签 (用于日志)
DOF_LABELS = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']


def allocate(fx=0.0, fy=0.0, fz=0.0, mx=0.0, my=0.0, mz=0.0):
    """推力分配: 6-DOF 力/力矩 -> 7 电机归一化命令

    参数 (全部 [-1, +1]):
      fx: 前进 (+ = 前进)
      fy: 侧向 (+ = 右移)
      fz: 垂直 (+ = 下潜)
      mx: 横滚 (+ = 右滚)
      my: 俯仰 (+ = 抬头)
      mz: 偏航 (+ = 右转)

    返回: dict {motor_id: normalized_command}
      所有值 in [-1, +1], + = 前进/主方向
      均匀饱和: 若任一电机超限, 全部等比缩小 (保持方向)
    """
    tau = [fx, fy, fz, mx, my, mz]
    u = {}
    for i, mid in enumerate(MOTOR_IDS):
        u[mid] = sum(BPLUS[i][j] * tau[j] for j in range(6))

    # 均匀饱和
    max_abs = max(abs(v) for v in u.values()) if u else 0.0
    if max_abs > 1.0:
        scale = 1.0 / max_abs
        for k in u:
            u[k] *= scale

    return u


def allocate_with_detail(fx=0.0, fy=0.0, fz=0.0, mx=0.0, my=0.0, mz=0.0):
    """推力分配 (带详细信息, 用于调试)

    返回: (u_dict, detail_dict)
      u_dict: {motor_id: normalized_command}
      detail_dict: {
        'tau': [fx, fy, fz, mx, my, mz],
        'u_raw': {motor_id: raw_command},
        'u_scaled': {motor_id: scaled_command},
        'saturation': float (饱和比, 1.0 = 无饱和),
      }
    """
    tau = [fx, fy, fz, mx, my, mz]
    u_raw = {}
    for i, mid in enumerate(MOTOR_IDS):
        u_raw[mid] = sum(BPLUS[i][j] * tau[j] for j in range(6))

    max_abs = max(abs(v) for v in u_raw.values()) if u_raw else 0.0
    sat = 1.0 / max_abs if max_abs > 1.0 else 1.0

    u_scaled = {}
    for k in u_raw:
        u_scaled[k] = u_raw[k] * sat

    return u_scaled, {
        'tau': tau,
        'u_raw': u_raw,
        'u_scaled': u_scaled,
        'saturation': sat,
    }


# ── 自测 ──────────────────────────────────────────────────────────
if __name__ == '__main__':
    print('=== Thrust Allocator Self-Test ===')
    print()

    tests = [
        ('Pure Forward (Fx=1)',    1, 0, 0, 0, 0, 0),
        ('Pure Backward (Fx=-1)', -1, 0, 0, 0, 0, 0),
        ('Pure Dive (Fz=1)',       0, 0, 1, 0, 0, 0),
        ('Pure Surface (Fz=-1)',   0, 0, -1, 0, 0, 0),
        ('Pure Roll R (Mx=1)',     0, 0, 0, 1, 0, 0),
        ('Pure Pitch Up (My=1)',   0, 0, 0, 0, 1, 0),
        ('Pure Yaw R (Mz=1)',      0, 0, 0, 0, 0, 1),
        ('Forward+Dive',           0.5, 0, 0.5, 0, 0, 0),
        ('All DOF',                0.3, 0.2, 0.4, 0.1, 0.1, 0.2),
    ]

    for name, fx, fy, fz, mx, my, mz in tests:
        u, detail = allocate_with_detail(fx, fy, fz, mx, my, mz)
        sat = detail['saturation']
        sat_str = ' [SAT {:.2f}]'.format(sat) if sat < 1.0 else ''
        motors_str = '  '.join('ID{}={:+.3f}'.format(mid, u[mid]) for mid in MOTOR_IDS)
        print('{}:  {}{}'.format(name, motors_str, sat_str))

    print()
    print('=== Allocation Matrix B+ ===')
    print('         Fx       Fy       Fz       Mx       My       Mz')
    for i, mid in enumerate(MOTOR_IDS):
        vals = '  '.join('{:8.4f}'.format(BPLUS[i][j]) for j in range(6))
        print('ID{}: {}'.format(mid, vals))
