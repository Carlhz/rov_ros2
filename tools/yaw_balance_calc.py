#!/usr/bin/env python3
"""
Yaw 推力平衡计算
分析尾部4电机 + ID7 协同转向的力和力矩分配

坐标系: X+前 Y+右 Z+下 (NED)
Mz+ = 右转 (CW)
"""

import math

A = 22.5 * math.pi / 180
CA = math.cos(A)   # 0.9239
SA = math.sin(A)   # 0.3827

# 推力方向 (g>0=前进)
THRUST_DIR = {
    0: (CA*CA,  CA*SA,  SA),     # (0.854, 0.354, 0.383) 右上: 前+右+下
    1: (CA*CA,  CA*SA, -SA),     # (0.854, 0.354,-0.383) 右下: 前+右+上
    2: (CA*CA, -CA*SA, -SA),     # (0.854,-0.354,-0.383) 左下: 前+左+上
    3: (CA*CA, -CA*SA,  SA),     # (0.854,-0.354, 0.383) 左上: 前+左+下
    7: (0.0, 1.0, 0.0),          # 横推: 右
}

# 电机位置 (相对质心)
POS = {
    0: (-0.527,  0.080, -0.062),
    1: (-0.527,  0.080,  0.073),
    2: (-0.527, -0.080,  0.073),
    3: (-0.527, -0.080, -0.062),
    7: ( 0.278, -0.050,  0.018),
}

def mz_contribution(mid, fx, fy):
    """计算单个电机对Mz的贡献: Mz = rx*fy - ry*fx"""
    rx, ry, _ = POS[mid]
    return rx * fy - ry * fx

def analyze_cw_turn(tail_norm, id7_norm):
    """
    分析 CW (右转) 转向:
    - ID0,1: 后退 (-norm)  → 产生 CW 力矩
    - ID2,3: 前进 (+norm)  → 产生 CW 力矩  
    - ID7:   前进 (+norm)  → 产生 CW 力矩 + 右向力平衡左侧力
    """
    # 尾部各电机
    forces = {}
    torques = {}
    
    # ID0: backward
    fx0 = -tail_norm * THRUST_DIR[0][0]
    fy0 = -tail_norm * THRUST_DIR[0][1]
    mz0 = mz_contribution(0, fx0, fy0)
    
    # ID1: backward
    fx1 = -tail_norm * THRUST_DIR[1][0]
    fy1 = -tail_norm * THRUST_DIR[1][1]
    mz1 = mz_contribution(1, fx1, fy1)
    
    # ID2: forward
    fx2 = tail_norm * THRUST_DIR[2][0]
    fy2 = tail_norm * THRUST_DIR[2][1]
    mz2 = mz_contribution(2, fx2, fy2)
    
    # ID3: forward
    fx3 = tail_norm * THRUST_DIR[3][0]
    fy3 = tail_norm * THRUST_DIR[3][1]
    mz3 = mz_contribution(3, fx3, fy3)
    
    # ID7: forward (右推)
    fx7 = 0.0
    fy7 = id7_norm * THRUST_DIR[7][1]
    mz7 = mz_contribution(7, fx7, fy7)
    
    fx_net = fx0 + fx1 + fx2 + fx3 + fx7
    fy_net = fy0 + fy1 + fy2 + fy3 + fy7
    mz_net = mz0 + mz1 + mz2 + mz3 + mz7
    
    return {
        'fx_net': fx_net,
        'fy_net': fy_net,
        'mz_net': mz_net,
        'per_motor': {
            0: (fx0, fy0, mz0),
            1: (fx1, fy1, mz1),
            2: (fx2, fy2, mz2),
            3: (fx3, fy3, mz3),
            7: (fx7, fy7, mz7),
        }
    }

# ── 主计算 ──
print("=" * 70)
print("尾推 + ID7 协同转向力分析")
print("=" * 70)

# 先看纯尾推(无ID7)和纯ID7(无尾推)
print("\n--- 单独对比 ---")
r1 = analyze_cw_turn(1.0, 0.0)
print(f"纯尾推(norm=1): Fx={r1['fx_net']:+.4f} Fy={r1['fy_net']:+.4f} Mz={r1['mz_net']:+.4f}")

r2 = analyze_cw_turn(0.0, 1.0)
print(f"纯ID7 (norm=1):  Fx={r2['fx_net']:+.4f} Fy={r2['fy_net']:+.4f} Mz={r2['mz_net']:+.4f}")

# 计算力平衡所需的 ID7/tail 比例
# Fy: tail产生的Fy + ID7产生的Fy = 0
# 尾推Fy系数(4电机) = -0.354*4 = -1.416 (CW时尾推产生左向力)
# ID7 Fy系数 = +1.0 (右推)
# 平衡条件: -1.416 * tail_norm + 1.0 * id7_norm = 0
# → id7_norm = 1.416 * tail_norm
print(f"\n--- 力平衡条件 ---")
print(f"尾推Fy系数: {r1['fy_net']:.4f}  (负=左向力, CW时尾推净Fy)")
print(f"ID7 Fy系数:  {r2['fy_net']:.4f}  (正=右向力)")
fy_ratio = abs(r1['fy_net'] / r2['fy_net'])
print(f"力平衡所需: ID7_norm = {fy_ratio:.3f} * tail_norm")

# 检查力平衡时的RPM
print(f"\n--- RPM分析 (tail=1100-1550, ID7=1100-1400) ---")
for tail_n in [0.2, 0.4, 0.6, 0.8, 1.0]:
    id7_n_balanced = fy_ratio * tail_n
    tail_rpm = 1100 + tail_n * 450
    id7_rpm = 1100 + id7_n_balanced * 300
    diff = id7_rpm - tail_rpm
    print(f"  tail_norm={tail_n:.1f}→{tail_rpm:.0f}RPM  id7_norm={id7_n_balanced:.3f}→{id7_rpm:.0f}RPM  "
          f"差值={diff:+.0f}  {'ID7>尾推 ✓' if diff > 0 else 'ID7<尾推 ✗'}")

# ── 寻找不完美平衡但 RPM 关系正确的比例 ──
print(f"\n--- 寻找 k = tail_norm/id7_norm 使 tail_rpm < id7_rpm ---")
print(f"{'k':>8}  {'tail_n':>8}  {'id7_n':>8}  {'tail_rpm':>9}  {'id7_rpm':>9}  {'rpm差':>7}  {'Fy_imbal':>9}  {'状态'}")
for k in [0.2, 0.3, 0.4, 0.5, 0.55, 0.6, 0.65, 0.7]:
    # 固定 id7_norm=1.0, tail_norm=k
    r = analyze_cw_turn(k, 1.0)
    tail_rpm = 1100 + k * 450
    id7_rpm = 1400
    diff = id7_rpm - tail_rpm
    fy_imbal = r['fy_net']
    status = '✓ ID7>尾推' if diff > 0 else '✗ 尾推≥ID7'
    print(f"  {k:.3f}    {k:.3f}      1.000      {tail_rpm:.0f}         {id7_rpm}        {diff:+4.0f}    {fy_imbal:+9.4f}    {status}")

# ── 最佳比例推荐 ──
print(f"\n--- 推荐: TAIL_YAW_RATIO = 0.5 ---")
r_best = analyze_cw_turn(0.5, 1.0)
print(f"  尾推 norm=0.5 (RPM={1100+0.5*450:.0f})  ID7 norm=1.0 (RPM=1400)")
print(f"  Fx_net = {r_best['fx_net']:+.4f} (前后力平衡 ✓)")
print(f"  Fy_net = {r_best['fy_net']:+.4f} (小右漂, ΔRPM=1400-{1100+0.5*450:.0f}={1400-(1100+0.5*450):.0f} ✓)")
print(f"  Mz_net = {r_best['mz_net']:+.4f} (纯尾推的{r_best['mz_net']/r1['mz_net']*100:.0f}% + ID7的{r_best['per_motor'][7][2]/r2['mz_net']*100:.0f}%)")
print(f"  尾推 PID 系数: B+给予的Mz修正值 (由伪逆自动计算)")

# 验证: 检查TAIL_YAW_RATIO=0.5时各工况
print(f"\n--- 各工况验证 (TAIL_YAW_RATIO=0.5) ---")
for mz_demand in [0.2, 0.5, 0.7, 1.0]:
    r = analyze_cw_turn(0.5 * mz_demand, mz_demand)
    tail_rpm = 1100 + 0.5 * mz_demand * 450
    id7_rpm = 1100 + mz_demand * 300
    print(f"  mz={mz_demand:.1f}: tail={tail_rpm:.0f}RPM  ID7={id7_rpm:.0f}RPM  "
          f"ΔRPM={id7_rpm-tail_rpm:+.0f}  Fy={r['fy_net']:+.4f}  Mz={r['mz_net']:+.4f}")
