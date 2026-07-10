#!/usr/bin/env python3
"""重新计算推力分配矩阵的伪逆 B+ (v1.4)
修正 ID6 方向: 叶片反装意味着 CW=下潜 (不是上浮)
"""
import math
import numpy as np

# 22.5 deg 矢量倾角
_A = 22.5 * math.pi / 180
_CA = math.cos(_A)   # 0.9239
_SA = math.sin(_A)   # 0.3827

# 推力方向系数 (g_i > 0 时电机产生的力方向)
# 格式: (fx, fy, fz)
# 重要: 这是物理方向, CAN反相在motor_controller的build_ctrl中处理
_THRUST_DIR = {
    0: (_CA * _CA,  _CA * _SA,  _SA),    # 右上尾推: 前, 右, 下
    1: (_CA * _CA,  _CA * _SA, -_SA),    # 右下尾推: 前, 右, 上
    2: (_CA * _CA, -_CA * _SA, -_SA),    # 左下尾推: 前, 左, 上
    3: (_CA * _CA, -_CA * _SA,  _SA),    # 左上尾推: 前, 左, 下
    5: (0.0, 0.0, -1.0),                 # 左垂推: CW=上浮, g>0->CW
    6: (0.0, 0.0, +1.0),                 # 右垂推: 叶片反装, CW=下潜, g>0->CW->下潜
    7: (0.0, 1.0, 0.0),                  # 横推: 右
}

# 电机位置 (相对质心, 米)
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

# 计算 B 矩阵 (6x7)
# 每列 = 电机 i 在 g=1 时对 6-DOF 力/力矩的贡献
B = np.zeros((6, 7))
for i, mid in enumerate(MOTOR_IDS):
    fx, fy, fz = _THRUST_DIR[mid]
    px, py, pz = _POS[mid]
    
    # 力分量
    B[0, i] = fx  # Fx
    B[1, i] = fy  # Fy
    B[2, i] = fz  # Fz
    
    # 力矩分量: M = r x f
    B[3, i] = py * fz - pz * fy  # Mx (横滚)
    B[4, i] = pz * fx - px * fz  # My (俯仰)
    B[5, i] = px * fy - py * fx  # Mz (偏航)

print("=== v1.4: 修正ID6方向 ===")
print("\n推力方向 (物理方向, 不补偿CAN反相):")
for k, v in _THRUST_DIR.items():
    print(f"  ID{k}: fx={v[0]:+.3f}, fy={v[1]:+.3f}, fz={v[2]:+.3f}")

print("\nB 矩阵 (6x7):")
labels = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
for i in range(6):
    row = f"  {labels[i]}: ["
    for j in range(7):
        row += f"{B[i,j]:+.3f}"
        if j < 6:
            row += ", "
    row += "]"
    print(row)

# 计算伪逆 B+ = B^T * (B * B^T)^(-1)
B_plus = B.T @ np.linalg.inv(B @ B.T)

print("\nB+ 矩阵 (7x6):")
for i in range(7):
    row = f"  ID{MOTOR_IDS[i]}: ["
    for j in range(6):
        row += f"{B_plus[i,j]:+.6f}"
        if j < 5:
            row += ", "
    row += "]"
    print(row)

# 验证 B * B+ = I (应该接近6x6单位矩阵)
I_approx = B @ B_plus
print("\n验证 B * B+ (应该接近单位矩阵):")
max_error = 0.0
for i in range(6):
    for j in range(6):
        error = abs(I_approx[i,j] - (1.0 if i == j else 0.0))
        max_error = max(max_error, error)
print(f"  Max error: {max_error:.2e}")
if max_error < 1e-10:
    print("  ✓ 验证通过!")
else:
    print("  ✗ 验证失败!")

# 测试: fz=+1 (下潜) 时, 垂推的g值应该为负 (产生下潜力)
print("\n测试: fz=+1 (下潜) 时的电机命令:")
tau_test = [0, 0, 1, 0, 0, 0]  # 只有Fz=+1
u_test = {}
for i, mid in enumerate(MOTOR_IDS):
    u_test[mid] = sum(B_plus[i][j] * tau_test[j] for j in range(6))
print("  垂推 g 值 (应该为负, 表示下潜):")
print(f"    ID5: {u_test[5]:+.3f}")
print(f"    ID6: {u_test[6]:+.3f}")

# 输出可用于代码的 B+ 矩阵
print("\n" + "="*60)
print("复制到 thrust_allocator.py:")
print("="*60)
print("BPLUS = [")
for i in range(7):
    row = "    ["
    for j in range(6):
        row += f"{B_plus[i,j]:+.6f}"
        if j < 5:
            row += ", "
    row += "],  # ID" + str(MOTOR_IDS[i])
    print(row)
print("]")
