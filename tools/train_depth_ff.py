#!/usr/bin/env python3
"""
深度前馈补偿模型训练 v1.0

用法 (工作站):
  python train_depth_ff.py <csv_file> [--output ff_coeffs.json]

输入: auto_depth_test.py 生成的 CSV
输出: 前馈补偿系数 (4 个浮点数), 供 motor_controller 集成

方法:
  1. 提取稳态样本: |depth_error| < 3cm
  2. 目标 y: 此时 PID 输出的 depth_pid_out (即平衡推力)
  3. 特征 X: [1.0, target_depth, sin(pitch_rad), sin(roll_rad)]
  4. 最小二乘拟合: fz_ff = w0 + w1*depth + w2*sin_pitch + w3*sin_roll
  5. 输出系数写入 JSON, 并打印诊断报告
"""

import sys
import csv
import json
import math
from collections import defaultdict


def load_and_parse(csv_path):
    """加载 CSV, 返回样本列表 [dict]"""
    samples = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                s = {
                    'round':         int(row['round']),
                    'target_depth':  float(row['target_depth']),
                    'elapsed_sec':   float(row['elapsed_sec']),
                    'current_depth': float(row['current_depth']),
                    'depth_error':   float(row['depth_error']),
                    'depth_pid_out': float(row['depth_pid_out']),
                    'depth_err_i':   float(row['depth_err_i']),
                    'pitch_deg':     float(row['pitch_deg']),
                    'roll_deg':      float(row['roll_deg']),
                    'yaw_deg':       float(row['yaw_deg']),
                    'id0':           float(row['id0']),
                    'id1':           float(row['id1']),
                    'id2':           float(row['id2']),
                    'id3':           float(row['id3']),
                    'id5':           float(row['id5']),
                    'id6':           float(row['id6']),
                    'id7':           float(row['id7']),
                    'fz_vert_avg':   float(row['fz_vert_avg']),
                    'fz_tail_avg':   float(row['fz_tail_avg']),
                }
                samples.append(s)
            except (ValueError, KeyError):
                continue
    return samples


def filter_steady_state(samples, err_threshold=0.03):
    """提取稳态样本: |depth_error| < threshold"""
    steady = [s for s in samples if abs(s['depth_error']) < err_threshold]
    return steady


def build_features(samples):
    """构建特征矩阵 X (Nx4) 和目标向量 y (N,)"""
    X = []
    y = []
    for s in samples:
        pitch_rad = math.radians(s['pitch_deg'])
        roll_rad  = math.radians(s['roll_deg'])
        feat = [
            1.0,                        # bias
            s['target_depth'],          # depth
            math.sin(pitch_rad),        # sin(pitch)
            math.sin(roll_rad),         # sin(roll)
        ]
        X.append(feat)
        y.append(s['depth_pid_out'])
    return X, y


def least_squares(X, y):
    """普通最小二乘: w = (X^T X)^-1 X^T y"""
    n = len(X)
    p = len(X[0])

    # X^T X
    XtX = [[0.0] * p for _ in range(p)]
    Xty = [0.0] * p
    for i in range(n):
        for j in range(p):
            Xty[j] += X[i][j] * y[i]
            for k in range(p):
                XtX[j][k] += X[i][j] * X[i][k]

    # 高斯消元求解 (小矩阵, 直接手写)
    # 增广矩阵
    aug = [row[:] + [Xty[i]] for i, row in enumerate(XtX)]
    for col in range(p):
        # 选主元
        pivot = col
        for r in range(col + 1, p):
            if abs(aug[r][col]) > abs(aug[pivot][col]):
                pivot = r
        aug[col], aug[pivot] = aug[pivot], aug[col]
        if abs(aug[col][col]) < 1e-12:
            continue
        # 消元
        for r in range(col + 1, p):
            factor = aug[r][col] / aug[col][col]
            for c in range(col, p + 1):
                aug[r][c] -= factor * aug[col][c]
    # 回代
    w = [0.0] * p
    for i in range(p - 1, -1, -1):
        if abs(aug[i][i]) < 1e-12:
            w[i] = 0.0
            continue
        s = aug[i][p]
        for j in range(i + 1, p):
            s -= aug[i][j] * w[j]
        w[i] = s / aug[i][i]
    return w


def compute_r_squared(X, y, w):
    """计算 R² (决定系数)"""
    n = len(y)
    ss_res = 0.0
    ss_tot = 0.0
    y_mean = sum(y) / n
    for i in range(n):
        y_pred = sum(X[i][j] * w[j] for j in range(len(w)))
        ss_res += (y[i] - y_pred) ** 2
        ss_tot += (y[i] - y_mean) ** 2
    if ss_tot < 1e-12:
        return 1.0
    return 1.0 - ss_res / ss_tot


def make_residual(X, y, w):
    """计算每个样本的残差"""
    residuals = []
    for i in range(len(y)):
        y_pred = sum(X[i][j] * w[j] for j in range(len(w)))
        residuals.append(y[i] - y_pred)
    return residuals


def fmt_rpm(v):
    if v > 0:
        return '{:.0f}'.format(v)
    return ' {}'.format(int(v))


def print_report(samples, steady, X_steady, y_steady, w):
    """打印诊断报告"""

    # ── 总体统计 ──
    print()
    print('=' * 58)
    print('  深度前馈补偿模型 — 训练报告')
    print('=' * 58)
    print()
    print('  总样本:  {} | 稳态样本: {} (|err| < 3cm)'.format(
        len(samples), len(steady)))
    print()

    # ── 每轮稳态统计 ──
    rounds = defaultdict(list)
    for s in steady:
        rounds[s['round']].append(s)
    print('  各轮稳态统计:')
    print('  {:>4s}  {:>6s}  {:>8s}  {:>8s}  {:>8s}'.format(
        '轮次', '目标m', 'PID均值', 'PID std', '样本数'))
    print('  ' + '-' * 44)
    for r in sorted(rounds.keys()):
        group = rounds[r]
        vals = [s['depth_pid_out'] for s in group]
        avg  = sum(vals) / len(vals)
        var  = sum((v - avg)**2 for v in vals) / len(vals)
        std  = math.sqrt(var)
        print('  {:>4d}  {:>6.2f}  {:>8.4f}  {:>8.4f}  {:>8d}'.format(
            r, group[0]['target_depth'], avg, std, len(vals)))
    print()

    # ── 模型系数 ──
    r2 = compute_r_squared(X_steady, y_steady, w)
    print('  拟合模型: fz_ff = bias + w_d*depth + w_p*sin(pitch) + w_r*sin(roll)')
    print()
    print('  {:>20s}  {:>12s}'.format('参数', '值'))
    print('  ' + '-' * 34)
    names = ['bias (w0)', 'depth_coeff (w1)', 'sin_pitch_coeff (w2)', 'sin_roll_coeff (w3)']
    for i, name in enumerate(names):
        print('  {:>20s}  {:>+12.6f}'.format(name, w[i]))
    print()
    print('  R² = {:.4f}'.format(r2))
    print()

    # ── 残差统计 ──
    residuals = make_residual(X_steady, y_steady, w)
    res_abs = [abs(r) for r in residuals]
    res_avg = sum(res_abs) / len(res_abs)
    res_max = max(res_abs)
    print('  残差 (实际PID输出 vs 模型预测):')
    print('    平均 |残差| = {:.5f}'.format(res_avg))
    print('    最大 |残差| = {:.5f}'.format(res_max))
    print()

    # ── 预测 vs 实际 (每轮) ──
    print('  每轮预测 vs 实际:')
    print('  {:>4s}  {:>6s}  {:>8s}  {:>8s}  {:>8s}'.format(
        '轮次', '目标m', '实际PID', '预测FF', '残差'))
    print('  ' + '-' * 44)
    for r in sorted(rounds.keys()):
        group = rounds[r]
        # 取后一半样本 (更稳定)
        half = group[len(group)//2:]
        s = half[0] if half else group[-1]
        pitch_rad = math.radians(s['pitch_deg'])
        roll_rad  = math.radians(s['roll_deg'])
        ff_pred = w[0] + w[1]*s['target_depth'] + w[2]*math.sin(pitch_rad) + w[3]*math.sin(roll_rad)
        avg_pid = sum(g['depth_pid_out'] for g in group) / len(group)
        print('  {:>4d}  {:>6.2f}  {:>8.4f}  {:>8.4f}  {:>+8.4f}'.format(
            r, s['target_depth'], avg_pid, ff_pred, avg_pid - ff_pred))
    print()

    # ── 使用建议 ──
    print('  ' + '-' * 58)
    print('  集成建议:')
    print()
    print('  在 motor_controller.py 的 heartbeat_tick() 中:')
    print()
    print('    fz = depth_pid_out + fz_ff')
    print()
    print('  其中 fz_ff = {:.6f} + {:.6f}*target_depth + {:.6f}*sin(pitch_rad)'.format(
        w[0], w[1], w[2]))
    if abs(w[3]) > 1e-6:
        print('             + {:.6f}*sin(roll_rad)'.format(w[3]))
    print()
    print('  建议 FF_GAIN = 0.5 开始, 逐步增加到 1.0')
    print('  模型预测了稳态平衡推力, PID 只需处理残差')
    print('=' * 58)


def save_coeffs(w, path):
    """保存系数到 JSON"""
    d = {
        'version': 1,
        'comment': 'Depth feedforward compensation model',
        'model': 'linear',
        'features': ['bias', 'target_depth_m', 'sin_pitch_rad', 'sin_roll_rad'],
        'coefficients': {
            'bias':           round(w[0], 8),
            'depth_coeff':    round(w[1], 8),
            'sin_pitch_coeff': round(w[2], 8),
            'sin_roll_coeff':  round(w[3], 8),
        },
        'ff_gain_suggested': 0.5,
        'r_squared': round(compute_r_squared(
            *build_features(filter_steady_state(load_and_parse(sys.argv[1]))),
            w
        ), 4)
    }
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(d, f, indent=2, ensure_ascii=False)
    print('  系数已保存: {}'.format(path))


def main():
    if len(sys.argv) < 2:
        print('用法: python train_depth_ff.py <csv_file> [--save ff_coeffs.json]')
        sys.exit(1)

    csv_path = sys.argv[1]
    print('读取数据: {}'.format(csv_path))

    samples = load_and_parse(csv_path)
    if not samples:
        print('错误: CSV 无有效数据')
        sys.exit(1)

    steady = filter_steady_state(samples, err_threshold=0.03)
    if len(steady) < 20:
        print('警告: 稳态样本仅 {} 个, 拟合可能不可靠'.format(len(steady)))

    X_steady, y_steady = build_features(steady)
    w = least_squares(X_steady, y_steady)

    print_report(samples, steady, X_steady, y_steady, w)

    # 可选: 保存系数
    save_flag = False
    save_path = None
    for i, arg in enumerate(sys.argv[2:], 2):
        if arg == '--save' and i + 1 < len(sys.argv):
            save_path = sys.argv[i + 1]
            save_flag = True
            break
    if save_flag:
        save_coeffs(w, save_path)


if __name__ == '__main__':
    main()
