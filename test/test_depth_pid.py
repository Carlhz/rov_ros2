#!/usr/bin/env python3
"""
简单测试: 验证深度PID和推力分配逻辑
"""
import sys
import os

# 添加路径以便导入motor_controller和thrust_allocator
sys.path.insert(0, r'D:\Carl_WorkStation\rov_ros2\rk3588')

# 模拟 motor_controller 的配置
DEPTH_KP = 2.0
DEPTH_KI = 0.1
DEPTH_DEADBAND = 0.05

# 模拟深度PID
class MockPID:
    def __init__(self):
        self.depth_err_i = 0.0
        self.depth_pid_out = 0.0
        
    def compute(self, target_depth, current_depth):
        err = target_depth - current_depth
        
        # P
        p_error = 0.0 if abs(err) < DEPTH_DEADBAND else err
        p = DEPTH_KP * p_error
        
        # I (简化版)
        self.depth_err_i += DEPTH_KI * err * 0.1
        self.depth_err_i = max(-0.3, min(0.3, self.depth_err_i))
        
        self.depth_pid_out = max(-1.0, min(1.0, p + self.depth_err_i))
        return self.depth_pid_out

pid = MockPID()

# 测试场景: 目标深度0.5m, 当前深度0.1m
target = 0.5
current = 0.1

print("=== 测试深度PID + 推力分配 ===")
print(f"目标深度: {target}m")
print(f"当前深度: {current}m")
print(f"深度误差: {target - current:.3f}m")
print()

# 计算PID
fz = pid.compute(target, current)
print(f"PID输出 fz={fz:.3f} (+ = 下潜)")

# 导入推力分配函数
try:
    from thrust_allocator import allocate, BPLUS
    
    # 只测试深度控制 (mx=0, my=0, mz=0)
    fx = 0.0
    fy = 0.0
    mx = 0.0
    my = 0.0
    mz = 0.0
    
    alloc = allocate(fx, fy, fz, mx, my, mz)
    
    print(f"\n推力分配结果 (fz={fz:.3f}):")
    for mid, norm in alloc.items():
        print(f"  ID{mid}: {norm:+.3f}")
    
    # 转换为RPM (简化)
    def norm_to_rpm(norm, min_rpm, max_rpm):
        if norm == 0:
            return 0
        if norm > 0:
            return int(min_rpm + abs(norm) * (max_rpm - min_rpm))
        else:
            return -int(min_rpm + abs(norm) * (max_rpm - min_rpm))
    
    print(f"\n电机RPM (简化计算):")
    g = [0] * 8
    for mid, norm in alloc.items():
        if mid in (0,1,2,3):
            g[mid] = norm_to_rpm(norm, 1100, 1550)
        elif mid in (5,6):
            g[mid] = norm_to_rpm(norm, 1100, 1550)
        elif mid == 7:
            g[mid] = norm_to_rpm(norm, 1100, 1500)
    
    for i in range(8):
        if i in (0,1,2,3,5,6,7):
            print(f"  ID{i}: {g[i]:+} RPM")
    
    # 分析垂推方向
    print(f"\n垂推分析:")
    print(f"  ID5 (左垂推): {g[5]:+} RPM")
    print(f"  ID6 (右垂推): {g[6]:+} RPM")
    print(f"  说明: fz={fz:.3f} 表示下潜指令")
    print(f"  预期: 两垂推应该产生下潜力 (电机向下转)")
    
except Exception as e:
    print(f"导入错误: {e}")
    import traceback
    traceback.print_exc()
