#!/usr/bin/env python3
"""
YLX 陀螺仪数据探测
==================
协议: 帧号+三轴陀螺仪嵌入 MJPEG 帧的 JPEG header
     X/Y/Z 各 2 字节, 低 4 位无效, 取高 12 位
     (例: X=0x0A60 → 0x0A6, Y=0x00F0 → 0x00F, Z=0x1F30 → 0x1F3)

策略: 捕获原始 MJPEG 帧 → 保存完整 JPEG → 搜索陀螺仪数据位置
"""
import cv2
import os
import struct
import time

SAVE_DIR = os.path.expanduser("~/ylx_gyro_probe")
os.makedirs(SAVE_DIR, exist_ok=True)

def capture_raw_frames(cap, num_frames=5):
    """捕获原始 MJPEG 帧并保存为文件"""
    print(f"\n[1] 捕获 {num_frames} 帧原始 MJPEG ...")
    frames = []
    
    for i in range(num_frames):
        ret, frame = cap.read()
        if not ret:
            print(f"  ✗ 帧 {i+1} 读取失败")
            continue
        
        # 编码为 JPEG 保存原始大小
        success, buf = cv2.imencode('.jpg', frame)
        if not success:
            print(f"  ✗ 帧 {i+1} 编码失败")
            continue
        
        raw = buf.tobytes()
        frames.append(raw)
        
        fpath = os.path.join(SAVE_DIR, f"frame_{i:04d}.jpg")
        with open(fpath, 'wb') as f:
            f.write(raw)
        print(f"  ✓ 帧 {i+1}: {len(raw)} bytes → {fpath}")
    
    return frames


def scan_jpeg_structure(data, label=""):
    """分析 JPEG 结构: 列出所有 marker segment"""
    print(f"\n--- JPEG 结构分析: {label} ({len(data)} bytes) ---")
    
    i = 0
    markers = []
    while i < len(data) - 1:
        if data[i] != 0xFF:
            i += 1
            continue
        
        marker = data[i+1]
        
        # SOI (0xD8), EOI (0xD9) or RST (0xD0-0xD7) have no length
        if marker in (0xD8, 0xD9) or (0xD0 <= marker <= 0xD7):
            name = {
                0xD8: "SOI", 0xD9: "EOI",
                0xD0: "RST0", 0xD1: "RST1", 0xD2: "RST2", 0xD3: "RST3",
                0xD4: "RST4", 0xD5: "RST5", 0xD6: "RST6", 0xD7: "RST7",
            }.get(marker, f"0x{marker:02X}")
            markers.append((i, marker, name, 2, None))
            i += 2
            continue
        
        # Other markers: 2-byte length field
        name = {
            0xE0: "APP0", 0xE1: "APP1", 0xE2: "APP2", 0xE3: "APP3",
            0xE4: "APP4", 0xE5: "APP5", 0xE6: "APP6", 0xE7: "APP7",
            0xE8: "APP8", 0xE9: "APP9", 0xEA: "APP10", 0xEB: "APP11",
            0xEC: "APP12", 0xED: "APP13", 0xEE: "APP14", 0xEF: "APP15",
            0xDB: "DQT", 0xC0: "SOF0", 0xC2: "SOF2", 0xC4: "DHT",
            0xDA: "SOS", 0xFE: "COM",
        }.get(marker, f"0x{marker:02X}")
        
        if i + 4 > len(data):
            break
        
        seg_len = (data[i+2] << 8) | data[i+3]  # length includes 2 bytes of length field itself
        seg_data = data[i+4 : i+2+seg_len]
        seg_data_preview = ' '.join(f'{b:02X}' for b in seg_data[:16])
        
        markers.append((i, marker, name, 2 + seg_len, seg_data_preview))
        
        # Highlight APP segments
        if 0xE0 <= marker <= 0xEF:
            print(f"  [{i:6d}] {name:<6s} len={seg_len:5d}: {seg_data_preview}")
        elif marker in (0xDB, 0xC0, 0xC2, 0xC4, 0xDA):
            print(f"  [{i:6d}] {name:<6s} len={seg_len:5d}")
        
        i += 2 + seg_len
    
    # Also check SOS data area for embedded patterns
    sos_idx = next((idx for idx, m, _, _, _ in markers if m == 0xDA), None)
    if sos_idx is not None:
        sos_data_start = sos_idx + 2 + ((data[sos_idx+2] << 8) | data[sos_idx+3])
        print(f"  SOS 后数据起始: {sos_data_start}")
        print(f"  SOS 后数据尾: {len(data) - 2} (EOI)")
        
        # Show a few bytes before EOI (gyro might be in JPEG trailer)
        eoi_region = data[-100:]
        print(f"\n  JPEG 尾部最后 100 字节:")
        for j in range(0, 100, 16):
            hex_str = ' '.join(f'{b:02X}' for b in eoi_region[j:j+16])
            print(f"    [{len(data)-100+j:6d}] {hex_str}")
    
    # Search for potential 6-byte gyro pattern
    # Looking for 3 consecutive int16 values (X, Y, Z)
    print(f"\n  搜索可能的陀螺仪数据区域...")
    # 扫描整个 JPEG 文件，找符合 "两个字节一组，低4位=0" 模式的连续区域
    candidates = []
    for j in range(0, len(data) - 6, 2):
        # 检查连续6个字节是否每2字节的低4位都是0
        v1 = (data[j] << 8) | data[j+1]   # X candidate
        v2 = (data[j+2] << 8) | data[j+3] # Y candidate
        v3 = (data[j+4] << 8) | data[j+5] # Z candidate
        
        if (v1 & 0x0F) == 0 and (v2 & 0x0F) == 0 and (v3 & 0x0F) == 0:
            x_val = v1 >> 4
            y_val = v2 >> 4
            z_val = v3 >> 4
            # 过滤掉全0和明显不合理的大值
            if not (x_val == 0 and y_val == 0 and z_val == 0):
                if max(x_val, y_val, z_val) < 4096:  # 高12位最大值
                    candidates.append((j, x_val, y_val, z_val))
    
    if candidates:
        print(f"\n  找到 {len(candidates)} 个候选位置 (低4位全为0的6字节连续序列):")
        for j, x, y, z in candidates[:10]:
            hex_str = ' '.join(f'{b:02X}' for b in data[j:j+6])
            print(f"    @{j:6d}: {hex_str}  → gyro(x={x:4d}, y={y:4d}, z={z:4d})")
        if len(candidates) > 10:
            print(f"    ... 还有 {len(candidates)-10} 个")
    
    return markers


def main():
    print("=" * 60)
    print("  YLX 陀螺仪 JPEG Header 探测")
    print("=" * 60)
    
    # 1. 打开摄像头
    cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("错误: 无法打开摄像头")
        return
    
    # 设置 MJPEG 格式
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"摄像头: {w}x{h}")
    
    # 2. 捕获帧
    frames = capture_raw_frames(cap, 5)
    cap.release()
    
    if not frames:
        print("未捕获到帧")
        return
    
    # 3. 分析第一帧
    print("\n" + "=" * 60)
    print("  分析 JPEG 结构")
    print("=" * 60)
    scan_jpeg_structure(frames[0], "Frame 0")
    
    # 4. 多帧对比找差异
    if len(frames) >= 2:
        print(f"\n{'='*60}")
        print(f"  多帧对比 (找变动的字节位置)")
        print(f"{'='*60}")
        
        # 找所有帧中位置相同但值不同的字节
        min_len = min(len(f) for f in frames)
        diff_positions = []
        for j in range(min_len):
            vals = set(frames[i][j] for i in range(len(frames)))
            if len(vals) > 1:
                diff_positions.append(j)
        
        print(f"  共有 {len(diff_positions)} 个字节在不同帧间变动")
        
        # 聚焦在 JPEG header 区域（通常在 SOS 之前，即帧数据前 ~500 字节）
        sos_pos = None
        for j in range(len(frames[0]) - 1):
            if frames[0][j] == 0xFF and frames[0][j+1] == 0xDA:
                sos_pos = j
                break
        
        header_diffs = [p for p in diff_positions if p < (sos_pos or 600)]
        
        if header_diffs:
            print(f"\n  JPEG Header 区变动的字节 ({len(header_diffs)} 个):")
            # 按连续组显示
            groups = []
            if header_diffs:
                start = header_diffs[0]
                end = header_diffs[0]
                for p in header_diffs[1:]:
                    if p <= end + 2:
                        end = p
                    else:
                        groups.append((start, end))
                        start = end = p
                groups.append((start, end))
            
            for s, e in groups[:10]:
                length = e - s + 1
                print(f"    字节 {s:5d} - {e:5d} ({length:3d} bytes):")
                for fi, fdata in enumerate(frames[:5]):
                    hex_str = ' '.join(f'{b:02X}' for b in fdata[s:e+1])
                    print(f"      Frame {fi}: {hex_str}")
    
    print(f"\n{'='*60}")
    print(f"  原始帧已保存到: {SAVE_DIR}")
    print(f"{'='*60}")

if __name__ == "__main__":
    main()
