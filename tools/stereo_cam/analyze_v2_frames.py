#!/usr/bin/env python3
"""
深度分析 V2 捕获的原始 MJPEG 帧 — 逐字节对比找陀螺仪数据
========================================================
前提: V2 已用 v4l2-ctl 捕获了原始 MJPEG 帧(未经 cv2 重编码)
      文件在 ~/ylx_gyro_probe_v2/frame_XXXX.jpg

策略:
  1. 读取所有帧
  2. 逐字节对比 (diff), 特别关注 header 区域 (SOI→SOS)
  3. 对每个变动的字节区域做详细 hex dump
  4. 搜索陀螺仪特征模式 (低4位=0 的 6字节序列)
"""
import os, struct, sys

SAVE_DIR = os.path.expanduser("~/ylx_gyro_probe_v2")

def load_frames():
    frames = []
    for i in range(10):  # max 10 frames
        fpath = os.path.join(SAVE_DIR, f"frame_{i:04d}.jpg")
        if not os.path.exists(fpath):
            break
        with open(fpath, 'rb') as f:
            frames.append(f.read())
        print(f"  Frame {i}: {len(frames[-1])} bytes")
    return frames

def find_jpeg_segments(data):
    """返回所有 JPEG 段的 offset 和名称"""
    segs = []
    i = 0
    while i < len(data) - 1:
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i+1]
        if marker == 0xFF or marker == 0x00:
            i += 1
            continue
        if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            name = {0xD8: "SOI", 0xD9: "EOI"}.get(marker, f"RST{marker-0xD0}")
            segs.append((i, name, 2, None))
            i += 2
            continue
        if i + 4 > len(data): break
        seg_len = (data[i+2] << 8) | data[i+3]
        name = {0xE0+i: f"APP{i}" for i in range(16)}.get(marker, 
               {0xDB: "DQT", 0xC0: "SOF0", 0xC2: "SOF2", 0xC4: "DHT",
                0xDA: "SOS", 0xFE: "COM", 0xDD: "DRI"}.get(marker, f"MK{marker:02X}"))
        segs.append((i, name, 2 + seg_len, data[i+4:i+2+seg_len]))
        i += 2 + seg_len
    return segs

def hexdump(data, offset=0, indent="  "):
    lines = []
    for j in range(0, len(data), 16):
        chunk = data[j:j+16]
        hex_str = ' '.join(f'{b:02X}' for b in chunk)
        ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        lines.append(f"{indent}[{offset+j:5d}] {hex_str:<48s} |{ascii_str}|")
    return '\n'.join(lines)

def main():
    print("=" * 70)
    print("  YLX 陀螺仪 — V2 帧深度分析")
    print("=" * 70)
    
    frames = load_frames()
    if len(frames) < 2:
        print("错误: 需要至少 2 帧")
        return
    
    # 1. 打印第一帧的完整 header (SOI → SOS)
    segs = find_jpeg_segments(frames[0])
    sos_idx = None
    for off, name, _, _ in segs:
        if name == "SOS":
            sos_idx = off
            break
    
    header_end = sos_idx + 2 + ((frames[0][sos_idx+2] << 8) | frames[0][sos_idx+3]) if sos_idx else 1000
    
    print(f"\n  第一帧 JPEG 段结构:")
    for off, name, seg_len, sdata in segs:
        if name == "SOS":
            print(f"  [{off:5d}] {name}   len={seg_len-2:5d} → 扫描数据从 {off+seg_len} 开始")
            break
        preview = ' '.join(f'{b:02X}' for b in (sdata[:20] if sdata else b''))
        end_off = off + seg_len - 1
        print(f"  [{off:5d}-{end_off:5d}] {name:<6s} len={len(sdata) if sdata else 0:5d}: {preview}")
    
    # 2. 完整 dump Header 区域 (SOI → SOS)
    print(f"\n  完整 hex dump: 字节 0 - {header_end} (SOI 到 SOS 尾)")
    header_data = frames[0][:header_end]
    print(hexdump(header_data))
    
    # 3. 逐字节对比所有帧
    print(f"\n{'='*70}")
    print(f"  逐字节对比 ({len(frames)} 帧)")
    print(f"{'='*70}")
    
    min_len = min(len(f) for f in frames)
    
    # 记录每个字节位置的最小值、最大值、是否全相同
    diff_regions = []
    region_start = None
    
    for j in range(min_len):
        vals = [f[j] for f in frames]
        if len(set(vals)) > 1:
            if region_start is None:
                region_start = j
        else:
            if region_start is not None:
                diff_regions.append((region_start, j - 1, j - region_start))
                region_start = None
    
    if region_start is not None:
        diff_regions.append((region_start, min_len - 1, min_len - region_start))
    
    print(f"  变动区域: {len(diff_regions)} 个")
    
    # 分类: header 内和 header 外
    header_diffs = [(s, e, l) for s, e, l in diff_regions if s < header_end]
    data_diffs = [(s, e, l) for s, e, l in diff_regions if s >= header_end]
    
    print(f"  Header 内变动: {len(header_diffs)} 个")
    print(f"  数据区变动: {len(data_diffs)} 个 (忽略)")
    
    # 4. 详细分析 header 内变动
    for s, e, length in header_diffs:
        print(f"\n  --- 变动区域 [{s:5d}-{e:5d}] ({length} bytes) ---")
        
        # 显示每帧的值
        table = {}
        for fi, fdata in enumerate(frames[:8]):
            chunk = fdata[s:e+1]
            table[fi] = chunk
        
        for fi in sorted(table):
            print(f"    Frame {fi}: {' '.join(f'{b:02X}' for b in table[fi])}")
            # 尝试解析为整数
            if length in (2, 4, 6, 8):
                vals_int = []
                for jj in range(0, len(table[fi]), 2):
                    if jj + 1 < len(table[fi]):
                        v = (table[fi][jj] << 8) | table[fi][jj+1]
                        vals_int.append(v)
                print(f"            as u16: {vals_int}")
        
        # 陀螺仪检测
        for fi in sorted(table):
            data = table[fi]
            for jj in range(0, len(data) - 6, 2):
                v1 = (data[jj] << 8) | data[jj+1]
                v2 = (data[jj+2] << 8) | data[jj+3]
                v3 = (data[jj+4] << 8) | data[jj+5]
                if (v1 & 0x0F) == 0 and (v2 & 0x0F) == 0 and (v3 & 0x0F) == 0:
                    x, y, z = v1 >> 4, v2 >> 4, v3 >> 4
                    if max(x, y, z) > 0 and max(x, y, z) < 4096:
                        hex_s = ' '.join(f'{b:02X}' for b in data[jj:jj+6])
                        print(f"    → GYRO@+{jj} Frame{fi}: {hex_s} (X={x},Y={y},Z={z})")
    
    # 5. 特别分析: APP 段内容是否跨帧变化
    print(f"\n{'='*70}")
    print(f"  APP 段原始数据对比")
    print(f"{'='*70}")
    
    for off, name, _, sdata in segs:
        if not name.startswith("APP"):
            continue
        print(f"\n  {name} @{off}: {len(sdata) if sdata else 0} bytes")
        print(hexdump(sdata, indent="      "))
        
        # 对比其他帧相同 APP 段
        for fi in range(1, min(len(frames), 5)):
            segs2 = find_jpeg_segments(frames[fi])
            for off2, name2, _, sdata2 in segs2:
                if name2 == name and off2 == off:  # same position
                    if sdata2 != sdata:
                        print(f"\n    Frame {fi} 不同!")
                        print(hexdump(sdata2, indent="      "))
                    break
    
    # 6. 搜索非标准段
    print(f"\n{'='*70}")
    print(f"  非标准 JPEG 段扫描")
    print(f"{'='*70}")
    
    for fi in range(min(len(frames), 3)):
        segs_n = find_jpeg_segments(frames[fi])
        for off, name, _, _ in segs_n:
            if name.startswith("MK"):  # 未知 marker
                print(f"  Frame {fi}: 未知 marker @{off}: {name}")
    
    print(f"\n{'='*70}")
    print(f"  分析完成")
    print(f"{'='*70}")

if __name__ == "__main__":
    main()
