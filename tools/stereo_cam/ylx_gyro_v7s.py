#!/usr/bin/env python3
"""
YLX Gyro V7S - 使用 v4l2-ctl 捕获 + JPEG APP 段解析

简化为两个步骤：
  1. v4l2-ctl 捕获原始 MJPEG 帧到文件 (20帧)
  2. Python 逐帧解析 JPEG APPn 段，提取陀螺仪数据

厂商文档确认：
  - 陀螺仪数据在 JPEG header 的 APP 段
  - 每轴 2字节，低4位不用，12-bit signed
"""

import subprocess
import struct
import os
import sys
import tempfile

NUM_FRAMES = 30
DEVICE = '/dev/video0'


def parse_jpeg_segments(data):
    """解析 JPEG 数据中的所有段标记
    
    返回: list of (marker_byte, segment_length, payload_bytes, offset)
    """
    segments = []
    pos = 0
    data_len = len(data)
    
    while pos < data_len - 1:
        if data[pos] == 0xFF and data[pos+1] != 0x00 and data[pos+1] != 0xFF:
            marker = data[pos + 1]
            # 对于有长度字段的段: APP0-APP15, DQT, DHT, SOF0-SOF15
            if (0xE0 <= marker <= 0xEF) or marker in (0xDB, 0xC4, 0xC0, 0xC2):
                if pos + 4 <= data_len:
                    seg_len = struct.unpack('>H', data[pos+2:pos+4])[0]
                    if pos + 2 + seg_len <= data_len:
                        payload = data[pos+4:pos+2+seg_len]
                        segments.append((marker, seg_len, payload, pos))
                        pos += 2 + seg_len
                        continue
            # SOS (FFDA) - header 结束，图像数据开始
            if marker == 0xDA:
                seg_len = struct.unpack('>H', data[pos+2:pos+4])[0]
                segments.append((marker, seg_len, b'', pos))
                break
        pos += 1
    
    return segments


def decode_gyro(payload_bytes):
    """从 6 字节 payload 解码三轴陀螺仪值
    
    格式 (厂商文档):
      Y_hi Y_lo  X_hi X_lo  Z_hi Z_lo  (大端序)
      每轴 2 字节，取高 12 位 (>> 4)，低 4 位不用
      12-bit signed: > 2047 则为负
    """
    if len(payload_bytes) < 6:
        return None
    
    raw = struct.unpack('>HHH', payload_bytes[:6])
    gyro = []
    for v in raw:
        v12 = v >> 4
        if v12 > 2047:
            v12 -= 4096
        gyro.append(v12)
    return gyro  # [Y, X, Z]


def scan_payload_for_gyro(payload, marker_name, verbose=True):
    """在 APP 段 payload 中扫描陀螺仪特征
    
    特征: 每 2 字节一组，低 4 位全为 0
    """
    candidates = []
    
    for offset in range(0, len(payload) - 5):
        chunk = payload[offset:offset+6]
        # 检查低4位全为0
        low_bits = [b & 0x0F for b in chunk]
        all_low_zero = all(lb == 0 for lb in low_bits)
        
        if all_low_zero:
            g = decode_gyro(chunk)
            if g:
                candidates.append({
                    'offset': offset,
                    'raw': chunk,
                    'gyro': g,
                    'marker': marker_name,
                })
    
    return candidates


def main():
    print("=" * 65)
    print("  YLX Gyroscope Scanner V7S")
    print("  Step 1: Capture raw MJPEG via v4l2-ctl")
    print("  Step 2: Parse APP segments for gyro data")
    print("=" * 65)
    
    # Step 1: Capture raw MJPEG frames
    print(f"\n[1] Capturing {NUM_FRAMES} raw MJPEG frames from {DEVICE}...")
    
    outdir = '/tmp/ylx_v7s'
    os.makedirs(outdir, exist_ok=True)
    rawfile = os.path.join(outdir, 'raw_mjpeg.bin')
    
    cmd = [
        'v4l2-ctl', '-d', DEVICE,
        '--set-fmt-video', 'width=640,height=480,pixelformat=MJPG',
        '--stream-mmap',
        f'--stream-count={NUM_FRAMES}',
        f'--stream-to={rawfile}'
    ]
    
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    if result.returncode != 0:
        print(f"  ERROR: v4l2-ctl failed: {result.stderr}")
        # Try without setting format
        print("  Retrying without format setting...")
        cmd2 = [
            'v4l2-ctl', '-d', DEVICE,
            '--stream-mmap',
            f'--stream-count={NUM_FRAMES}',
            f'--stream-to={rawfile}'
        ]
        result = subprocess.run(cmd2, capture_output=True, text=True, timeout=30)
        if result.returncode != 0:
            print(f"  ERROR: {result.stderr}")
            sys.exit(1)
    
    filesize = os.path.getsize(rawfile)
    print(f"  Captured: {filesize} bytes ({filesize/1024:.1f} KB)")
    
    # Step 2: Find JPEG frame boundaries and parse
    print(f"\n[2] Parsing JPEG frames and analyzing APP segments...")
    print("-" * 65)
    
    with open(rawfile, 'rb') as f:
        all_data = f.read()
    
    # Find all SOI (FFD8) markers to split frames
    frame_starts = []
    pos = 0
    while pos < len(all_data) - 1:
        if all_data[pos] == 0xFF and all_data[pos+1] == 0xD8:
            frame_starts.append(pos)
        pos += 1
    
    # Add end marker
    frame_starts.append(len(all_data))
    
    print(f"  Found {len(frame_starts)-1} JPEG SOI markers")
    
    # Analyze each frame
    all_candidates = []
    app_summary = {}
    
    for fi in range(min(len(frame_starts)-1, NUM_FRAMES)):
        start = frame_starts[fi]
        end = frame_starts[fi+1] if fi < len(frame_starts)-2 else len(all_data)
        frame_data = all_data[start:end]
        
        segments = parse_jpeg_segments(frame_data)
        
        # Collect APP segment info
        for marker, seg_len, payload, offset in segments:
            name = f'APP{marker - 0xE0}' if 0xE0 <= marker <= 0xEF else f'0x{marker:02X}'
            if name not in app_summary:
                app_summary[name] = []
            
            candidates = scan_payload_for_gyro(payload, name, verbose=False)
            if candidates:
                all_candidates.extend([(fi, c) for c in candidates])
            
            # Track payload sizes
            app_summary[name].append(len(payload))
        
        # Print first 3 frames in detail
        if fi < 3:
            print(f"\n  Frame {fi}: {len(frame_data)} bytes, {len(segments)} segments")
            for marker, seg_len, payload, offset in segments:
                name = f'APP{marker - 0xE0}' if 0xE0 <= marker <= 0xEF else f'0x{marker:02X}'
                hexdump = payload[:48].hex(' ')
                print(f"    {name:6s} @{offset:5d} len={seg_len:5d} data({len(payload)}B): {hexdump}")
    
    # Summary
    print(f"\n{'='*65}")
    print("  APP SEGMENT SUMMARY")
    print(f"{'='*65}")
    for name, sizes in sorted(app_summary.items()):
        unique = set(sizes)
        print(f"  {name}: payload sizes = {sorted(unique)[:10]} (samples={len(sizes)})")
    
    # Gyro candidates
    print(f"\n{'='*65}")
    print("  GYRO CANDIDATES (low 4 bits all zero)")
    print(f"{'='*65}")
    
    if all_candidates:
        frames_with_gyro = set()
        for fi, c in all_candidates:
            frames_with_gyro.add(fi)
            g = c['gyro']
            marker = c['marker']
            raw_hex = c['raw'].hex(' ')
            print(f"  Frame {fi:3d} [{marker} @+{c['offset']:3d}] "
                  f"Y={g[0]:+5d}  X={g[1]:+5d}  Z={g[2]:+5d}  "
                  f"[hex: {raw_hex}]")
        
        # Check if values change
        gyro_values = [(fi, c['gyro']) for fi, c in all_candidates]
        all_same = True
        first_val = gyro_values[0][1]
        for _, g in gyro_values[1:]:
            if g != first_val:
                all_same = False
                break
        
        if all_same:
            print(f"\n  All {len(gyro_values)} readings identical → camera is stationary.")
            print(f"  Gyro data confirmed at rest: {first_val}")
            print(f"  Move the camera and re-run to verify dynamic response!")
        else:
            ys = [g[0] for _, g in gyro_values]
            xs = [g[1] for _, g in gyro_values]
            zs = [g[2] for _, g in gyro_values]
            print(f"\n  GYRO RESPONSE CONFIRMED!")
            print(f"  Y range: {min(ys):+5d} ~ {max(ys):+5d}")
            print(f"  X range: {min(xs):+5d} ~ {max(xs):+5d}")
            print(f"  Z range: {min(zs):+5d} ~ {max(zs):+5d}")
    else:
        print("  No 6-byte low-4-bits-zero candidates found!")
        print("\n  This could mean:")
        print("  1. Gyro data is not in a standard APP segment")
        print("  2. Gyro data has non-zero low 4 bits (different encoding)")
        print("  3. v4l2-ctl strips custom header data before SOI")
        print("\n  Checking for any non-standard markers...")
        # Look for any 0xFF marker that's not standard JPEG
        pos = 0
        found_custom = set()
        while pos < len(all_data) - 1:
            if all_data[pos] == 0xFF:
                marker = all_data[pos+1]
                if marker not in (0x00, 0x01, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD,
                                   0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5,
                                   0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7,
                                   0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF,
                                   0xFE):
                    found_custom.add(marker)
            pos += 1
        if found_custom:
            print(f"  Custom markers found: {[f'FF{hex(m)[2:].upper()}' for m in found_custom]}")
        else:
            print("  No custom markers found")
    
    print(f"\nDone. Raw data saved: {rawfile}")


if __name__ == '__main__':
    main()
