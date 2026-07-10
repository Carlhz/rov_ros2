#!/usr/bin/env python3
"""
YLX 陀螺仪数据探测 V3
=====================
V2 发现: 原始 MJPEG 帧的 JPEG header 中只有标准段，没有嵌入陀螺仪数据。
       APP1 只有 6 字节全零，不是陀螺仪数据的位置。

V3 新方向:
  1. 检查 UVC Metadata Video 节点 (UVC 1.5+)
  2. 检查原始 V4L2 buffer 中 JPEG 帧之外的额外数据
  3. 尝试 UVC Extension Unit (XU) 读取
  4. 用 v4l2-ctl 捕获的原始 stream 分析帧间填充/元数据
"""
import os, sys, subprocess, struct, time
from collections import defaultdict

SAVE_DIR = os.path.expanduser("~/ylx_gyro_probe_v3")
os.makedirs(SAVE_DIR, exist_ok=True)


def check_video_devices():
    """列出所有 /dev/video* 设备及其能力"""
    print("=" * 60)
    print("  [Step 1] 检查 UVC 视频设备")
    print("=" * 60)
    
    for n in range(10):
        dev = f"/dev/video{n}"
        if not os.path.exists(dev):
            continue
        
        print(f"\n--- {dev} ---")
        # 获取设备名称
        r = subprocess.run(["v4l2-ctl", "-d", dev, "--info"],
                          capture_output=True, text=True, timeout=5)
        for line in r.stdout.split('\n'):
            if 'Driver name' in line or 'Card type' in line or 'Bus info' in line:
                print(f"  {line.strip()}")
        
        # 列出支持的所有格式（包括 metadata）
        r = subprocess.run(["v4l2-ctl", "-d", dev, "--list-formats-ext"],
                          capture_output=True, text=True, timeout=5)
        print(f"  Formats:\n{r.stdout.strip()[:500]}")
        
        # 检查是否为 metadata 节点
        r2 = subprocess.run(["v4l2-ctl", "-d", dev, "--all"],
                           capture_output=True, text=True, timeout=5)
        for line in r2.stdout.split('\n'):
            if 'Metadata' in line or 'meta' in line.lower():
                print(f"  !! {line.strip()}")


def capture_raw_stream_with_headers():
    """
    使用 v4l2-ctl --stream-mmap 捕获原始流
    分析每帧的 V4L2 buffer header 和帧间数据
    """
    print("\n" + "=" * 60)
    print("  [Step 2] 捕获原始流并分析 buffer 结构")
    print("=" * 60)
    
    stream_path = os.path.join(SAVE_DIR, "raw_stream.bin")
    
    # 使用更大的帧数和输出格式来保留 buffer 信息
    subprocess.run(["v4l2-ctl", "-d", "/dev/video0",
                    "--set-fmt-video", "width=640,height=480,pixelformat=MJPG"],
                   capture_output=True, timeout=5)
    
    result = subprocess.run(
        ["v4l2-ctl", "-d", "/dev/video0", "--stream-mmap",
         "--stream-count=10", "--stream-to", stream_path],
        capture_output=True, text=True, timeout=30
    )
    
    with open(stream_path, 'rb') as f:
        data = f.read()
    
    print(f"  原始流总大小: {len(data)} bytes")
    
    # 寻找所有的 JPEG SOI 标记
    soi_positions = []
    pos = 0
    while True:
        soi = data.find(b'\xff\xd8', pos)
        if soi < 0:
            break
        soi_positions.append(soi)
        pos = soi + 1
    
    print(f"  找到 {len(soi_positions)} 个 SOI 标记")
    
    # 分析 SOI 之间的 gap（帧间数据）
    frame_analysis = []
    for i, soi in enumerate(soi_positions):
        eoi = data.find(b'\xff\xd9', soi + 2)
        gdata = data[soi: soi+6] if soi+6 <= len(data) else b''
        frame_len = (eoi - soi + 2) if eoi > 0 else 0
        
        # 到下一帧 SOI 的 gap
        next_soi = soi_positions[i+1] if i+1 < len(soi_positions) else len(data)
        gap_before_next = next_soi - (eoi + 2) if eoi > 0 else 0
        
        frame_analysis.append({
            'idx': i, 'soi': soi, 'eoi': eoi, 
            'len': frame_len, 'gap': gap_before_next
        })
        
        print(f"  Frame {i}: SOI@{soi} EOI@{eoi} len={frame_len} gap_to_next={gap_before_next} bytes")
    
    # 特别分析：如果 gap > 0，帧间可能有额外的元数据
    extra_data_regions = []
    for f in frame_analysis:
        if f['gap'] > 0 and f['gap'] < 1000:  # 忽略流末尾的大 gap
            gap_start = f['eoi'] + 2
            gap_data = data[gap_start: gap_start + f['gap']]
            extra_data_regions.append((f['idx'], gap_start, gap_data))
            hex_preview = ' '.join(f'{b:02X}' for b in gap_data[:64])
            print(f"\n  Frame {f['idx']} → Frame {f['idx']+1} 间数据 ({f['gap']} bytes):")
            print(f"    {hex_preview}")
            
            # 检查是否是陀螺仪格式 (低4位=0)
            for j in range(0, len(gap_data) - 6, 2):
                v1 = (gap_data[j] << 8) | gap_data[j+1]
                v2 = (gap_data[j+2] << 8) | gap_data[j+3]
                v3 = (gap_data[j+4] << 8) | gap_data[j+5]
                if (v1 & 0x0F) == 0 and (v2 & 0x0F) == 0 and (v3 & 0x0F) == 0:
                    x, y, z = v1 >> 4, v2 >> 4, v3 >> 4
                    if max(x, y, z) > 0:
                        _hex = ' '.join(f'{b:02X}' for b in gap_data[j:j+6])
                        print(f"      → gyro@+{j}: {_hex} (X={x},Y={y},Z={z})")


def check_uvc_extension_units():
    """
    检查 UVC Extension Unit (XU) 
    使用 /sys/kernel/debug/usb/devices 或 lsusb -v
    """
    print("\n" + "=" * 60)
    print("  [Step 3] 检查 UVC Extension Units")
    print("=" * 60)
    
    # lsusb 找到 YLX 摄像头
    r = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=5)
    for line in r.stdout.split('\n'):
        if 'Sunplus' in line or '1bcf' in line or '0b15' in line.lower():
            print(f"  设备: {line.strip()}")
            # 提取 bus/device
            parts = line.strip().split()
            if len(parts) >= 4:
                bus = parts[1]
                dev = parts[3].rstrip(':')
                print(f"  Bus={bus}, Device={dev}")
                
                # 用 lsusb -v 获取详细信息
                r2 = subprocess.run(["lsusb", "-v", "-s", f"{bus}:{dev}"],
                                   capture_output=True, text=True, timeout=10)
                
                # 查找 Extension Unit
                in_xu = False
                xu_lines = []
                for l2 in r2.stdout.split('\n'):
                    if 'EXTENSION_UNIT' in l2:
                        in_xu = True
                    if in_xu:
                        xu_lines.append(l2)
                        if l2.strip() == '':
                            in_xu = False
                            break
                
                if xu_lines:
                    print(f"  Extension Units:")
                    for l in xu_lines:
                        print(f"    {l.strip()}")
                else:
                    print(f"  (未找到 Extension Unit)")

    # 检查 /dev/uvc* 设备
    for n in range(5):
        uvcdev = f"/dev/uvc{n}"
        if os.path.exists(uvcdev):
            print(f"  找到 UVC 设备: {uvcdev}")


def check_metadata_node():
    """检查 /dev/video 中是否有 metadata 节点"""
    print("\n" + "=" * 60)
    print("  [Step 4] Metadata 节点探测")
    print("=" * 60)
    
    for n in range(10):
        dev = f"/dev/video{n}"
        if not os.path.exists(dev):
            continue
        
        # 获取 capabilities
        r = subprocess.run(["v4l2-ctl", "-d", dev, "--info"],
                          capture_output=True, text=True, timeout=5)
        
        is_meta = False
        for line in r.stdout.split('\n'):
            if 'Metadata' in line or 'meta output' in line.lower() or 'meta capture' in line.lower():
                is_meta = True
                print(f"  {dev}: {line.strip()}")
        
        if is_meta:
            # 尝试读取 metadata
            r2 = subprocess.run(["v4l2-ctl", "-d", dev, "--list-formats-ext"],
                               capture_output=True, text=True, timeout=5)
            print(f"  {dev} formats:\n{r2.stdout.strip()[:400]}")
            
            # 尝试 stream metadata
            meta_path = os.path.join(SAVE_DIR, f"metadata_{n}.bin")
            r3 = subprocess.run(
                ["v4l2-ctl", "-d", dev, "--stream-mmap",
                 "--stream-count=5", "--stream-to", meta_path],
                capture_output=True, text=True, timeout=15
            )
            print(f"  Metadata stream result: {r3.stderr.strip()[:200]}")
            
            if os.path.exists(meta_path) and os.path.getsize(meta_path) > 0:
                with open(meta_path, 'rb') as f:
                    mdata = f.read()
                print(f"  Metadata size: {len(mdata)} bytes")
                print(f"  First 64 bytes: {' '.join(f'{b:02X}' for b in mdata[:64])}")
                
                # 分析 metadata
                chk = []
                for j in range(0, min(len(mdata)-6, 200), 2):
                    v1 = (mdata[j] << 8) | mdata[j+1]
                    v2 = (mdata[j+2] << 8) | mdata[j+3]
                    v3 = (mdata[j+4] << 8) | mdata[j+5]
                    if (v1 & 0x0F) == 0 and (v2 & 0x0F) == 0 and (v3 & 0x0F) == 0:
                        x, y, z = v1 >> 4, v2 >> 4, v3 >> 4
                        if max(x, y, z) > 0 and max(x, y, z) < 4096:
                            _hex = ' '.join(f'{b:02X}' for b in mdata[j:j+8])
                            chk.append((j, x, y, z, _hex))
                
                if chk:
                    print(f"  可能的陀螺仪数据 ({len(chk)} 个):")
                    for j, x, y, z, _hex in chk[:20]:
                        print(f"    @{j:4d}: {_hex}  → (X={x:4d}, Y={y:4d}, Z={z:4d})")


def check_frame_app_data_detailed():
    """
    详细分析单个 MJPEG 帧中 APP 段的原始数据
    特别检查是否有隐藏在 APP0 或非标准段中的数据
    """
    print("\n" + "=" * 60)
    print("  [Step 5] APP 段深层分析")
    print("=" * 60)
    
    # 重新捕获 1 帧到文件
    stream_path = os.path.join(SAVE_DIR, "single_frame.bin")
    subprocess.run(["v4l2-ctl", "-d", "/dev/video0",
                    "--set-fmt-video", "width=640,height=480,pixelformat=MJPG"],
                   capture_output=True, timeout=5)
    
    result = subprocess.run(
        ["v4l2-ctl", "-d", "/dev/video0", "--stream-mmap",
         "--stream-count=1", "--stream-to", stream_path],
        capture_output=True, text=True, timeout=15
    )
    
    with open(stream_path, 'rb') as f:
        data = f.read()
    
    # 找 SOI
    soi = data.find(b'\xff\xd8')
    if soi < 0:
        print("  未找到 JPEG SOI")
        return
    
    jpeg = data[soi:]
    print(f"  JPEG 大小: {len(jpeg)} bytes (SOI @ stream offset {soi})")
    
    # 显示 SOI 之前的数据
    if soi > 0:
        pre_data = data[:soi]
        print(f"  SOI 前数据: {len(pre_data)} bytes")
        print(f"    {' '.join(f'{b:02X}' for b in pre_data[:64])}")
    
    # 详细解析 JPEG 段，显示所有段的完整 hex
    print(f"\n  完整 JPEG 段 dump:")
    i = 0
    seg_num = 0
    while i < len(jpeg) - 1:
        if jpeg[i] != 0xFF:
            i += 1
            continue
        
        marker = jpeg[i+1]
        
        if marker == 0xFF or marker == 0x00:
            i += 1
            continue
        
        if marker == 0xD8:
            print(f"  [{i:6d}] SOI")
            i += 2
            continue
        if marker == 0xD9:
            print(f"  [{i:6d}] EOI")
            i += 2
            continue
        if 0xD0 <= marker <= 0xD7:
            if seg_num < 10:
                print(f"  [{i:6d}] RST{marker-0xD0}")
            i += 2
            seg_num += 1
            continue
        
        if i + 4 > len(jpeg):
            break
        
        seg_len = (jpeg[i+2] << 8) | jpeg[i+3]
        
        if 0xE0 <= marker <= 0xEF:
            name = f"APP{marker-0xE0}"
        elif marker == 0xDB: name = "DQT"
        elif marker == 0xC0: name = "SOF0"
        elif marker == 0xC2: name = "SOF2"
        elif marker == 0xC4: name = "DHT"
        elif marker == 0xDA: name = "SOS"
        elif marker == 0xFE: name = "COM"
        elif marker == 0xDD: name = "DRI"
        else: name = f"MK{marker:02X}"
        
        data_start = i + 4
        data_end = i + 2 + seg_len
        if data_end > len(jpeg):
            break
        seg_data = jpeg[data_start:data_end]
        
        # 对非标准段、APP段、COM段显示完整 hex
        show_full = (name.startswith("APP") or name == "COM" or name.startswith("MK"))
        
        if show_full and len(seg_data) > 0:
            print(f"  [{i:6d}] {name:<6s} len={len(seg_data):5d}:")
            for j in range(0, len(seg_data), 32):
                hex_str = ' '.join(f'{b:02X}' for b in seg_data[j:j+32])
                ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in seg_data[j:j+32])
                print(f"           [{j:4d}] {hex_str}")
                if len(seg_data[j:j+32]) > 16:
                    print(f"                {ascii_str}")
        elif name == "SOS":
            print(f"  [{i:6d}] SOS     len={len(seg_data):5d}  (scan data follows until EOI)")
            # 显示 SOS header 和紧随其后的扫描数据开头
            sos_bytes = ' '.join(f'{b:02X}' for b in seg_data[:16])
            print(f"           [{0:4d}] {sos_bytes}")
            # 也显示紧接 SOS 数据段后面的扫描数据
            after_sos = jpeg[data_end: data_end + 32]
            if after_sos:
                hex2 = ' '.join(f'{b:02X}' for b in after_sos)
                print(f"           [scan] {hex2}")
        
        i += 2 + seg_len


def main():
    print("=" * 70)
    print("  YLX 陀螺仪 V3 - UVC Metadata / Extension Unit 探测")
    print("=" * 70)
    
    check_video_devices()
    capture_raw_stream_with_headers()
    check_uvc_extension_units()
    check_metadata_node()
    check_frame_app_data_detailed()
    
    print(f"\n{'='*70}")
    print(f"  所有数据已保存到: {SAVE_DIR}")
    print(f"{'='*70}")


if __name__ == "__main__":
    main()
