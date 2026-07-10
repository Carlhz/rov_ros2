#!/usr/bin/env python3
"""
YLX 陀螺仪数据探测 V2
=====================
核心改进: 不使用 cv2.imencode() 重编码，而是直接从 V4L2 读取原始 MJPEG 帧。
摄像头固件将陀螺仪数据嵌入在 MJPEG 帧的 JPEG header 中，
OpenCV 的 decode→encode 流程会丢失这些元数据。

协议: 帧号+三轴陀螺仪，X/Y/Z 各 2 字节，低 4 位无效，取高 12 位

方法:
  A. 优先用 v4l2-ctl 直接捕获原始 MJPEG 流
  B. 备选用 Python ioctl/mmap 做 V4L2 裸读
"""
import os, sys, struct, subprocess, time
from collections import defaultdict

SAVE_DIR = os.path.expanduser("~/ylx_gyro_probe_v2")
os.makedirs(SAVE_DIR, exist_ok=True)


# ─── V4L2 常量 ───
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1
VIDIOC_REQBUFS = 0xC0145608
VIDIOC_QUERYBUF = 0xC0585609
VIDIOC_QBUF = 0xC058560F
VIDIOC_DQBUF = 0xC0585611
VIDIOC_STREAMON = 0x40045612
VIDIOC_STREAMOFF = 0x40045613
VIDIOC_S_FMT = 0xC0D05605
VIDIOC_G_FMT = 0xC0D05604


def try_v4l2_direct(device="/dev/video0", num_frames=5):
    """
    使用 Python ioctl/mmap 直接从 V4L2 读取 MJPEG 帧
    这是最可靠的方式 —— 输出就是摄像头原始输出的字节流
    """
    import fcntl, mmap, ctypes, array

    fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
    if fd < 0:
        raise OSError(f"Cannot open {device}")

    try:
        # 1. 获取当前格式
        fmt = struct.pack('I 4x 8x 4x 8x 4x 4I 4x', V4L2_BUF_TYPE_VIDEO_CAPTURE, 0,0,0,0)
        fmt = array.array('B', fmt)
        fcntl.ioctl(fd, VIDIOC_G_FMT, fmt, True)
        _, _, _, _, _, _, w, h, pixelformat, field = struct.unpack('I 4x 8x 4x 8x 4x 4I 4x', fmt.tobytes())
        print(f"  当前格式: {w}x{h}, pixelformat=0x{pixelformat:08X}")

        # 2. 设置为 MJPEG (如果还不是)
        mjpg_fourcc = ord('M') | (ord('J') << 8) | (ord('P') << 16) | (ord('G') << 24)
        if pixelformat != mjpg_fourcc:
            print(f"  切换为 MJPEG 格式...")
            fmt = struct.pack('I 4x 8x 4x 8x 4x 4I 4x',
                V4L2_BUF_TYPE_VIDEO_CAPTURE, w, h, mjpg_fourcc, field, 0,0,0,0)
            fmt = array.array('B', fmt)
            fcntl.ioctl(fd, VIDIOC_S_FMT, fmt, True)
            _, _, _, _, _, _, w, h, pixelformat, field = struct.unpack(
                'I 4x 8x 4x 8x 4x 4I 4x', fmt.tobytes())
            print(f"  切换后: {w}x{h}, pixelformat=0x{pixelformat:08X}")

        # 3. 请求缓冲区
        req = struct.pack('I I I', V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP, num_frames)
        req = array.array('B', req)
        fcntl.ioctl(fd, VIDIOC_REQBUFS, req, True)

        # 4. 查询缓冲区并 mmap
        buffers = []
        for i in range(num_frames):
            qbuf = struct.pack('I I I 4x I I I', V4L2_BUF_TYPE_VIDEO_CAPTURE, i, V4L2_MEMORY_MMAP, 0,0,0)
            qbuf = array.array('B', qbuf)
            fcntl.ioctl(fd, VIDIOC_QUERYBUF, qbuf, True)
            _, _, _, _, offset, length, _ = struct.unpack('I I I 4x I I I', qbuf.tobytes())
            buf = mmap.mmap(fd, length, offset=offset, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ)
            buffers.append((buf, length))

        # 5. 所有缓冲区入队
        for i in range(num_frames):
            qbuf = struct.pack('I I I 4x I', V4L2_BUF_TYPE_VIDEO_CAPTURE, i, V4L2_MEMORY_MMAP, 0)
            qbuf = array.array('B', qbuf)
            fcntl.ioctl(fd, VIDIOC_QBUF, qbuf, True)

        # 6. 开始流
        fcntl.ioctl(fd, VIDIOC_STREAMON, array.array('B', struct.pack('I', V4L2_BUF_TYPE_VIDEO_CAPTURE)), True)

        # 7. 捕获帧
        frames = []
        for n in range(num_frames):
            dqbuf = struct.pack('I I I 4x I I I', V4L2_BUF_TYPE_VIDEO_CAPTURE, 0, V4L2_MEMORY_MMAP, 0,0,0)
            dqbuf = array.array('B', dqbuf)
            fcntl.ioctl(fd, VIDIOC_DQBUF, dqbuf, True)
            _, index, _, _, bytesused, _, _ = struct.unpack('I I I 4x I I I', dqbuf.tobytes())

            raw = bytes(buffers[index][0][:bytesused])
            
            # 查找 JPEG SOI 标记确定起始
            soi = raw.find(b'\xff\xd8')
            if soi >= 0:
                raw = raw[soi:]
            
            frames.append(raw)
            fpath = os.path.join(SAVE_DIR, f"frame_{n:04d}.jpg")
            with open(fpath, 'wb') as f:
                f.write(raw)
            print(f"  ✓ 帧 {n}: {len(raw)} bytes (bytesused={bytesused}) → {fpath}")

            # 重新入队
            qbuf = struct.pack('I I I 4x I', V4L2_BUF_TYPE_VIDEO_CAPTURE, index, V4L2_MEMORY_MMAP, 0)
            qbuf = array.array('B', qbuf)
            fcntl.ioctl(fd, VIDIOC_QBUF, qbuf, True)

        # 8. 停止流
        fcntl.ioctl(fd, VIDIOC_STREAMOFF, array.array('B', struct.pack('I', V4L2_BUF_TYPE_VIDEO_CAPTURE)), True)

        return frames

    finally:
        os.close(fd)


def try_v4l2ctl(device="/dev/video0", num_frames=5):
    """
    备选方案: 用 v4l2-ctl 捕获原始流, 解析出 MJPEG 帧
    """
    stream_path = os.path.join(SAVE_DIR, "raw_stream.bin")
    
    print(f"  使用 v4l2-ctl 捕获 {num_frames} 帧...")
    
    # 设置 MJPEG 格式
    subprocess.run(["v4l2-ctl", "-d", device, "--set-fmt-video",
                    "width=640,height=480,pixelformat=MJPG"],
                   capture_output=True, timeout=10)
    
    # 捕获原始流
    result = subprocess.run(
        ["v4l2-ctl", "-d", device, "--stream-mmap",
         f"--stream-count={num_frames}", "--stream-to", stream_path],
        capture_output=True, text=True, timeout=30
    )
    print(f"  v4l2-ctl 输出: {result.stderr.strip()}")
    
    if not os.path.exists(stream_path) or os.path.getsize(stream_path) == 0:
        return None
    
    # 从原始流中提取 JPEG 帧 (用 SOI/EOI 分帧)
    with open(stream_path, 'rb') as f:
        raw_stream = f.read()
    
    print(f"  原始流: {len(raw_stream)} bytes")
    
    frames = []
    pos = 0
    while pos < len(raw_stream) - 1:
        soi = raw_stream.find(b'\xff\xd8', pos)
        if soi < 0:
            break
        eoi = raw_stream.find(b'\xff\xd9', soi + 2)
        if eoi < 0:
            break
        frame_data = raw_stream[soi:eoi+2]
        frames.append(frame_data)
        pos = eoi + 2
    
    print(f"  从流中提取到 {len(frames)} 帧")
    for i, fdata in enumerate(frames):
        fpath = os.path.join(SAVE_DIR, f"frame_{i:04d}.jpg")
        with open(fpath, 'wb') as f:
            f.write(fdata)
        print(f"    Frame {i}: {len(fdata)} bytes")
    
    return frames


def analyze_jpeg_segments(data, label=""):
    """
    详细分析 JPEG 所有段的字节内容
    特别关注 APP 段和可能嵌入陀螺仪数据的位置
    """
    print(f"\n{'='*70}")
    print(f"  JPEG 段分析: {label} ({len(data)} bytes)")
    print(f"{'='*70}")
    
    i = 0
    segments = []
    
    while i < len(data) - 1:
        if data[i] != 0xFF:
            i += 1
            continue
        
        marker = data[i+1]
        
        # Padding (0xFF 0xFF) or 0x00 after 0xFF in entropy data
        if marker == 0xFF or marker == 0x00:
            i += 1
            continue
        
        # SOI, EOI, RST markers have no length
        if marker == 0xD8:  # SOI
            segments.append((i, "SOI", 2, b''))
            i += 2
            continue
        if marker == 0xD9:  # EOI
            segments.append((i, "EOI", 2, b''))
            i += 2
            continue
        if 0xD0 <= marker <= 0xD7:  # RSTn
            name = f"RST{marker-0xD0}"
            segments.append((i, name, 2, b''))
            i += 2
            continue
        
        # 其他有长度字段的段
        if i + 4 > len(data):
            break
        
        seg_len = (data[i+2] << 8) | data[i+3]
        
        # 段名
        if 0xE0 <= marker <= 0xEF:
            name = f"APP{marker-0xE0}"
        elif marker == 0xDB:
            name = "DQT"
        elif marker == 0xC0:
            name = "SOF0"
        elif marker == 0xC2:
            name = "SOF2"
        elif marker == 0xC4:
            name = "DHT"
        elif marker == 0xDA:
            name = "SOS"
        elif marker == 0xFE:
            name = "COM"
        elif marker == 0xDD:
            name = "DRI"
        else:
            name = f"0x{marker:02X}"
        
        # 读取段数据 (不包含 marker 和 length 字段本身)
        data_start = i + 4
        data_end = i + 2 + seg_len
        if data_end > len(data):
            break
        seg_data = data[data_start:data_end]
        
        segments.append((i, name, 2 + seg_len, seg_data))
        i += 2 + seg_len
    
    # 打印所有段
    for offset, name, total_len, sdata in segments:
        if name in ("RST0","RST1","RST2","RST3","RST4","RST5","RST6","RST7"):
            if offset < 2000:  # 只打印前 2000 字节的 RST
                print(f"  [{offset:6d}] {name}")
            continue
        
        if name == "SOS":
            print(f"  [{offset:6d}] {name}   len={len(sdata):5d}  (扫描数据起始, 直到 EOI)")
            # 打印 SOS 后第一段数据的 hex
            preview = ' '.join(f'{b:02X}' for b in sdata[:32])
            print(f"            SOS 开头: {preview}")
            continue
        
        data_preview = ' '.join(f'{b:02X}' for b in sdata[:32])
        print(f"  [{offset:6d}] {name:<6s} len={len(sdata):5d}: {data_preview}")
        
        # 对 APP 段做详细分析
        if name.startswith("APP"):
            # 检查是否是陀螺仪数据 (模式: N字节标识 + 帧号2B + X2B + Y2B + Z2B 低4位=0)
            if len(sdata) >= 8:
                # 找段中所有可能的陀螺仪数据位置
                for j in range(0, len(sdata) - 6, 2):
                    v1 = (sdata[j] << 8) | sdata[j+1]
                    v2 = (sdata[j+2] << 8) | sdata[j+3]
                    v3 = (sdata[j+4] << 8) | sdata[j+5]
                    
                    if (v1 & 0x0F) == 0 and (v2 & 0x0F) == 0 and (v3 & 0x0F) == 0:
                        x, y, z = v1 >> 4, v2 >> 4, v3 >> 4
                        if max(x, y, z) > 0 and max(x, y, z) < 4096:
                            hex_str = ' '.join(f'{b:02X}' for b in sdata[j:j+8])
                            print(f"            → 疑似陀螺仪 @段内+{j}: {hex_str}  (X={x:4d}, Y={y:4d}, Z={z:4d})")
    
    return segments


def compare_frames(frames):
    """多帧对比：找出在 APP/header 段中变动的字节"""
    print(f"\n{'='*70}")
    print(f"  多帧对比分析")
    print(f"{'='*70}")
    
    if len(frames) < 2:
        print("  需要至少 2 帧")
        return
    
    # 先分析第一帧找到 SOS 位置
    data0 = frames[0]
    sos_pos = None
    for j in range(len(data0) - 1):
        if data0[j] == 0xFF and data0[j+1] == 0xDA:
            sos_pos = j
            break
    
    header_end = sos_pos if sos_pos else 1000
    print(f"  Header 区域: 0 - {header_end} (SOS @ {sos_pos})")
    
    # 找出 header 区域中在帧间变动的字节
    min_len = min(len(f) for f in frames)
    header_len = min(header_end, min_len)
    
    diff_bytes = []
    for j in range(header_len):
        vals = set(f[j] for f in frames[:min(5, len(frames))])
        if len(vals) > 1:
            diff_bytes.append(j)
    
    print(f"  Header 区域变动字节: {len(diff_bytes)} 个")
    
    if diff_bytes:
        # 按连续范围分组
        groups = []
        start = diff_bytes[0]
        end = diff_bytes[0]
        for p in diff_bytes[1:]:
            if p <= end + 3:
                end = p
            else:
                groups.append((start, end))
                start = end = p
        groups.append((start, end))
        
        print(f"\n  变动区域 ({len(groups)} 组):")
        for s, e in groups:
            length = e - s + 1
            print(f"\n    偏移 {s:5d} - {e:5d} ({length:3d} bytes):")
            for fi in range(min(5, len(frames))):
                hex_str = ' '.join(f'{b:02X}' for b in frames[fi][s:e+1])
                ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in frames[fi][s:e+1])
                print(f"      Frame {fi}: {hex_str}  |{ascii_str}|")
            
            # 尝试解析这段数据为陀螺仪
            if length >= 6:
                for fi in range(min(5, len(frames))):
                    data_seg = frames[fi][s:e+1]
                    for j in range(0, len(data_seg) - 6, 2):
                        v1 = (data_seg[j] << 8) | data_seg[j+1]
                        v2 = (data_seg[j+2] << 8) | data_seg[j+3]
                        v3 = (data_seg[j+4] << 8) | data_seg[j+5]
                        if (v1 & 0x0F) == 0 and (v2 & 0x0F) == 0 and (v3 & 0x0F) == 0:
                            x, y, z = v1 >> 4, v2 >> 4, v3 >> 4
                            _hex = ' '.join(f'{b:02X}' for b in data_seg[j:j+6])
                            print(f"        → Frame{fi} @+{j}: {_hex}  gyro(X={x:4d},Y={y:4d},Z={z:4d})")
    
    # 额外检查：EOS 尾部之前的数据（JPEG 尾部可能也有嵌入数据）
    print(f"\n  检查 JPEG 尾部变动区域:")
    for fi in range(min(5, len(frames))):
        frame = frames[fi]
        # 查找 EOI 之前的最后几个字节
        eoi = frame.rfind(b'\xff\xd9')
        if eoi > 0:
            tail_start = max(0, eoi - 30)
            tail = frame[tail_start:eoi+2]
            hex_str = ' '.join(f'{b:02X}' for b in tail)
            print(f"    Frame {fi} 尾部: {hex_str}")


def main():
    print("=" * 70)
    print("  YLX 陀螺仪 JPEG Header 探测 V2 (Raw V4L2)")
    print("=" * 70)
    
    # 1. 尝试直接 V4L2 读取
    frames = None
    device = "/dev/video0"
    
    print(f"\n[1] 尝试直接从 V4L2 读取 MJPEG 帧 ({device})...")
    try:
        frames = try_v4l2_direct(device, num_frames=5)
        print(f"  ✓ V4L2 direct 成功: {len(frames)} 帧")
    except Exception as e:
        print(f"  ✗ V4L2 direct 失败: {e}")
        print(f"\n[1b] 回退到 v4l2-ctl 方式...")
        try:
            frames = try_v4l2ctl(device, num_frames=5)
            if frames:
                print(f"  ✓ v4l2-ctl 成功: {len(frames)} 帧")
        except Exception as e2:
            print(f"  ✗ v4l2-ctl 也失败了: {e2}")
    
    if not frames:
        print("\n所有方法都失败了")
        return
    
    # 2. 分析第一帧的 JPEG 结构
    print(f"\n[2] 分析 JPEG 段结构 (第一帧)...")
    segments = analyze_jpeg_segments(frames[0], "Frame 0")
    
    # 3. 多帧对比
    print(f"\n[3] 多帧对比找陀螺仪数据...")
    compare_frames(frames)
    
    # 4. 总结
    print(f"\n{'='*70}")
    print(f"  所有帧已保存到: {SAVE_DIR}")
    print(f"  帧文件: frame_0000.jpg ~ frame_{len(frames)-1:04d}.jpg")
    print(f"{'='*70}")


if __name__ == "__main__":
    main()
