#!/usr/bin/env python3
"""
YLX 双目摄像头验证 — 步骤3: 探测陀螺仪/IMU 通道
================================================
YLX Camera (Sunplus 1bcf:0b15)

方法:
  1. UVC Extension Unit (XU) ioctl 扫描
  2. V4L2 metadata 设备扫描
  3. pyusb 描述符解析

运行:
  sudo python3 ylx_imu_probe.py
"""

import os, sys, struct, time, ctypes, fcntl, glob, subprocess

# ============================================================
#  UVC XU ioctl 常量
# ============================================================
UVCIOC_CTRL_QUERY = 0xC0186F21   # struct uvc_xu_control_query

UVC_SET_CUR = 0x01
UVC_GET_CUR = 0x81
UVC_GET_LEN = 0x85
UVC_GET_INFO = 0x86


class UvcXuQuery(ctypes.Structure):
    _fields_ = [
        ('unit',     ctypes.c_uint8),
        ('selector', ctypes.c_uint8),
        ('query',    ctypes.c_uint16),
        ('size',     ctypes.c_uint16),
        ('reserved', ctypes.c_uint16),
        ('data',     ctypes.c_void_p),
    ]


def xu_query(fd, unit, selector, qtype, length=64):
    buf = (ctypes.c_uint8 * length)()
    xu = UvcXuQuery()
    xu.unit = unit
    xu.selector = selector
    xu.query = qtype
    xu.size = length
    xu.reserved = 0
    xu.data = ctypes.cast(buf, ctypes.c_void_p)
    try:
        fcntl.ioctl(fd, UVCIOC_CTRL_QUERY, xu)
        return bytes(buf)
    except Exception:
        return None


def scan_xu(video_dev):
    """暴力扫描所有 unit/selector 组合"""
    print(f"\n  扫描 {video_dev} 的 Extension Unit...")
    try:
        fd = os.open(video_dev, os.O_RDWR | os.O_NONBLOCK)
    except Exception as e:
        print(f"  无法打开 {video_dev}: {e}")
        return []

    results = []
    for unit in range(1, 10):
        for sel in range(1, 12):
            # 先查长度
            r = xu_query(fd, unit, sel, UVC_GET_LEN, 2)
            if r:
                try:
                    ln = struct.unpack("<H", r[:2])[0]
                except Exception:
                    continue
                if 0 < ln < 512:
                    # 有效长度，尝试读值
                    val = xu_query(fd, unit, sel, UVC_GET_CUR, ln)
                    if val and len(val) >= ln:
                        hex_str = ' '.join(f'{b:02X}' for b in val[:min(ln, 32)])
                        print(f"  Unit={unit} Sel={sel} Len={ln}  Data: {hex_str}")
                        results.append((unit, sel, ln, val[:ln]))
    os.close(fd)
    if not results:
        print(f"  {video_dev}: 未发现 XU 控制")
    return results


# ============================================================
#  V4L2 metadata 设备
# ============================================================
def scan_metadata():
    print("\n[Metadata 设备扫描]")
    for dev in sorted(glob.glob("/dev/video*")):
        try:
            out = subprocess.check_output(
                ["v4l2-ctl", "-d", dev, "--info"],
                stderr=subprocess.DEVNULL, timeout=3
            ).decode()
            if any(k in out.lower() for k in ("meta", "imu", "gyro")):
                print(f"  {dev}: METADATA 设备!")
                print("  " + out[:400])
            else:
                # 打印设备名
                for line in out.splitlines():
                    if "Card type" in line or "Driver" in line:
                        print(f"  {dev}: {line.strip()}")
        except Exception:
            pass


# ============================================================
#  lsusb 描述符分析
# ============================================================
def scan_usb_descriptor():
    print("\n[USB 描述符 Extension Unit 分析]")
    try:
        out = subprocess.check_output(
            ["lsusb", "-v", "-d", "1bcf:0b15"],
            stderr=subprocess.STDOUT, timeout=10
        ).decode()
        # 找所有 Extension Unit 段
        lines = out.splitlines()
        in_xu = False
        xu_blocks = []
        cur_block = []
        for line in lines:
            if 'bDescriptorSubtype' in line and '0x06' in line:
                in_xu = True
                cur_block = [line]
            elif in_xu:
                cur_block.append(line)
                if 'bDescriptorSubtype' in line and '0x06' not in line:
                    xu_blocks.append(cur_block)
                    cur_block = []
                    in_xu = False
                elif len(cur_block) > 30:
                    xu_blocks.append(cur_block)
                    cur_block = []
                    in_xu = False

        if cur_block:
            xu_blocks.append(cur_block)

        if xu_blocks:
            print(f"  找到 {len(xu_blocks)} 个 Extension Unit:")
            for i, blk in enumerate(xu_blocks):
                print(f"\n  --- XU {i+1} ---")
                for l in blk[:25]:
                    print(f"  {l}")
        else:
            # 降级: 打印 Interface 相关行
            print("  未找到明确 XU，打印所有接口信息:")
            for line in lines:
                if any(k in line for k in ('Interface', 'bDescriptor', 'bUnit', 'GUID', 'Gyro', 'IMU', 'Extension')):
                    print(f"  {line}")

    except Exception as e:
        print(f"  lsusb 失败: {e} (可能需要 sudo)")


# ============================================================
#  Main
# ============================================================
def main():
    print("=" * 60)
    print("  YLX 双目摄像头 陀螺仪通道探测")
    print("=" * 60)

    if os.geteuid() != 0:
        print("\n⚠  非 root 运行，XU ioctl 可能失败")
        print("   建议: sudo python3 ylx_imu_probe.py\n")

    # USB 描述符
    scan_usb_descriptor()

    # XU 扫描
    print("\n[UVC Extension Unit 暴力扫描]")
    video_devs = sorted(glob.glob("/dev/video*"))
    if not video_devs:
        print("  未找到 /dev/video* 设备")
    else:
        for dev in video_devs[:4]:
            found = scan_xu(dev)
            if found:
                print(f"\n  *** 找到 XU 控制: {dev} ***")
                for unit, sel, ln, data in found:
                    print(f"     Unit={unit} Selector={sel} Length={ln}")
                    # 尝试解析为 IMU 数据
                    if ln >= 12:
                        try:
                            vals = struct.unpack_from('<6h', data, 0)
                            print(f"     作为 int16x6: {vals}  (可能: gx,gy,gz,ax,ay,az)")
                        except Exception:
                            pass
                    if ln >= 24:
                        try:
                            vals = struct.unpack_from('<6f', data, 0)
                            print(f"     作为 float32x6: {[f'{v:.4f}' for v in vals]}")
                        except Exception:
                            pass

    # Metadata
    scan_metadata()

    print("\n" + "=" * 60)
    print("  探测完成！请把输出截图给上位机。")
    print("  如果找到 XU 数据，告知 Unit/Selector/Length")
    print("  将根据实际数据格式编写解析代码。")
    print("=" * 60)


if __name__ == "__main__":
    main()
