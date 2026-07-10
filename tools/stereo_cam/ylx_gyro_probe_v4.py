#!/usr/bin/env python3
"""
YLX 陀螺仪数据探测 V4
=====================
V3 发现:
  - UVC Extension Unit #4 (guid 63610682-5070-49ab-b8cc-b3855e8d221d) 有 25 个控制项
  - 中断端点 EP 7 IN: 16-byte 数据包, 间隔 8ms
  - /dev/video1 是 UVC Metadata 节点 (UVCH 格式)
  - /dev/video0 的 "metadata" 实际就是 MJPEG 视频流

V4 策略:
  A. 通过 UVC XU ioctl (UVCIOC_CTRL_QUERY) 读取 XU #4 数据
  B. 通过 pyusb 读取中断端点 EP 7 IN
  C. 尝试 /dev/video1 metadata 格式读取
"""
import os, sys, struct, array, fcntl, time, ctypes
import subprocess

SAVE_DIR = os.path.expanduser("~/ylx_gyro_probe_v4")
os.makedirs(SAVE_DIR, exist_ok=True)

# ─── UVC ioctl 常量 ───
_u = lambda s: s.encode('ascii') if isinstance(s, str) else s
UVCIOC_CTRL_QUERY = 0xC0085502  # UVC query control ioctl

# XU #4 GUID
XU4_GUID = bytes([
    0x82, 0x06, 0x61, 0x63, 0x70, 0x50, 0xab, 0x49,
    0xb8, 0xcc, 0xb3, 0x85, 0x5e, 0x8d, 0x22, 0x1d
])

# XU #3 GUID
XU3_GUID = bytes([
    0x2c, 0xf4, 0xc2, 0xd5, 0x08, 0x18, 0x9f, 0x4d,
    0xbe, 0x56, 0x75, 0x3e, 0x27, 0x1c, 0x92, 0x44
])

UVC_GET_CUR = 0x81
UVC_GET_LEN = 0x85
UVC_SET_CUR = 0x01


def find_uvc_device():
    """找到 YLX 摄像头的 /dev/video0 和 USB bus/dev"""
    print("=" * 60)
    print("  [Step A] 定位 UVC 设备")
    print("=" * 60)
    
    # 获取 USB bus:device
    r = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=5)
    bus, dev = None, None
    for line in r.stdout.split('\n'):
        if '1bcf:0b15' in line or 'Sunplus' in line:
            parts = line.strip().split()
            if len(parts) >= 4:
                bus = int(parts[1])
                dev = int(parts[3].rstrip(':'))
            print(f"  设备: {line.strip()}")
            print(f"  Bus={bus}, Device={dev}")
            break
    
    if bus is None:
        print("  错误: 未找到 YLX 摄像头")
        return None, None
    
    # 获取 sysfs 路径
    sysfs_video0 = "/sys/class/video4linux/video0/device"
    if not os.path.exists(sysfs_video0):
        print("  错误: 无 sysfs 路径")
        return (bus, dev), None
    
    return (bus, dev), "/dev/video0"


def probe_xu_controls_ioctl(video_dev="/dev/video0"):
    """
    通过 UVC ioctl 查询 Extension Unit 的控制项
    uvc_xu_control_query 结构体:
      unit, selector, query (UVC_GET_CUR/UVC_GET_LEN)
      size, data pointer
    """
    print(f"\n{'='*60}")
    print(f"  [Step B] XU ioctl 探测 (device={video_dev})")
    print(f"{'='*60}")
    
    fd = os.open(video_dev, os.O_RDWR)
    if fd < 0:
        print(f"  无法打开 {video_dev}")
        return
    
    try:
        # 探测 XU #4 (25 controls)
        for xu_name, xu_guid, num_controls in [("XU4", XU4_GUID, 25), ("XU3", XU3_GUID, 3)]:
            print(f"\n  --- {xu_name} (guid={xu_guid.hex()}, {num_controls} controls) ---")
            
            for selector in range(1, min(num_controls + 1, 32)):
                # Step 1: GET_LEN - 查询数据长度
                try:
                    # struct uvc_xu_control_query:
                    #   __u8 unit     - XU unit ID (4 bytes padded)
                    #   __u8 selector - control selector
                    #   __u8 query    - UVC_GET_LEN=0x85
                    #   __u16 size    - output param
                    #   __u8 *data    - user pointer (8 bytes)
                    # Total: 16 bytes
                    
                    query = struct.pack('BBBH 2x Q', 
                        4,           # unit ID (XU #4)
                        selector,    # control selector
                        UVC_GET_LEN, # query type
                        2,           # size (we expect length in uint16)
                        0            # data pointer (null - just query length)
                    )
                    
                    query_arr = array.array('B', query)
                    fcntl.ioctl(fd, UVCIOC_CTRL_QUERY, query_arr, True)
                    
                    # Read back: size field is at offset 4
                    result = struct.unpack('BBBH 2x Q', query_arr.tobytes())
                    data_len = result[3]
                    
                    if data_len > 0 and data_len <= 4096:
                        # Step 2: GET_CUR - 读取当前值
                        data_buf = ctypes.create_string_buffer(data_len)
                        buf_addr = ctypes.addressof(data_buf)
                        
                        query2 = struct.pack('BBBH 2x Q',
                            4, selector, UVC_GET_CUR, data_len, buf_addr)
                        
                        query_arr2 = array.array('B', query2)
                        fcntl.ioctl(fd, UVCIOC_CTRL_QUERY, query_arr2, True)
                        
                        raw_data = bytes(data_buf[:data_len])
                        hex_str = ' '.join(f'{b:02X}' for b in raw_data[:64])
                        print(f"    Selector {selector:2d}: len={data_len:3d} → {hex_str}")
                        
                        # 检查是否是陀螺仪格式
                        if data_len >= 6:
                            for j in range(0, len(raw_data) - 6, 2):
                                v1 = (raw_data[j] << 8) | raw_data[j+1]
                                v2 = (raw_data[j+2] << 8) | raw_data[j+3]
                                v3 = (raw_data[j+4] << 8) | raw_data[j+5]
                                if (v1 & 0x0F) == 0 and (v2 & 0x0F) == 0 and (v3 & 0x0F) == 0:
                                    x, y, z = v1 >> 4, v2 >> 4, v3 >> 4
                                    if max(x, y, z) > 0 and max(x, y, z) < 4096:
                                        _hex = ' '.join(f'{b:02X}' for b in raw_data[j:j+8])
                                        print(f"      → gyro@+{j}: {_hex} (X={x},Y={y},Z={z})")
                        
                except OSError as e:
                    pass  # 跳过不支持或无效的 selector
                    
    finally:
        os.close(fd)


def probe_interrupt_endpoint(bus, dev):
    """
    通过 pyusb 读取中断端点 EP 7 IN (16 bytes)
    """
    print(f"\n{'='*60}")
    print(f"  [Step C] 中断端点 EP 7 IN 读取")
    print(f"{'='*60}")
    
    try:
        import usb.core
        import usb.util
    except ImportError:
        print("  pyusb 未安装，跳过")
        return
    
    print(f"  定位 USB 设备 bus={bus}, dev={dev}...")
    
    try:
        device = usb.core.find(idVendor=0x1bcf, idProduct=0x0b15)
        if device is None:
            print("  找不到 USB 设备")
            return
        
        print(f"  找到设备: {device}")
        
        # 分离内核驱动以便 pyusb 访问
        try:
            if device.is_kernel_driver_active(0):
                device.detach_kernel_driver(0)
                print("  已分离 kernel driver (interface 0)")
        except Exception as e:
            print(f"  kernel driver detach: {e}")
        
        # 设置为活动配置
        try:
            device.set_configuration()
        except Exception as e:
            print(f"  set_configuration: {e}")
        
        # 列出所有端点
        cfg = device.get_active_configuration()
        for intf in cfg:
            if hasattr(intf, '__iter__'):  # 处理多接口
                for ep in intf:
                    if hasattr(ep, 'bEndpointAddress'):
                        addr = ep.bEndpointAddress
                        ep_type = ep.bmAttributes & 0x03
                        ep_type_name = {0: "Control", 1: "Isochronous", 2: "Bulk", 3: "Interrupt"}.get(ep_type, f"Type{ep_type}")
                        max_pkt = ep.wMaxPacketSize
                        interval = ep.bInterval
                        direction = "IN" if addr & 0x80 else "OUT"
                        print(f"  EP 0x{addr:02X} ({direction}): {ep_type_name}, max={max_pkt}, interval={interval}")
        
        # 读取中断端点 0x87 (EP 7 IN)
        endpoint_addr = 0x87
        
        print(f"\n  尝试读取中断端点 0x{endpoint_addr:02X}...")
        for attempt in range(10):
            try:
                data = device.read(endpoint_addr, 64, timeout=1000)
                hex_str = ' '.join(f'{b:02X}' for b in data[:64])
                print(f"    [{attempt}] {len(data)} bytes: {hex_str}")
                
                # 检查陀螺仪格式
                for j in range(0, len(data) - 6, 2):
                    v1 = (data[j] << 8) | data[j+1]
                    v2 = (data[j+2] << 8) | data[j+3]
                    v3 = (data[j+4] << 8) | data[j+5]
                    if (v1 & 0x0F) == 0 and (v2 & 0x0F) == 0 and (v3 & 0x0F) == 0:
                        x, y, z = v1 >> 4, v2 >> 4, v3 >> 4
                        if max(x, y, z) > 0:
                            _hex = ' '.join(f'{b:02X}' for b in data[j:j+6])
                            print(f"      → gyro@+{j}: {_hex} (X={x},Y={y},Z={z})")
                            
            except usb.core.USBError as e:
                if "timeout" in str(e).lower() or "pipe" in str(e).lower():
                    print(f"    [{attempt}] timeout/pipe error")
                else:
                    print(f"    [{attempt}] USBError: {e}")
                    break
                time.sleep(0.1)
                
    except Exception as e:
        print(f"  中断端点读取失败: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 重新连接内核驱动
        try:
            if device:
                usb.util.dispose_resources(device)
                device.attach_kernel_driver(0)
        except:
            pass


def probe_metadata_node():
    """
    读取 /dev/video1 的 UVC Payload Header Metadata
    需要以 metadata 格式打开（不是 MJPEG）
    """
    print(f"\n{'='*60}")
    print(f"  [Step D] Metadata 节点 (/dev/video1) UVCH 格式")
    print(f"{'='*60}")
    
    if not os.path.exists("/dev/video1"):
        print("  /dev/video1 不存在")
        return
    
    # 设置 metadata 格式
    r = subprocess.run(["v4l2-ctl", "-d", "/dev/video1", "--all"],
                      capture_output=True, text=True, timeout=5)
    print(f"  /dev/video1 当前状态:")
    for line in r.stdout.split('\n')[:20]:
        print(f"    {line.strip()}")
    
    # 尝试用 python v4l2 读取 metadata
    # UVCH fourcc = 'U' 'V' 'C' 'H'
    print(f"\n  尝试用 Python ioctl 读取 metadata...")
    
    fd = os.open("/dev/video1", os.O_RDWR)
    if fd < 0:
        print(f"  无法打开 /dev/video1")
        return
    
    try:
        # 检查当前格式
        VIDIOC_G_FMT = 0xC0D05604
        V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
        
        fmt = struct.pack('I 4x 8x 4x 8x 4x 4I 4x', V4L2_BUF_TYPE_VIDEO_CAPTURE, 0,0,0,0)
        fmt_arr = array.array('B', fmt)
        fcntl.ioctl(fd, VIDIOC_G_FMT, fmt_arr, True)
        _, _, _, _, _, _, w, h, pixelformat, field = struct.unpack(
            'I 4x 8x 4x 8x 4x 4I 4x', fmt_arr.tobytes())
        pf_str = ''.join(chr((pixelformat >> (8*i)) & 0xFF) for i in range(4))
        print(f"  当前格式: {w}x{h}, fourcc='{pf_str}'")
        
    except Exception as e:
        print(f"  格式查询失败: {e}")
    finally:
        os.close(fd)


def dump_xu_via_uvcd_dbg():
    """通过 /sys/kernel/debug 查看 XU 信息"""
    print(f"\n{'='*60}")
    print(f"  [Step E] 内核 debug 信息")
    print(f"{'='*60}")
    
    # 尝试读取 uvcvideo debug
    debug_paths = [
        "/sys/kernel/debug/usb/devices",
        "/sys/kernel/debug/usb/uvcvideo",
    ]
    
    for path in debug_paths:
        if os.path.exists(path):
            if os.path.isdir(path):
                files = os.listdir(path)
                print(f"  {path}/: {files}")
            else:
                with open(path) as f:
                    content = f.read()
                # 过滤出 YLX/Sunplus 相关内容
                for line in content.split('\n'):
                    if any(k in line for k in ['Sunplus', 'YLX', '1bcf', 'uvc', '0b15']):
                        print(f"  {line.strip()[:120]}")


def main():
    print("=" * 70)
    print("  YLX 陀螺仪 V4 - UVC XU ioctl + 中断端点 + Metadata")
    print("=" * 70)
    
    # Step A: 定位设备
    (bus, dev), video_dev = find_uvc_device()
    if bus is None:
        return
    
    # Step B: XU ioctl
    probe_xu_controls_ioctl(video_dev)
    
    # Step C: 中断端点
    probe_interrupt_endpoint(bus, dev)
    
    # Step D: Metadata 节点
    probe_metadata_node()
    
    # Step E: 内核 debug
    dump_xu_via_uvcd_dbg()
    
    print(f"\n{'='*70}")
    print(f"  探测完成")
    print(f"{'='*70}")


if __name__ == "__main__":
    main()
