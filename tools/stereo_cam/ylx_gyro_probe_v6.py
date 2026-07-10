#!/usr/bin/env python3
"""
YLX 陀螺仪 V6 — /dev/input/event5 读取
======================================
V5 发现: YLX 摄像头注册了 /dev/input/event5 (usb-YLX-...-event-if00)
这很可能就是 IMU/陀螺仪数据的输入路径。

还发现:
  - /dev/hidraw0 可能也是同一设备
  - /dev/video1 是 Metadata Capture (UVCH) 节点
  
策略:
  1. 用 evdev 读取 /dev/input/event5 的事件
  2. 如果不行, 尝试 hidraw
  3. 最后尝试 /dev/video1 metadata
"""
import struct, os, time, fcntl, array

# evdev 常量
EVIOCGVERSION = 0x80044501
EVIOCGID = 0x80084502
EVIOCGNAME = lambda l: 0x80004506 | (l << 16)  # len=256
EVIOCGBIT = lambda ev, l: 0x80404520 | ((ev) << 8) | (l << 16)

# 事件类型
EV_SYN = 0x00
EV_KEY = 0x01
EV_REL = 0x02
EV_ABS = 0x03
EV_MSC = 0x04

# ABS 轴
ABS_X = 0x00
ABS_Y = 0x01
ABS_Z = 0x02
ABS_RX = 0x03
ABS_RY = 0x04
ABS_RZ = 0x05
ABS_MISC = 0x28

ABS_NAMES = {
    0x00: 'ABS_X', 0x01: 'ABS_Y', 0x02: 'ABS_Z',
    0x03: 'ABS_RX', 0x04: 'ABS_RY', 0x05: 'ABS_RZ',
    0x06: 'ABS_THROTTLE', 0x07: 'ABS_RUDDER',
    0x08: 'ABS_WHEEL', 0x09: 'ABS_GAS', 0x0A: 'ABS_BRAKE',
    0x10: 'ABS_HAT0X', 0x11: 'ABS_HAT0Y',
    0x12: 'ABS_HAT1X', 0x13: 'ABS_HAT1Y',
    0x14: 'ABS_HAT2X', 0x15: 'ABS_HAT2Y',
    0x16: 'ABS_HAT3X', 0x17: 'ABS_HAT3Y',
    0x18: 'ABS_PRESSURE', 0x19: 'ABS_DISTANCE',
    0x1A: 'ABS_TILT_X', 0x1B: 'ABS_TILT_Y',
    0x1C: 'ABS_TOOL_WIDTH', 0x20: 'ABS_VOLUME',
    0x28: 'ABS_MISC',
}

REL_NAMES = {
    0x00: 'REL_X', 0x01: 'REL_Y', 0x02: 'REL_Z',
    0x03: 'REL_RX', 0x04: 'REL_RY', 0x05: 'REL_RZ',
    0x06: 'REL_HWHEEL', 0x07: 'REL_DIAL',
    0x08: 'REL_WHEEL', 0x09: 'REL_MISC',
}


def get_device_info(fd):
    """获取输入设备信息"""
    # 设备名称
    name_buf = array.array('B', b'\x00' * 256)
    fcntl.ioctl(fd, EVIOCGNAME(256), name_buf, True)
    name = name_buf.tobytes().split(b'\x00')[0].decode('utf-8', errors='replace')
    
    # 设备 ID
    id_buf = array.array('B', b'\x00' * 8)
    fcntl.ioctl(fd, EVIOCGID, id_buf, True)
    bus, vendor, product, version = struct.unpack('HHHH', id_buf.tobytes())
    
    # 支持的事件类型
    ev_bits = array.array('B', b'\x00' * 32)
    fcntl.ioctl(fd, EVIOCGBIT(0, 32), ev_bits, True)  # EV_MAX
    supported_ev = []
    for i in range(32):
        byte_idx = i // 8
        bit_idx = i % 8
        if ev_bits[byte_idx] & (1 << bit_idx):
            supported_ev.append(i)
    
    return {
        'name': name, 'bus': bus, 'vendor': vendor,
        'product': product, 'version': version,
        'supported_ev': supported_ev
    }


def get_abs_info(fd, axis):
    """获取 ABS 轴信息 (min, max, resolution, etc.)"""
    if not hasattr(get_abs_info, '_cache'):
        get_abs_info._cache = {}
    
    if axis in get_abs_info._cache:
        return get_abs_info._cache[axis]
    
    EVIOCGABS = lambda a: 0x80084540 | (a << 16)
    try:
        abs_buf = array.array('B', b'\x00' * 24)
        fcntl.ioctl(fd, EVIOCGABS(axis), abs_buf, True)
        value, minimum, maximum, fuzz, flat, resolution = struct.unpack('iiiiii', abs_buf.tobytes())
        info = {'value': value, 'min': minimum, 'max': maximum,
                'fuzz': fuzz, 'flat': flat, 'resolution': resolution}
        get_abs_info._cache[axis] = info
        return info
    except OSError:
        return None


def probe_event5():
    """读取 /dev/input/event5 的事件"""
    dev_path = "/dev/input/event5"
    
    if not os.path.exists(dev_path):
        print(f"  {dev_path} 不存在")
        return
    
    print("=" * 60)
    print(f"  [Step 1] /dev/input/event5 探测")
    print("=" * 60)
    
    fd = os.open(dev_path, os.O_RDONLY)
    try:
        info = get_device_info(fd)
        print(f"  设备名: {info['name']}")
        print(f"  Bus: 0x{info['bus']:04X} Vendor: 0x{info['vendor']:04X} "
              f"Product: 0x{info['product']:04X} Version: 0x{info['version']:04X}")
        print(f"  支持的事件类型: {info['supported_ev']}")
        
        ev_type_names = {0: 'EV_SYN', 1: 'EV_KEY', 2: 'EV_REL', 3: 'EV_ABS', 4: 'EV_MSC'}
        
        # 检查 ABS 轴
        abs_axes = []
        if EV_ABS in info['supported_ev']:
            for axis in range(0x40):
                abs_info = get_abs_info(fd, axis)
                if abs_info:
                    name = ABS_NAMES.get(axis, f'0x{axis:02X}')
                    print(f"  ABS {name}: min={abs_info['min']}, max={abs_info['max']}, "
                          f"res={abs_info['resolution']}, fuzz={abs_info['fuzz']}")
                    abs_axes.append((axis, name, abs_info))
        
        # 检查 REL 轴
        rel_bit = array.array('B', b'\x00' * 32)
        fcntl.ioctl(fd, EVIOCGBIT(EV_REL, 32), rel_bit, True)
        rel_axes = []
        for i in range(32):
            byte_idx = i // 8
            bit_idx = i % 8
            if rel_bit[byte_idx] & (1 << bit_idx):
                name = REL_NAMES.get(i, f'0x{i:02X}')
                print(f"  REL {name}")
                rel_axes.append(i)
        
        print(f"\n  读取 60 个事件 (3 秒)...")
        print(f"  {'timestamp':>12s}  type    code    value")
        print(f"  {'─'*50}")
        
        import select
        count = 0
        start = time.time()
        
        while count < 60 and time.time() - start < 5:
            r, _, _ = select.select([fd], [], [], 0.5)
            if not r:
                continue
            
            try:
                raw = os.read(fd, 24)
                if len(raw) < 16:
                    continue
                
                # struct input_event: timeval (8) + type (2) + code (2) + value (4) = 16
                tv_sec, tv_usec, ev_type, ev_code, ev_value = struct.unpack('LLHHi', raw[:16])
                
                type_name = ev_type_names.get(ev_type, f'0x{ev_type:02X}')
                
                if ev_type == EV_ABS:
                    code_name = ABS_NAMES.get(ev_code, f'0x{ev_code:02X}')
                elif ev_type == EV_REL:
                    code_name = REL_NAMES.get(ev_code, f'0x{ev_code:02X}')
                elif ev_type == EV_SYN:
                    code_name = 'SYN_REPORT' if ev_code == 0 else f'{ev_code}'
                else:
                    code_name = f'{ev_code}'
                
                ts = f'{tv_sec}.{tv_usec:06d}'
                print(f"  {ts}  {type_name:<8s} {code_name:<10s} {ev_value}")
                count += 1
                
            except Exception as e:
                print(f"  read error: {e}")
                break
        
        print(f"  读取了 {count} 个事件")
        
    finally:
        os.close(fd)


def probe_hidraw():
    """读取 /dev/hidraw0 的原始 HID 数据"""
    dev_path = "/dev/hidraw0"
    
    if not os.path.exists(dev_path):
        print(f"  /dev/hidraw0 不存在")
        return
    
    print(f"\n{'='*60}")
    print(f"  [Step 2] /dev/hidraw0 探测")
    print(f"{'='*60}")
    
    # 获取设备描述符
    import subprocess
    r = subprocess.run(["sudo", "cat", "/sys/class/hidraw/hidraw0/device/uevent"],
                      capture_output=True, text=True, timeout=5)
    print(f"  uevent:")
    for line in r.stdout.strip().split('\n'):
        print(f"    {line}")
    
    r2 = subprocess.run(["sudo", "cat", "/sys/class/hidraw/hidraw0/device/report_descriptor"],
                       capture_output=True, timeout=5)
    raw_desc = r2.stdout
    if raw_desc:
        print(f"  Report descriptor ({len(raw_desc)} bytes):")
        print(f"    {' '.join(f'{b:02X}' for b in raw_desc[:64])}")
    
    # 尝试读取原始 HID 数据
    print(f"\n  尝试读取 HID raw 数据...")
    try:
        fd = os.open(dev_path, os.O_RDONLY | os.O_NONBLOCK)
        import select
        for i in range(5):
            r, _, _ = select.select([fd], [], [], 0.5)
            if r:
                data = os.read(fd, 64)
                hex_str = ' '.join(f'{b:02X}' for b in data)
                print(f"  [{i}] {len(data)} bytes: {hex_str}")
                # gyro check
                for j in range(0, len(data) - 6, 2):
                    v1 = (data[j] << 8) | data[j+1]
                    v2 = (data[j+2] << 8) | data[j+3]
                    v3 = (data[j+4] << 8) | data[j+5]
                    if (v1 & 0x0F) == 0 and (v2 & 0x0F) == 0 and (v3 & 0x0F) == 0:
                        x, y, z = v1 >> 4, v2 >> 4, v3 >> 4
                        if max(x, y, z) > 0:
                            print(f"    → GYRO: X={x:4d} Y={y:4d} Z={z:4d}")
            else:
                print(f"  [{i}] timeout")
        os.close(fd)
    except Exception as e:
        print(f"  HID read error: {e}")


def main():
    print("=" * 70)
    print("  YLX 陀螺仪 V6 — /dev/input/event5 + hidraw")
    print("=" * 70)
    
    probe_event5()
    probe_hidraw()
    
    print(f"\n{'='*70}")
    print(f"  探测完成")
    print(f"{'='*70}")


if __name__ == "__main__":
    main()
