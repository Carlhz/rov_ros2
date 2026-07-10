#!/usr/bin/env python3
"""
YLX 陀螺仪 V5 — XU ioctl 深度探测 (无 pyusb)
=============================================
策略: 仅通过 UVCIOC_CTRL_QUERY ioctl 读取 Extension Units #3 和 #4
      无需分离内核驱动，直接通过 /dev/video0 的 uvcvideo 驱动通信
"""
import os, struct, array, fcntl, ctypes

UVCIOC_CTRL_QUERY = 0xC0085502
UVC_GET_CUR = 0x81
UVC_GET_LEN = 0x85
UVC_GET_MIN = 0x82
UVC_GET_MAX = 0x83
UVC_GET_DEF = 0x87

XU4_GUID = bytes.fromhex('820661637050ab49b8ccb3855e8d221d')
XU3_GUID = bytes.fromhex('2cf4c2d508189f4dbe56753e271c9244')

def xu_query(video_dev, unit_id, selector, query_type, data_size=0):
    """
    发送 UVC XU 控制查询
    返回 (data_len, raw_data_bytes)
    """
    fd = os.open(video_dev, os.O_RDWR)
    try:
        if query_type in (UVC_GET_LEN,):
            # 仅查询长度: data pointer = null
            packed = struct.pack('BBBH 2x Q',
                unit_id, selector, query_type, 2, 0)
            arr = array.array('B', packed)
            fcntl.ioctl(fd, UVCIOC_CTRL_QUERY, arr, True)
            result = struct.unpack('BBBH 2x Q', arr.tobytes())
            return result[3], None
        
        # 查询数据: 需要分配缓冲区并传指针
        buf = ctypes.create_string_buffer(max(data_size, 1))
        buf_addr = ctypes.addressof(buf)
        
        packed = struct.pack('BBBH 2x Q',
            unit_id, selector, query_type, data_size, buf_addr)
        arr = array.array('B', packed)
        fcntl.ioctl(fd, UVCIOC_CTRL_QUERY, arr, True)
        
        result = struct.unpack('BBBH 2x Q', arr.tobytes())
        actual_len = result[3]
        return actual_len, bytes(buf[:actual_len])
    finally:
        os.close(fd)
    return 0, None


def probe_xu(video_dev, name, unit_id, guid, num_controls):
    """探测一个 Extension Unit 的所有控制项"""
    print(f"\n{'='*60}")
    print(f"  {name} (unit={unit_id}, guid={guid.hex()})")
    print(f"  bNumControl={num_controls}")
    print(f"{'='*60}")
    
    found_any = False
    
    for selector in range(1, 33):
        try:
            # GET_LEN
            data_len, _ = xu_query(video_dev, unit_id, selector, UVC_GET_LEN)
            
            if data_len <= 0 or data_len > 65535:
                continue
            
            # 也获取 MIN/MAX/DEF
            results = {}
            for qtype, qname in [(UVC_GET_CUR, 'CUR'), (UVC_GET_MIN, 'MIN'),
                                  (UVC_GET_MAX, 'MAX'), (UVC_GET_DEF, 'DEF')]:
                try:
                    _, data = xu_query(video_dev, unit_id, selector, qtype, data_len)
                    results[qname] = data
                except OSError:
                    pass
            
            # 主要用 CUR 和 DEF
            cur_data = results.get('CUR', b'')
            if cur_data:
                found_any = True
                hex_cur = ' '.join(f'{b:02X}' for b in cur_data[:64])
                print(f"  Selector {selector:2d}: len={data_len:3d} CUR={hex_cur}")
                
                if len(cur_data) > 64:
                    print(f"           ... +{len(cur_data)-64} more bytes")
                
                # 检查陀螺仪格式
                for j in range(0, len(cur_data) - 6, 2):
                    v1 = (cur_data[j] << 8) | cur_data[j+1]
                    v2 = (cur_data[j+2] << 8) | cur_data[j+3]
                    v3 = (cur_data[j+4] << 8) | cur_data[j+5]
                    if (v1 & 0x0F) == 0 and (v2 & 0x0F) == 0 and (v3 & 0x0F) == 0:
                        x, y, z = v1 >> 4, v2 >> 4, v3 >> 4
                        if max(x, y, z) > 0 and max(x, y, z) < 4096:
                            _hex = ' '.join(f'{b:02X}' for b in cur_data[j:j+8])
                            print(f"      >>> GYRO@+{j}: {_hex} (X={x:4d}, Y={y:4d}, Z={z:4d})")
                    
        except OSError as e:
            # 跳过不可用的 selector
            pass
    
    if not found_any:
        print("  (无有效 selector)")


def probe_xu_get_res():
    """用 UVC_GET_RES 查询 (0x84) 来发现所有有效 selector"""
    print(f"\n{'='*60}")
    print(f"  XU Selector 扫描 (GET_RES 方法)")
    print(f"{'='*60}")
    
    import subprocess
    
    # 尝试通过 uvc_dbg 或 sysfs 获取
    paths = [
        "/sys/kernel/debug/usb/uvcvideo", 
        "/sys/class/video4linux/video0/device/uvc/video0"
    ]
    
    for p in paths:
        if os.path.exists(p):
            print(f"  {p}/ 存在")
            try:
                for f in os.listdir(p):
                    print(f"    {f}")
            except:
                pass
    
    # 尝试读取 /sys/class/video4linux/video0/device/下的 uvc 相关
    dev_root = "/sys/class/video4linux/video0/device"
    if os.path.exists(f"{dev_root}/bNumConfigurations"):
        out = subprocess.run(["lsusb", "-v", "-d", "1bcf:0b15"],
                           capture_output=True, text=True, timeout=5)
        for line in out.stdout.split('\n'):
            if 'EXTENSION' in line or 'bUnitID' in line or 'bmControls' in line:
                print(f"  {line.strip()}")


def continuous_read(video_dev, unit_id, selector, count=5, interval=0.5):
    """连续读取一个 XU 控制项，看值如何变化"""
    print(f"\n{'='*60}")
    print(f"  连续读取 XU{unit_id} selector={selector}")
    print(f"{'='*60}")
    
    import time
    
    # 先获取长度
    try:
        data_len, _ = xu_query(video_dev, unit_id, selector, UVC_GET_LEN)
        print(f"  data_len = {data_len}")
    except OSError as e:
        print(f"  GET_LEN 失败: {e}")
        return
    
    for i in range(count):
        try:
            _, data = xu_query(video_dev, unit_id, selector, UVC_GET_CUR, data_len)
            if data:
                hex_str = ' '.join(f'{b:02X}' for b in data[:32])
                print(f"  [{i}] {hex_str}")
                
                # gyro check
                for j in range(0, len(data) - 6, 2):
                    v1 = (data[j] << 8) | data[j+1]
                    v2 = (data[j+2] << 8) | data[j+3]
                    v3 = (data[j+4] << 8) | data[j+5]
                    if (v1 & 0x0F) == 0 and (v2 & 0x0F) == 0 and (v3 & 0x0F) == 0:
                        x, y, z = v1 >> 4, v2 >> 4, v3 >> 4
                        if max(x, y, z) > 0 and max(x, y, z) < 4096:
                            print(f"    → GYRO: X={x:4d} Y={y:4d} Z={z:4d}")
        except OSError as e:
            print(f"  [{i}] ERROR: {e}")
        
        time.sleep(interval)


def main():
    print("=" * 70)
    print("  YLX 陀螺仪 V5 — XU ioctl 探测")
    print("=" * 70)
    
    video_dev = "/dev/video0"
    
    # XU #4 (IMU 相关 - 25 controls)
    probe_xu(video_dev, "XU4 (IMU?)", 4, XU4_GUID, 25)
    
    # XU #3 (3 controls)
    probe_xu(video_dev, "XU3", 3, XU3_GUID, 3)
    
    # 扫描方法
    probe_xu_get_res()
    
    # 如果找到有效 selector，连续读取检查值变化
    # (需要先运行一次手动确认)
    
    print(f"\n{'='*70}")
    print(f"  探测完成")
    print(f"  (如需连续读取某个 selector, 运行: continuous_read 函数)")
    print(f"{'='*70}")


if __name__ == "__main__":
    main()
