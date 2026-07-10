#!/usr/bin/env python3
"""
YLX IMU activation via V4L2 video stream + libusb EP 0x87 read (v2)

Fixed: robust USB bus/device detection using sysfs instead of hardcoded paths.
Strategy:
1. Keep uvcvideo bound (needed for /dev/video*)
2. Open /dev/video0 via V4L2, start streaming
3. Simultaneously read EP 0x87 via libusb (interrupt transfer)
4. Check if video streaming activates IMU data push
5. If not, unbind uvcvideo and retry
"""
import ctypes
import ctypes.util
import struct
import os
import sys
import time
import fcntl
import mmap
import select
import glob

VID, PID = 0x1BCF, 0x0B15
EP_IMU = 0x87
IMU_PKT_SIZE = 8

# ========== V4L2 Constants ==========
_IOC_NRBITS = 8
_IOC_TYPEBITS = 8
_IOC_SIZEBITS = 14
_IOC_DIRBITS = 2
_IOC_NONE = 0
_IOC_WRITE = 1
_IOC_READ = 2

def _IOC(dir, type, nr, size):
    return (dir << (_IOC_NRBITS + _IOC_TYPEBITS + _IOC_SIZEBITS)) | \
           (type << (_IOC_NRBITS + _IOC_SIZEBITS)) | \
           (nr << _IOC_SIZEBITS) | size

def _IOC_TYPECHECK(t):
    return ctypes.sizeof(t)

V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1
V4L2_FIELD_NONE = 1

class V4L2Capability(ctypes.Structure):
    _fields_ = [
        ("driver", ctypes.c_char * 16), ("card", ctypes.c_char * 32),
        ("bus_info", ctypes.c_char * 32), ("version", ctypes.c_uint32),
        ("capabilities", ctypes.c_uint32), ("device_caps", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 3),
    ]

class V4L2Format(ctypes.Structure):
    _fields_ = [
        ("type", ctypes.c_uint32), ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32), ("pixelformat", ctypes.c_uint32),
        ("field", ctypes.c_uint32), ("bytesperline", ctypes.c_uint32),
        ("sizeimage", ctypes.c_uint32), ("colorspace", ctypes.c_uint32),
        ("priv", ctypes.c_uint32), ("flags", ctypes.c_uint32),
        ("ycbcr_enc", ctypes.c_uint32), ("quantization", ctypes.c_uint32),
        ("xfer_func", ctypes.c_uint32),
    ]

class V4L2RequestBuffers(ctypes.Structure):
    _fields_ = [
        ("count", ctypes.c_uint32), ("type", ctypes.c_uint32),
        ("memory", ctypes.c_uint32), ("reserved", ctypes.c_uint32 * 2),
    ]

class TimeVal(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]

class V4L2Buffer(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint32), ("type", ctypes.c_uint32),
        ("bytesused", ctypes.c_uint32), ("flags", ctypes.c_uint32),
        ("field", ctypes.c_uint32), ("timestamp", TimeVal),
        ("timecode", ctypes.c_uint32 * 8), ("sequence", ctypes.c_uint32),
        ("memory", ctypes.c_uint32), ("m_offset", ctypes.c_uint32),
        ("length", ctypes.c_uint32), ("reserved2", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32),
    ]

VIDIOC_QUERYCAP = _IOC(_IOC_READ, ord('V'), 0, ctypes.sizeof(V4L2Capability))
VIDIOC_G_FMT = _IOC(_IOC_READ | _IOC_WRITE, ord('V'), 4, ctypes.sizeof(V4L2Format))
VIDIOC_S_FMT = _IOC(_IOC_READ | _IOC_WRITE, ord('V'), 5, ctypes.sizeof(V4L2Format))
VIDIOC_REQBUFS = _IOC(_IOC_READ | _IOC_WRITE, ord('V'), 8, ctypes.sizeof(V4L2RequestBuffers))
VIDIOC_QUERYBUF = _IOC(_IOC_READ | _IOC_WRITE, ord('V'), 9, ctypes.sizeof(V4L2Buffer))
VIDIOC_QBUF = _IOC(_IOC_READ | _IOC_WRITE, ord('V'), 15, ctypes.sizeof(V4L2Buffer))
VIDIOC_DQBUF = _IOC(_IOC_READ | _IOC_WRITE, ord('V'), 17, ctypes.sizeof(V4L2Buffer))
VIDIOC_STREAMON = _IOC(_IOC_WRITE, ord('V'), 18, ctypes.sizeof(ctypes.c_int))
VIDIOC_STREAMOFF = _IOC(_IOC_WRITE, ord('V'), 19, ctypes.sizeof(ctypes.c_int))


def find_ylx_info():
    """Find YLX USB sysfs path and video device using glob"""
    # Search sysfs for YLX
    for dev_path in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(f"{dev_path}/idVendor") as f:
                v = int(f.read().strip(), 16)
            with open(f"{dev_path}/idProduct") as f:
                p = int(f.read().strip(), 16)
            if v == VID and p == PID:
                usb_id = os.path.basename(dev_path)
                # Find video devices for this USB device
                video_devs = glob.glob(f"{dev_path}/*/video4linux/video*")
                video_paths = []
                for vd in sorted(video_devs, key=lambda x: int(x.split('video')[-1])):
                    vn = os.path.basename(vd)
                    video_paths.append(f"/dev/{vn}")
                # Find interface 0 unbind path
                iface0_path = f"{dev_path}/{usb_id}:1.0"
                unbind_path = f"{iface0_path}/driver/unbind" if os.path.exists(f"{iface0_path}/driver") else None
                bind_path = f"/sys/bus/usb/drivers/uvcvideo/bind"
                return {
                    'usb_id': usb_id,
                    'iface0_path': iface0_path,
                    'unbind_path': unbind_path,
                    'bind_path': bind_path,
                    'video_devs': video_paths,
                }
        except:
            continue
    return None


# ========== libusb ==========
def load_libusb():
    path = ctypes.util.find_library('usb-1.0')
    if not path:
        for p in ['/usr/lib/x86_64-linux-gnu/libusb-1.0.so']:
            if os.path.exists(p): path = p; break
    return ctypes.cdll.LoadLibrary(path)

libusb = load_libusb()

class Desc(ctypes.Structure):
    _fields_ = [
        ("bLength", ctypes.c_uint8), ("bDescriptorType", ctypes.c_uint8),
        ("bcdUSB", ctypes.c_uint16), ("bDeviceClass", ctypes.c_uint8),
        ("bDeviceSubClass", ctypes.c_uint8), ("bDeviceProtocol", ctypes.c_uint8),
        ("bMaxPacketSize0", ctypes.c_uint8), ("idVendor", ctypes.c_uint16),
        ("idProduct", ctypes.c_uint16), ("bcdDevice", ctypes.c_uint16),
        ("iManufacturer", ctypes.c_uint8), ("iProduct", ctypes.c_uint8),
        ("iSerialNumber", ctypes.c_uint8), ("bNumConfigurations", ctypes.c_uint8),
    ]

libusb.libusb_init.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
libusb.libusb_init.restype = ctypes.c_int
libusb.libusb_exit.argtypes = [ctypes.c_void_p]
libusb.libusb_exit.restype = None
libusb.libusb_get_device_list.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_void_p))]
libusb.libusb_get_device_list.restype = ctypes.c_ssize_t
libusb.libusb_free_device_list.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_int]
libusb.libusb_free_device_list.restype = None
libusb.libusb_get_device_descriptor.argtypes = [ctypes.c_void_p, ctypes.POINTER(Desc)]
libusb.libusb_get_device_descriptor.restype = ctypes.c_int
libusb.libusb_open.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p)]
libusb.libusb_open.restype = ctypes.c_int
libusb.libusb_close.argtypes = [ctypes.c_void_p]
libusb.libusb_close.restype = None
libusb.libusb_set_auto_detach_kernel_driver.argtypes = [ctypes.c_void_p, ctypes.c_int]
libusb.libusb_set_auto_detach_kernel_driver.restype = ctypes.c_int
libusb.libusb_interrupt_transfer.argtypes = [ctypes.c_void_p, ctypes.c_uint8, ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_uint]
libusb.libusb_interrupt_transfer.restype = ctypes.c_int


def read_imu_ep(handle, duration=5.0, label="IMU"):
    """Read IMU data from EP 0x87, return count and data points"""
    buf_imu = ctypes.create_string_buffer(64)
    transferred = ctypes.c_int()
    count = 0
    points = []
    start = time.time()
    
    while time.time() - start < duration:
        ret = libusb.libusb_interrupt_transfer(handle, EP_IMU, buf_imu, IMU_PKT_SIZE, ctypes.byref(transferred), 500)
        if ret == 0 and transferred.value >= IMU_PKT_SIZE:
            data = bytes(buf_imu[:transferred.value])
            header = struct.unpack('<H', data[:2])[0]
            x = struct.unpack('<h', data[2:4])[0]
            y = struct.unpack('<h', data[4:6])[0]
            z = struct.unpack('<h', data[6:8])[0]
            count += 1
            if count <= 10 or count % 20 == 0:
                raw_hex = ' '.join(f'{b:02X}' for b in data[:8])
                print(f"  [{label} {count:4d}] H={header:04X} X={x:+6d} Y={y:+6d} Z={z:+6d}  [{raw_hex}]")
            points.append((x, y, z))
        elif ret == -7:
            pass  # timeout
        elif ret != 0 and ret != -7:
            pass  # other errors
    
    elapsed = time.time() - start
    return count, elapsed, points


def main():
    print("=== YLX IMU: V4L2 video stream activation + EP 0x87 read v2 ===\n")
    
    # Step 1: Find YLX info via sysfs
    info = find_ylx_info()
    if not info:
        print("YLX camera not found on USB bus!")
        return
    
    print(f"YLX found: usb_id={info['usb_id']}")
    print(f"  Video devices: {info['video_devs']}")
    print(f"  Interface 0: {info['iface0_path']}")
    
    video_dev = info['video_devs'][0] if info['video_devs'] else None
    if not video_dev:
        print("No video device found for YLX!")
        return
    
    # Step 2: Initialize libusb
    ctx = ctypes.c_void_p()
    libusb.libusb_init(ctypes.byref(ctx))
    libusb_handle = None
    total_imu = 0
    
    try:
        dev_list = ctypes.POINTER(ctypes.c_void_p)()
        count = libusb.libusb_get_device_list(ctx, ctypes.byref(dev_list))
        ylx_dev = None
        for i in range(count):
            desc = Desc()
            libusb.libusb_get_device_descriptor(dev_list[i], ctypes.byref(desc))
            if desc.idVendor == VID and desc.idProduct == PID:
                ylx_dev = dev_list[i]
                break
        
        if not ylx_dev:
            print("YLX not found in libusb device list!")
            return
        
        # ===== Strategy A: V4L2 streaming + libusb auto_detach =====
        print(f"\n{'='*60}")
        print(f"Strategy A: V4L2 stream ON + libusb auto_detach + EP 0x87")
        print(f"{'='*60}")
        
        # Open V4L2
        print(f"\nOpening V4L2: {video_dev}")
        fd = os.open(video_dev, os.O_RDWR)
        
        try:
            cap = V4L2Capability()
            fcntl.ioctl(fd, VIDIOC_QUERYCAP, cap)
            print(f"  Driver: {cap.driver.decode()}  Card: {cap.card.decode()}")
            
            fmt = V4L2Format()
            fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            fcntl.ioctl(fd, VIDIOC_G_FMT, fmt)
            print(f"  Format: {fmt.width}x{fmt.height} pixel=0x{fmt.pixelformat:08X}")
            
            # Request buffers
            req = V4L2RequestBuffers()
            req.count = 4
            req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            req.memory = V4L2_MEMORY_MMAP
            fcntl.ioctl(fd, VIDIOC_REQBUFS, req)
            print(f"  Buffers: {req.count}")
            
            buffers = []
            for i in range(req.count):
                buf = V4L2Buffer()
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
                buf.memory = V4L2_MEMORY_MMAP
                buf.index = i
                fcntl.ioctl(fd, VIDIOC_QUERYBUF, buf)
                mm = mmap.mmap(fd, buf.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=buf.m_offset)
                buffers.append((buf, mm))
                fcntl.ioctl(fd, VIDIOC_QBUF, buf)
            
            # Start stream
            stream_type = ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(fd, VIDIOC_STREAMON, stream_type)
            print("  Stream ON")
            
            # Open libusb with auto_detach
            handle = ctypes.c_void_p()
            ret = libusb.libusb_open(ylx_dev, ctypes.byref(handle))
            if ret != 0:
                print(f"  libusb_open failed: {ret}")
            else:
                libusb_handle = handle
                ret = libusb.libusb_set_auto_detach_kernel_driver(handle, 1)
                print(f"  auto_detach(1): {ret}")
                
                # Read IMU while keeping V4L2 stream alive
                print("\n  Reading EP 0x87 (5s, V4L2 streaming)...")
                
                buf_imu = ctypes.create_string_buffer(64)
                transferred = ctypes.c_int()
                imu_a = 0
                start = time.time()
                
                while time.time() - start < 5.0:
                    ret = libusb.libusb_interrupt_transfer(handle, EP_IMU, buf_imu, IMU_PKT_SIZE, ctypes.byref(transferred), 200)
                    if ret == 0 and transferred.value >= IMU_PKT_SIZE:
                        data = bytes(buf_imu[:transferred.value])
                        x = struct.unpack('<h', data[2:4])[0]
                        y = struct.unpack('<h', data[4:6])[0]
                        z = struct.unpack('<h', data[6:8])[0]
                        imu_a += 1
                        if imu_a <= 5:
                            print(f"  A[{imu_a}] X={x:+6d} Y={y:+6d} Z={z:+6d}")
                    
                    # DQBUF to keep stream alive
                    try:
                        r, _, _ = select.select([fd], [], [], 0.05)
                        if r:
                            dbuf = V4L2Buffer()
                            dbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
                            dbuf.memory = V4L2_MEMORY_MMAP
                            fcntl.ioctl(fd, VIDIOC_DQBUF, dbuf)
                            fcntl.ioctl(fd, VIDIOC_QBUF, dbuf)
                    except:
                        pass
                
                elapsed = time.time() - start
                if imu_a > 0:
                    print(f"  Strategy A: {imu_a} pkts in {elapsed:.1f}s ({imu_a/elapsed:.1f} Hz) *** SUCCESS ***")
                    total_imu = imu_a
                else:
                    print(f"  Strategy A: 0 pkts in {elapsed:.1f}s")
            
            # Stop V4L2
            fcntl.ioctl(fd, VIDIOC_STREAMOFF, stream_type)
            print("  Stream OFF")
        finally:
            os.close(fd)
        
        # ===== Strategy B: Unbind uvcvideo + libusb direct =====
        if total_imu == 0:
            print(f"\n{'='*60}")
            print(f"Strategy B: Unbind uvcvideo + libusb direct + EP 0x87")
            print(f"{'='*60}")
            
            # Close previous libusb handle
            if libusb_handle:
                libusb.libusb_close(libusb_handle)
                libusb_handle = None
            
            # Unbind
            iface0 = info['iface0_path']
            usb_id = info['usb_id']
            unbind_file = f"{iface0}/driver/unbind"
            print(f"  Unbinding {usb_id}:1.0 from {unbind_file}...")
            try:
                with open(unbind_file, "w") as f:
                    f.write(f"{usb_id}:1.0")
                print("  Unbound")
            except Exception as e:
                print(f"  Unbind error: {e}")
            
            time.sleep(0.5)
            
            # Re-open libusb
            handle2 = ctypes.c_void_p()
            ret = libusb.libusb_open(ylx_dev, ctypes.byref(handle2))
            if ret != 0:
                print(f"  libusb_open error: {ret}")
            else:
                libusb_handle = handle2
                
                print("  Reading EP 0x87 after unbind (5s)...")
                count_b, elapsed_b, _ = read_imu_ep(handle2, 5.0, "B")
                
                if count_b > 0:
                    print(f"  Strategy B: {count_b} pkts in {elapsed_b:.1f}s ({count_b/elapsed_b:.1f} Hz) *** SUCCESS ***")
                    total_imu = count_b
                else:
                    print("  Strategy B: 0 pkts")
            
            # Rebind
            print("\n  Rebinding uvcvideo...")
            try:
                with open("/sys/bus/usb/drivers/uvcvideo/bind", "w") as f:
                    f.write(f"{usb_id}:1.0")
                print("  Rebound")
            except Exception as e:
                print(f"  Rebind error: {e}")
        
    finally:
        if libusb_handle:
            libusb.libusb_close(libusb_handle)
        libusb.libusb_free_device_list(dev_list, 1)
        libusb.libusb_exit(ctx)
    
    print(f"\n{'='*60}")
    if total_imu > 0:
        print(f"SUCCESS! IMU data received: {total_imu} packets")
    else:
        print("FAILED: No IMU data from EP 0x87 in any strategy")
        print("\nPossible causes:")
        print("  1. IMU only activates when firmware sends specific init command")
        print("  2. IMU needs different endpoint (check Windows USBPcap for EP number)")
        print("  3. IMU only works on Windows driver stack")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()
