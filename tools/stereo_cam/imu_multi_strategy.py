#!/usr/bin/env python3
"""
YLX IMU: Try multiple strategies to read EP 0x87

Strategy:
1. Start v4l2 background streaming via v4l2-ctl
2. Try libusb auto_detach + EP 0x87 (with V4L2 active)
3. Unbind uvcvideo + libusb direct + EP 0x87
4. Rebind uvcvideo

All libusb via ctypes (no pyusb needed)
"""
import ctypes
import ctypes.util
import struct
import os
import sys
import time
import subprocess
import signal

VID, PID = 0x1BCF, 0x0B15
EP_IMU = 0x87
IMU_PKT_SIZE = 8

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
libusb.libusb_claim_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
libusb.libusb_claim_interface.restype = ctypes.c_int
libusb.libusb_release_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
libusb.libusb_release_interface.restype = ctypes.c_int
libusb.libusb_interrupt_transfer.argtypes = [ctypes.c_void_p, ctypes.c_uint8, ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_uint]
libusb.libusb_interrupt_transfer.restype = ctypes.c_int


def find_ylx_info():
    """Find YLX USB sysfs path"""
    import glob
    for dev_path in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(f"{dev_path}/idVendor") as f:
                v = int(f.read().strip(), 16)
            with open(f"{dev_path}/idProduct") as f:
                p = int(f.read().strip(), 16)
            if v == VID and p == PID:
                usb_id = os.path.basename(dev_path)
                iface0 = f"{dev_path}/{usb_id}:1.0"
                unbind = f"{iface0}/driver/unbind" if os.path.exists(f"{iface0}/driver") else None
                return {'usb_id': usb_id, 'iface0': iface0, 'unbind': unbind}
        except:
            continue
    return None


def read_imu(handle, duration=3.0, label="", print_all=False):
    """Read IMU from EP 0x87"""
    buf = ctypes.create_string_buffer(64)
    transferred = ctypes.c_int()
    count = 0
    start = time.time()
    
    while time.time() - start < duration:
        ret = libusb.libusb_interrupt_transfer(handle, EP_IMU, buf, IMU_PKT_SIZE, ctypes.byref(transferred), 200)
        if ret == 0 and transferred.value >= IMU_PKT_SIZE:
            data = bytes(buf[:transferred.value])
            header = struct.unpack('<H', data[:2])[0]
            x = struct.unpack('<h', data[2:4])[0]
            y = struct.unpack('<h', data[4:6])[0]
            z = struct.unpack('<h', data[6:8])[0]
            count += 1
            if count <= 10 or (print_all and count % 10 == 0):
                raw = ' '.join(f'{b:02X}' for b in data[:8])
                print(f"  [{label}{count:4d}] H={header:04X} X={x:+6d} Y={y:+6d} Z={z:+6d}  [{raw}]")
        elif ret == -7:
            pass
        elif ret == -99:
            print(f"  [{label}] err=-99 (OTHER/IO)")
            break
    
    elapsed = time.time() - start
    if count > 0:
        print(f"  [{label}] {count} pkts in {elapsed:.1f}s ({count/elapsed:.1f} Hz)")
    return count


def main():
    print("=== YLX IMU EP 0x87 multi-strategy test ===\n")
    
    info = find_ylx_info()
    if not info:
        print("YLX not found!")
        return
    print(f"YLX: usb_id={info['usb_id']}, iface0={info['iface0']}")
    
    # Init libusb
    ctx = ctypes.c_void_p()
    libusb.libusb_init(ctypes.byref(ctx))
    libusb_handle = None
    v4l2_proc = None
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
            print("YLX not in libusb list!")
            return
        
        # ===== Strategy A: V4L2 streaming (v4l2-ctl) + libusb auto_detach =====
        print(f"\n{'='*60}")
        print("Strategy A: v4l2-ctl stream + libusb auto_detach + EP 0x87")
        print(f"{'='*60}")
        
        # Start v4l2-ctl background stream
        print("Starting v4l2-ctl --stream-mmap in background...")
        v4l2_proc = subprocess.Popen(
            ["v4l2-ctl", "-d", "/dev/video0", "--stream-mmap", "--stream-count=300", "--stream-to=/dev/null"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        time.sleep(0.5)
        print(f"  v4l2-ctl PID: {v4l2_proc.pid}, running: {v4l2_proc.poll() is None}")
        
        # Open libusb with auto_detach
        handle = ctypes.c_void_p()
        ret = libusb.libusb_open(ylx_dev, ctypes.byref(handle))
        if ret != 0:
            print(f"  libusb_open error: {ret}")
        else:
            libusb_handle = handle
            ret = libusb.libusb_set_auto_detach_kernel_driver(handle, 1)
            print(f"  auto_detach(1): {ret}")
            
            if ret == 0:
                # Claim interface 0
                ret = libusb.libusb_claim_interface(handle, 0)
                print(f"  claim_interface(0): {ret}")
                
                # Read IMU
                print("\n  Reading EP 0x87...")
                imu_a = read_imu(handle, 3.0, "A")
                if imu_a > 0:
                    total_imu = imu_a
        
        # Stop v4l2
        if v4l2_proc and v4l2_proc.poll() is None:
            v4l2_proc.terminate()
            try: v4l2_proc.wait(timeout=2)
            except: v4l2_proc.kill()
            print("  v4l2-ctl stopped")
        
        # Close libusb
        if libusb_handle:
            libusb.libusb_release_interface(libusb_handle, 0)
            libusb.libusb_close(libusb_handle)
            libusb_handle = None
        
        # ===== Strategy B: Unbind + libusb direct =====
        if total_imu == 0:
            print(f"\n{'='*60}")
            print("Strategy B: Unbind uvcvideo + libusb direct + EP 0x87")
            print(f"{'='*60}")
            
            # Unbind
            usb_id = info['usb_id']
            unbind_file = info['unbind']
            if unbind_file:
                print(f"  Unbinding {usb_id}:1.0...")
                try:
                    with open(unbind_file, "w") as f:
                        f.write(f"{usb_id}:1.0")
                    print("  Unbound")
                except Exception as e:
                    print(f"  Error: {e}")
            
            time.sleep(0.5)
            
            # Open and claim
            handle2 = ctypes.c_void_p()
            ret = libusb.libusb_open(ylx_dev, ctypes.byref(handle2))
            if ret != 0:
                print(f"  libusb_open error: {ret}")
            else:
                libusb_handle = handle2
                ret = libusb.libusb_claim_interface(handle2, 0)
                print(f"  claim_interface(0): {ret}")
                
                if ret == 0:
                    print("\n  Reading EP 0x87 (5s)...")
                    imu_b = read_imu(handle2, 5.0, "B", print_all=True)
                    if imu_b > 0:
                        total_imu = imu_b
            
            # Rebind
            print("\n  Rebinding uvcvideo...")
            try:
                with open("/sys/bus/usb/drivers/uvcvideo/bind", "w") as f:
                    f.write(f"{usb_id}:1.0")
                print("  Rebound")
            except Exception as e:
                print(f"  Error: {e}")
        
    finally:
        if libusb_handle:
            libusb.libusb_close(libusb_handle)
        libusb.libusb_free_device_list(dev_list, 1)
        libusb.libusb_exit(ctx)
    
    print(f"\n{'='*60}")
    if total_imu > 0:
        print(f"SUCCESS! {total_imu} IMU packets received")
    else:
        print("FAILED: No IMU data from EP 0x87")
        print("\nThe YLX camera IMU on Linux appears fundamentally different from Windows:")
        print("  - On Windows: IMU data auto-pushes on EP 0x82 without activation")
        print("  - On Linux: EP 0x87 never produces data regardless of:")
        print("    * uvcvideo bound/unbound")
        print("    * V4L2 streaming on/off")
        print("    * XU#4 control queries (all return PIPE error)")
        print("    * Direct libusb interrupt transfers")
        print("\nPossible root causes:")
        print("  1. Firmware detects OS and behaves differently (unlikely)")
        print("  2. IMU needs a vendor-specific USB control command that only")
        print("     the Windows driver sends during initialization")
        print("  3. EP 0x82 (Windows) vs EP 0x87 (Linux) are different endpoints")
        print("     -- need to verify Windows USBPcap capture for endpoint number")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()
