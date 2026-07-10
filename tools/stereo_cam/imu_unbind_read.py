#!/usr/bin/env python3
"""
YLX Camera IMU Reader - unbind uvcvideo, read EP 0x87 directly

The YLX camera is at /sys/bus/usb/devices/3-2
  - 3-2:1.0 = Video Control interface (has EP 0x87 interrupt, bound to uvcvideo)
  - 3-2:1.1 = Video Streaming interface (bound to uvcvideo)

Strategy:
  1. Unbind 3-2:1.0 from uvcvideo driver
  2. Use libusb to claim interface 0 and read EP 0x87
  3. On exit, rebind uvcvideo
"""
import ctypes
import ctypes.util
import struct
import os
import sys
import time

VID, PID = 0x1BCF, 0x0B15
EP_IMU = 0x87
SYSFS_DEV = "/sys/bus/usb/devices/3-2"

def load_libusb():
    path = ctypes.util.find_library('usb-1.0')
    if not path:
        for p in ['/usr/lib/x86_64-linux-gnu/libusb-1.0.so']:
            if os.path.exists(p): path = p; break
    return ctypes.cdll.LoadLibrary(path)

libusb = load_libusb()

# Setup libusb functions
libusb.libusb_init.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
libusb.libusb_init.restype = ctypes.c_int
libusb.libusb_exit.argtypes = [ctypes.c_void_p]
libusb.libusb_exit.restype = None
libusb.libusb_get_device_list.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_void_p))]
libusb.libusb_get_device_list.restype = ctypes.c_ssize_t
libusb.libusb_free_device_list.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_int]
libusb.libusb_free_device_list.restype = None

class Desc(ctypes.Structure):
    _fields_ = [("bLength", ctypes.c_uint8),("bDescriptorType", ctypes.c_uint8),
                ("bcdUSB", ctypes.c_uint16),("bDeviceClass", ctypes.c_uint8),
                ("bDeviceSubClass", ctypes.c_uint8),("bDeviceProtocol", ctypes.c_uint8),
                ("bMaxPacketSize0", ctypes.c_uint8),("idVendor", ctypes.c_uint16),
                ("idProduct", ctypes.c_uint16),("bcdDevice", ctypes.c_uint16),
                ("iManufacturer", ctypes.c_uint8),("iProduct", ctypes.c_uint8),
                ("iSerialNumber", ctypes.c_uint8),("bNumConfigurations", ctypes.c_uint8)]
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

def errname(c):
    return {-3:"ACCESS",-4:"NO_DEVICE",-5:"NOT_FOUND",-6:"BUSY",-7:"TIMEOUT",-99:"OTHER"}.get(c,str(c))

def unbind_driver(iface_path):
    """Unbind a USB interface from its driver"""
    driver_link = os.path.join(iface_path, "driver")
    if os.path.exists(driver_link):
        driver_name = os.path.basename(os.readlink(driver_link))
        unbind_path = os.path.join(iface_path, "driver/unbind")
        dev_name = os.path.basename(iface_path)
        print(f"  Unbinding {dev_name} from {driver_name}...")
        with open(unbind_path, 'w') as f:
            f.write(dev_name)
        print(f"  Unbound OK")
        return driver_name
    return None

def bind_driver(iface_path, driver_name):
    """Rebind a USB interface to a driver"""
    bind_path = f"/sys/bus/usb/drivers/{driver_name}/bind"
    dev_name = os.path.basename(iface_path)
    if os.path.exists(bind_path):
        print(f"  Rebinding {dev_name} to {driver_name}...")
        try:
            with open(bind_path, 'w') as f:
                f.write(dev_name)
        except Exception as e:
            print(f"  Rebind failed: {e}")

def main():
    # Step 1: Unbind uvcvideo from interface 0
    iface0_path = f"{SYSFS_DEV}:1.0"
    old_driver = unbind_driver(iface0_path)
    
    try:
        # Step 2: Init libusb and find device
        ctx = ctypes.c_void_p()
        libusb.libusb_init(ctypes.byref(ctx))
        
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
                print("ERROR: YLX camera not found!")
                return
            
            # Step 3: Open device
            handle = ctypes.c_void_p()
            ret = libusb.libusb_open(ylx_dev, ctypes.byref(handle))
            if ret != 0:
                print(f"ERROR: libusb_open: {errname(ret)}")
                return
            
            try:
                # Claim interface 0 (now free since we unbound uvcvideo)
                ret = libusb.libusb_claim_interface(handle, 0)
                if ret != 0:
                    print(f"ERROR: claim interface 0: {errname(ret)}")
                    return
                
                print(f"Claimed interface 0 OK")
                print(f"Reading IMU from EP 0x{EP_IMU:02X} (10 seconds)...")
                print()
                
                buf = ctypes.create_string_buffer(64)
                transferred = ctypes.c_int()
                count = 0
                errors = 0
                start = time.time()
                last_print = start
                
                while time.time() - start < 10:
                    ret = libusb.libusb_interrupt_transfer(
                        handle, EP_IMU, buf, 8, ctypes.byref(transferred), 1000)
                    
                    if ret == 0 and transferred.value >= 8:
                        data = bytes(buf[:transferred.value])
                        hdr = struct.unpack('<H', data[0:2])[0]
                        x = struct.unpack('<h', data[2:4])[0]
                        y = struct.unpack('<h', data[4:6])[0]
                        z = struct.unpack('<h', data[6:8])[0]
                        count += 1
                        
                        now = time.time()
                        if now - last_print >= 1.0:
                            elapsed = now - start
                            rate = count / elapsed if elapsed > 0 else 0
                            print(f"  [{elapsed:5.1f}s] #{count:4d} rate={rate:5.1f}Hz  "
                                  f"X={x:+6d} Y={y:+6d} Z={z:+6d}  "
                                  f"raw={' '.join(f'{b:02X}' for b in data[:8])}")
                            last_print = now
                    elif ret == -7:
                        pass  # timeout, normal
                    else:
                        errors += 1
                        if errors <= 3:
                            print(f"  Error: {errname(ret)}")
                        if errors > 10:
                            print("Too many errors")
                            break
                
                elapsed = time.time() - start
                rate = count / elapsed if elapsed > 0 else 0
                print(f"\nTotal: {count} packets in {elapsed:.1f}s, {rate:.1f}Hz, {errors} errors")
                
                if count == 0:
                    print("\nNo data received! Trying without timeout...")
                    # Try one blocking read
                    for attempt in range(5):
                        ret = libusb.libusb_interrupt_transfer(
                            handle, EP_IMU, buf, 8, ctypes.byref(transferred), 3000)
                        print(f"  Attempt {attempt}: ret={ret} transferred={transferred.value}")
                        if ret == 0:
                            data = bytes(buf[:transferred.value])
                            print(f"  Data: {' '.join(f'{b:02X}' for b in data)}")
                
                libusb.libusb_release_interface(handle, 0)
            finally:
                libusb.libusb_close(handle)
        finally:
            libusb.libusb_free_device_list(dev_list, 1)
            libusb.libusb_exit(ctx)
    
    finally:
        # Step 4: Rebind uvcvideo
        if old_driver:
            bind_driver(iface0_path, old_driver)

if __name__ == '__main__':
    main()
