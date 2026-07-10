#!/usr/bin/env python3
"""
Final attempt: Try ALL possible interrupt endpoints on YLX camera
- EP 0x81 through 0x8F (any possible IN interrupt endpoint)
- After unbind from uvcvideo
"""
import ctypes
import ctypes.util
import struct
import os
import sys
import time

VID, PID = 0x1BCF, 0x0B15

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
libusb.libusb_claim_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
libusb.libusb_claim_interface.restype = ctypes.c_int
libusb.libusb_release_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
libusb.libusb_release_interface.restype = ctypes.c_int
libusb.libusb_interrupt_transfer.argtypes = [ctypes.c_void_p, ctypes.c_uint8, ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_uint]
libusb.libusb_interrupt_transfer.restype = ctypes.c_int


def main():
    print("=== YLX IMU: Scan ALL interrupt endpoints ===\n")
    
    # Unbind uvcvideo
    print("Unbinding uvcvideo from 3-2:1.0...")
    try:
        with open("/sys/bus/usb/devices/3-2:1.0/driver/unbind", "w") as f:
            f.write("3-2:1.0")
        print("  Unbound")
    except Exception as e:
        print(f"  Error: {e}")
    
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
            print("YLX not found!")
            return
        
        handle = ctypes.c_void_p()
        ret = libusb.libusb_open(ylx_dev, ctypes.byref(handle))
        if ret != 0:
            print(f"libusb_open error: {ret}")
            return
        
        try:
            # Claim interface 0
            ret = libusb.libusb_claim_interface(handle, 0)
            print(f"claim_interface(0): {ret}")
            
            # Try ALL IN interrupt endpoints: 0x81-0x8F
            print("\nScanning EP 0x81 through 0x8F (1s each)...\n")
            
            buf = ctypes.create_string_buffer(64)
            transferred = ctypes.c_int()
            
            for ep in [0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F]:
                found = 0
                start = time.time()
                while time.time() - start < 1.0:
                    ret = libusb.libusb_interrupt_transfer(handle, ep, buf, 8, ctypes.byref(transferred), 100)
                    if ret == 0 and transferred.value >= 8:
                        data = bytes(buf[:transferred.value])
                        found += 1
                        if found <= 3:
                            raw = ' '.join(f'{b:02X}' for b in data[:8])
                            x = struct.unpack('<h', data[2:4])[0] if len(data) >= 4 else 0
                            y = struct.unpack('<h', data[4:6])[0] if len(data) >= 6 else 0
                            z = struct.unpack('<h', data[6:8])[0] if len(data) >= 8 else 0
                            print(f"  EP 0x{ep:02X} [{found}]: {len(data)}b raw=[{raw}] X={x:+6d} Y={y:+6d} Z={z:+6d}")
                    elif ret == -7:
                        pass  # timeout
                    elif ret == -99:
                        pass  # IO error
                
                if found > 0:
                    print(f"  >>> EP 0x{ep:02X}: {found} packets in 1s *** IMU FOUND ***")
                elif ep == 0x82 or ep == 0x87:
                    print(f"  EP 0x{ep:02X}: 0 packets (no data)")
        
        finally:
            libusb.libusb_release_interface(handle, 0)
            libusb.libusb_close(handle)
    finally:
        libusb.libusb_free_device_list(dev_list, 1)
        libusb.libusb_exit(ctx)
    
    # Rebind
    print("\nRebinding uvcvideo...")
    try:
        with open("/sys/bus/usb/drivers/uvcvideo/bind", "w") as f:
            f.write("3-2:1.0")
        print("  Rebound")
    except Exception as e:
        print(f"  Error: {e}")
    
    print("\nDone.")


if __name__ == '__main__':
    main()
