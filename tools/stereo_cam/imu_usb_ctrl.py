#!/usr/bin/env python3
"""
YLX IMU activation via USB control transfers (bypass kernel uvcvideo)
Send UVC XU#4 commands directly over USB control endpoint 0

UVC control request format:
  bmRequestType = 0xA1 (Device-to-Host, Class, Interface)
  bRequest = UVC_GET_CUR (0x81) or UVC_SET_CUR (0x01)  
  wValue = (selector << 8) | (0x04 for XU#4)  -- XU unit ID shifted differently
  wIndex = (unit << 8) | interface
  data = control data

Actually, UVC XU uses:
  bmRequestType = 0xA1 (GET) or 0x21 (SET)  
  bRequest = 0x01 (SET_CUR) or 0x81 (GET_CUR) etc
  wValue = (UVC_CTRL << 8) | selector  where UVC_CTRL values:
    GET_CUR=0x81, GET_MIN=0x82, GET_MAX=0x83, GET_RES=0x84,
    GET_LEN=0x85, GET_INFO=0x86, GET_DEF=0x87
    SET_CUR=0x01, SET_CUR_ALL=0x11
  wIndex = (unit << 8) | interface_number
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

# libusb function signatures
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

# libusb_control_transfer
libusb.libusb_control_transfer.argtypes = [
    ctypes.c_void_p,  # dev_handle
    ctypes.c_uint8,   # bmRequestType
    ctypes.c_uint8,   # bRequest
    ctypes.c_uint16,  # wValue
    ctypes.c_uint16,  # wIndex
    ctypes.c_void_p,  # data
    ctypes.c_uint16,  # wLength
    ctypes.c_uint,    # timeout
]
libusb.libusb_control_transfer.restype = ctypes.c_int

def ctrl_transfer(handle, bmReq, bReq, wVal, wIdx, data=None, wLen=0, timeout=1000):
    if data is None:
        buf = ctypes.create_string_buffer(wLen) if wLen > 0 else ctypes.c_void_p(0)
    else:
        buf = ctypes.create_string_buffer(data, len(data))
        wLen = len(data)
    
    ret = libusb.libusb_control_transfer(handle, bmReq, bReq, wVal, wIdx, buf, wLen, timeout)
    if ret < 0:
        return f"ERR:{ret}"
    if wLen > 0 and (bmReq & 0x80):  # Device-to-Host
        return bytes(buf[:ret])
    return ret

def main():
    # Unbind uvcvideo from interface 0
    print("Unbinding uvcvideo from 3-2:1.0...")
    try:
        with open("/sys/bus/usb/devices/3-2:1.0/driver/unbind", "w") as f:
            f.write("3-2:1.0")
        print("  Unbound")
    except Exception as e:
        print(f"  Unbind error: {e}")
    
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
            print(f"Open error: {ret}")
            return
        
        try:
            # Claim interface 0
            ret = libusb.libusb_set_auto_detach_kernel_driver(handle, 1)
            print(f"auto_detach: {ret}")
            
            # ============ Probe XU#4 via USB control transfers ============
            XU_ID = 4
            IFACE = 0  # interface 0
            
            print("\n=== Probing XU#4 via USB control transfers ===")
            
            for sel in range(25):
                # GET_INFO: bmReq=0xA1, bReq=0x86, wVal=(0x00 << 8) | sel, wIdx=(XU_ID << 8) | IFACE
                # Actually UVC bRequest values:
                # RC_UNDEFINED=0x00, SET_CUR=0x01, GET_CUR=0x81, GET_MIN=0x82,
                # GET_MAX=0x83, GET_RES=0x84, GET_LEN=0x85, GET_INFO=0x86, GET_DEF=0x87
                
                wVal = sel  # (0 << 8) | selector
                wIdx = (XU_ID << 8) | IFACE
                
                result = ctrl_transfer(handle, 0xA1, 0x86, wVal, wIdx, wLen=1)
                
                if isinstance(result, str):
                    if sel == 0:
                        print(f"  sel={sel:2d}: {result}")
                    continue
                
                if result and len(result) > 0:
                    info = result[0]
                    supported = bool(info & 0x01)
                    if supported:
                        # GET_LEN
                        len_result = ctrl_transfer(handle, 0xA1, 0x85, wVal, wIdx, wLen=2)
                        length = 0
                        if not isinstance(len_result, str) and len(len_result) >= 2:
                            length = struct.unpack('<H', len_result[:2])[0]
                        
                        # GET_CUR
                        cur_data = ""
                        if length > 0 and length <= 256:
                            cur_result = ctrl_transfer(handle, 0xA1, 0x81, wVal, wIdx, wLen=length)
                            if not isinstance(cur_result, str):
                                cur_data = ' '.join(f'{b:02X}' for b in cur_result)
                        
                        print(f"  sel={sel:2d}: SUPPORTED len={length} cur=[{cur_data}]")
                    else:
                        print(f"  sel={sel:2d}: unsupported")
            
            # Try to read EP 0x87 now
            print("\n=== Reading EP 0x87 ===")
            libusb.libusb_interrupt_transfer.argtypes = [ctypes.c_void_p, ctypes.c_uint8, ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_uint]
            libusb.libusb_interrupt_transfer.restype = ctypes.c_int
            
            buf = ctypes.create_string_buffer(64)
            transferred = ctypes.c_int()
            
            for attempt in range(20):
                ret = libusb.libusb_interrupt_transfer(handle, 0x87, buf, 8, ctypes.byref(transferred), 1000)
                if ret == 0 and transferred.value >= 8:
                    data = bytes(buf[:transferred.value])
                    x = struct.unpack('<h', data[2:4])[0]
                    y = struct.unpack('<h', data[4:6])[0]
                    z = struct.unpack('<h', data[6:8])[0]
                    print(f"  #{attempt}: X={x:+6d} Y={y:+6d} Z={z:+6d}  raw={' '.join(f'{b:02X}' for b in data[:8])}")
                elif ret == 0:
                    print(f"  #{attempt}: short {transferred.value}b")
                elif ret == -7:
                    pass  # timeout
                else:
                    print(f"  #{attempt}: err={ret}")
            
            print("\nDone")
            
        finally:
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
        print(f"  Rebind error: {e}")

if __name__ == '__main__':
    main()
