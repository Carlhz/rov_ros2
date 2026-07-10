#!/usr/bin/env python3
"""
YLX Camera IMU Reader - ctypes-based, NO pip dependencies
Uses libusb-1.0 directly via ctypes (libusb-1.0 is already installed)
"""
import ctypes
import ctypes.util
import struct
import sys
import os

VID = 0x1BCF
PID = 0x0B15

# Load libusb
libusb_path = ctypes.util.find_library('usb-1.0')
if not libusb_path:
    # Try explicit paths
    for p in ['/usr/lib/x86_64-linux-gnu/libusb-1.0.so', '/usr/lib/libusb-1.0.so']:
        if os.path.exists(p):
            libusb_path = p
            break
if not libusb_path:
    print("ERROR: libusb-1.0 not found!")
    sys.exit(1)

print(f"libusb: {libusb_path}")
libusb = ctypes.cdll.LoadLibrary(libusb_path)

# libusb types
class libusb_device_descriptor(ctypes.Structure):
    _fields_ = [
        ("bLength", ctypes.c_uint8),
        ("bDescriptorType", ctypes.c_uint8),
        ("bcdUSB", ctypes.c_uint16),
        ("bDeviceClass", ctypes.c_uint8),
        ("bDeviceSubClass", ctypes.c_uint8),
        ("bDeviceProtocol", ctypes.c_uint8),
        ("bMaxPacketSize0", ctypes.c_uint8),
        ("idVendor", ctypes.c_uint16),
        ("idProduct", ctypes.c_uint16),
        ("bcdDevice", ctypes.c_uint16),
        ("iManufacturer", ctypes.c_uint8),
        ("iProduct", ctypes.c_uint8),
        ("iSerialNumber", ctypes.c_uint8),
        ("bNumConfigurations", ctypes.c_uint8),
    ]

class libusb_config_descriptor(ctypes.Structure):
    pass

class libusb_interface_descriptor(ctypes.Structure):
    pass

class libusb_endpoint_descriptor(ctypes.Structure):
    _fields_ = [
        ("bLength", ctypes.c_uint8),
        ("bDescriptorType", ctypes.c_uint8),
        ("bEndpointAddress", ctypes.c_uint8),
        ("bmAttributes", ctypes.c_uint8),
        ("wMaxPacketSize", ctypes.c_uint16),
        ("bInterval", ctypes.c_uint8),
        ("bRefresh", ctypes.c_uint8),
        ("bSynchAddress", ctypes.c_uint8),
        ("extra", ctypes.c_void_p),
        ("extra_length", ctypes.c_int),
    ]

# Function signatures
libusb.libusb_init.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
libusb.libusb_init.restype = ctypes.c_int

libusb.libusb_exit.argtypes = [ctypes.c_void_p]
libusb.libusb_exit.restype = None

libusb.libusb_get_device_list.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_void_p))]
libusb.libusb_get_device_list.restype = ctypes.c_ssize_t

libusb.libusb_free_device_list.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_int]
libusb.libusb_free_device_list.restype = None

libusb.libusb_get_device_descriptor.argtypes = [ctypes.c_void_p, ctypes.POINTER(libusb_device_descriptor)]
libusb.libusb_get_device_descriptor.restype = ctypes.c_int

libusb.libusb_open.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p)]
libusb.libusb_open.restype = ctypes.c_int

libusb.libusb_close.argtypes = [ctypes.c_void_p]
libusb.libusb_close.restype = None

libusb.libusb_get_config_descriptor.argtypes = [ctypes.c_void_p, ctypes.c_uint8, ctypes.POINTER(ctypes.POINTER(libusb_config_descriptor))]
libusb.libusb_get_config_descriptor.restype = ctypes.c_int

libusb.libusb_free_config_descriptor.argtypes = [ctypes.POINTER(libusb_config_descriptor)]
libusb.libusb_free_config_descriptor.restype = None

libusb.libusb_set_configuration.argtypes = [ctypes.c_void_p, ctypes.c_int]
libusb.libusb_set_configuration.restype = ctypes.c_int

libusb.libusb_set_auto_detach_kernel_driver.argtypes = [ctypes.c_void_p, ctypes.c_int]
libusb.libusb_set_auto_detach_kernel_driver.restype = ctypes.c_int

libusb.libusb_claim_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
libusb.libusb_claim_interface.restype = ctypes.c_int

libusb.libusb_release_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
libusb.libusb_release_interface.restype = ctypes.c_int

libusb.libusb_interrupt_transfer.argtypes = [
    ctypes.c_void_p,  # dev_handle
    ctypes.c_uint8,   # endpoint
    ctypes.c_void_p,  # data
    ctypes.c_int,     # length
    ctypes.POINTER(ctypes.c_int),  # transferred
    ctypes.c_uint,    # timeout
]
libusb.libusb_interrupt_transfer.restype = ctypes.c_int

# For navigating config descriptor
libusb.libusb_get_config_descriptor_by_value = libusb.libusb_get_config_descriptor

def errname(code):
    names = {0:"SUCCESS", -1:"ERROR_IO", -2:"ERROR_INVALID_PARAM", -3:"ERROR_ACCESS",
             -4:"ERROR_NO_DEVICE", -5:"ERROR_NOT_FOUND", -6:"ERROR_BUSY",
             -7:"ERROR_TIMEOUT", -8:"ERROR_OVERFLOW", -9:"ERROR_PIPE",
             -10:"ERROR_INTERRUPTED", -11:"ERROR_NO_MEM", -12:"ERROR_NOT_SUPPORTED"}
    return names.get(code, f"ERROR_{code}")

def find_ylx(ctx):
    """Find YLX camera and return device pointer"""
    dev_list = ctypes.POINTER(ctypes.c_void_p)()
    count = libusb.libusb_get_device_list(ctx, ctypes.byref(dev_list))
    if count <= 0:
        print("No USB devices!")
        return None
    
    found = None
    for i in range(count):
        dev = dev_list[i]
        desc = libusb_device_descriptor()
        if libusb.libusb_get_device_descriptor(dev, ctypes.byref(desc)) == 0:
            if desc.idVendor == VID and desc.idProduct == PID:
                found = dev
                print(f"Found YLX camera: {desc.idVendor:04X}:{desc.idProduct:04X}")
                break
    
    libusb.libusb_free_device_list(dev_list, 1)
    return found

def probe_endpoints(handle, dev):
    """Get raw config descriptor and find interrupt endpoints"""
    # We'll parse the raw descriptor using a simpler approach
    # Get config descriptor
    cfg_ptr = ctypes.POINTER(libusb_config_descriptor)()
    ret = libusb.libusb_get_config_descriptor(dev, 0, ctypes.byref(cfg_ptr))
    if ret != 0:
        print(f"get_config_descriptor: {errname(ret)}")
        return None, None
    
    # The config_descriptor in libusb is laid out as:
    # uint8_t bLength, bDescriptorType, uint16_t wTotalLength, 
    # uint8_t bNumInterfaces, bConfigurationValue, iConfiguration,
    # bmAttributes, bMaxPower, then interface array
    # Each interface has altsetting array, each altsetting has endpoint array
    
    # Read wTotalLength (offset 2)
    wTotalLength = struct.unpack_from('<H', ctypes.string_at(cfg_ptr, 4), 2)[0]
    bNumInterfaces = struct.unpack_from('B', ctypes.string_at(cfg_ptr, 5), 0)[0]
    
    raw = ctypes.string_at(cfg_ptr, wTotalLength)
    
    print(f"Config descriptor: {wTotalLength} bytes, {bNumInterfaces} interfaces")
    
    # Parse manually - walk through the descriptor
    # Standard USB descriptor layout
    offset = 9  # Skip config descriptor header (bLength=9)
    imu_iface = None
    imu_ep = None
    
    for iface_idx in range(bNumInterfaces):
        if offset >= len(raw):
            break
        # Interface descriptor (bLength=9)
        bLength = raw[offset]
        if bLength != 9:
            offset += bLength
            continue
        iface_num = raw[offset + 2]
        num_eps = raw[offset + 4]
        iface_class = raw[offset + 5]
        iface_sub = raw[offset + 6]
        iface_proto = raw[offset + 7]
        
        print(f"  Interface {iface_num}: class={iface_class:02X} sub={iface_sub:02X} proto={iface_proto:02X} eps={num_eps}")
        offset += 9
        
        # Walk endpoints
        for ep_idx in range(num_eps):
            if offset >= len(raw):
                break
            ep_len = raw[offset]
            if ep_len != 7:
                offset += ep_len
                continue
            ep_addr = raw[offset + 2]
            ep_attr = raw[offset + 3]
            ep_max = struct.unpack_from('<H', raw, offset + 4)[0]
            ep_intv = raw[offset + 6]
            ep_type = ep_attr & 0x03
            dirn = "IN" if ep_addr & 0x80 else "OUT"
            type_names = {0:"CTRL", 1:"ISOCH", 2:"BULK", 3:"INTR"}
            print(f"    EP 0x{ep_addr:02X} {dirn} {type_names.get(ep_type,'?')} max={ep_max} intv={ep_intv}")
            
            if ep_addr & 0x80 and ep_type == 3:  # IN, Interrupt
                if imu_ep is None or ep_addr == 0x87:
                    imu_ep = ep_addr
                    imu_iface = iface_num
                    imu_max = ep_max
            
            offset += 7
    
    libusb.libusb_free_config_descriptor(cfg_ptr)
    
    if imu_ep:
        print(f"\nIMU endpoint: 0x{imu_ep:02X} on interface {imu_iface}, max={imu_max}")
    else:
        print("No interrupt IN endpoint found!")
    
    return imu_iface, imu_ep

def main():
    ctx = ctypes.c_void_p()
    ret = libusb.libusb_init(ctypes.byref(ctx))
    if ret != 0:
        print(f"libusb_init: {errname(ret)}")
        sys.exit(1)
    
    try:
        dev = find_ylx(ctx)
        if dev is None:
            print("YLX camera not found!")
            sys.exit(1)
        
        # Open device
        handle = ctypes.c_void_p()
        ret = libusb.libusb_open(dev, ctypes.byref(handle))
        if ret != 0:
            print(f"libusb_open: {errname(ret)}")
            sys.exit(1)
        
        try:
            # Set auto-detach kernel driver
            libusb.libusb_set_auto_detach_kernel_driver(handle, 1)
            
            # Probe endpoints
            imu_iface, imu_ep = probe_endpoints(handle, dev)
            if imu_iface is None:
                print("No IMU endpoint found")
                sys.exit(1)
            
            # Set configuration (auto-detach handles driver unbinding)
            ret = libusb.libusb_set_configuration(handle, 1)
            if ret != 0:
                print(f"set_configuration: {errname(ret)}")
            
            # Claim interface
            ret = libusb.libusb_claim_interface(handle, imu_iface)
            if ret != 0:
                print(f"claim_interface: {errname(ret)}")
                # Try without auto-detach
                sys.exit(1)
            
            print(f"\nReading IMU data from EP 0x{imu_ep:02X}...")
            print("(10 second test)")
            
            buf = ctypes.create_string_buffer(64)
            transferred = ctypes.c_int()
            count = 0
            errors = 0
            import time
            start = time.time()
            last_print = start
            
            while time.time() - start < 10:
                ret = libusb.libusb_interrupt_transfer(
                    handle, imu_ep, buf, 8, ctypes.byref(transferred), 1000)
                
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
                              f"X={x:+6d} Y={y:+6d} Z={z:+6d}")
                        last_print = now
                elif ret == -7:  # TIMEOUT
                    pass  # normal
                else:
                    errors += 1
                    if errors <= 5:
                        print(f"  Transfer error: {errname(ret)} ({ret})")
                    if errors > 20:
                        print("Too many errors")
                        break
            
            elapsed = time.time() - start
            rate = count / elapsed if elapsed > 0 else 0
            print(f"\nTotal: {count} packets in {elapsed:.1f}s, {rate:.1f}Hz, {errors} errors")
            
            # Release
            libusb.libusb_release_interface(handle, imu_iface)
            
        finally:
            libusb.libusb_close(handle)
    finally:
        libusb.libusb_exit(ctx)

if __name__ == '__main__':
    main()
