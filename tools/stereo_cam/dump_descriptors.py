#!/usr/bin/env python3
"""Dump full USB descriptor tree for YLX camera"""
import ctypes, ctypes.util, struct, os, sys

libusb_path = ctypes.util.find_library('usb-1.0')
if not libusb_path:
    for p in ['/usr/lib/x86_64-linux-gnu/libusb-1.0.so', '/usr/lib/libusb-1.0.so']:
        if os.path.exists(p):
            libusb_path = p
            break
libusb = ctypes.cdll.LoadLibrary(libusb_path)

class libusb_device_descriptor(ctypes.Structure):
    _fields_ = [
        ("bLength", ctypes.c_uint8), ("bDescriptorType", ctypes.c_uint8),
        ("bcdUSB", ctypes.c_uint16), ("bDeviceClass", ctypes.c_uint8),
        ("bDeviceSubClass", ctypes.c_uint8), ("bDeviceProtocol", ctypes.c_uint8),
        ("bMaxPacketSize0", ctypes.c_uint8), ("idVendor", ctypes.c_uint16),
        ("idProduct", ctypes.c_uint16), ("bcdDevice", ctypes.c_uint16),
        ("iManufacturer", ctypes.c_uint8), ("iProduct", ctypes.c_uint8),
        ("iSerialNumber", ctypes.c_uint8), ("bNumConfigurations", ctypes.c_uint8),
    ]

class libusb_config_descriptor(ctypes.Structure):
    pass

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
libusb.libusb_get_config_descriptor.argtypes = [ctypes.c_void_p, ctypes.c_uint8, ctypes.POINTER(ctypes.POINTER(libusb_config_descriptor))]
libusb.libusb_get_config_descriptor.restype = ctypes.c_int
libusb.libusb_free_config_descriptor.argtypes = [ctypes.POINTER(libusb_config_descriptor)]
libusb.libusb_free_config_descriptor.restype = None

ctx = ctypes.c_void_p()
libusb.libusb_init(ctypes.byref(ctx))

dev_list = ctypes.POINTER(ctypes.c_void_p)()
count = libusb.libusb_get_device_list(ctx, ctypes.byref(dev_list))

found = None
for i in range(count):
    dev = dev_list[i]
    desc = libusb_device_descriptor()
    libusb.libusb_get_device_descriptor(dev, ctypes.byref(desc))
    if desc.idVendor == 0x1BCF and desc.idProduct == 0x0B15:
        found = dev
        break

if not found:
    print("YLX not found!")
    libusb.libusb_free_device_list(dev_list, 1)
    libusb.libusb_exit(ctx)
    sys.exit(1)

cfg_ptr = ctypes.POINTER(libusb_config_descriptor)()
libusb.libusb_get_config_descriptor(found, 0, ctypes.byref(cfg_ptr))
wTotalLength = struct.unpack_from('<H', ctypes.string_at(cfg_ptr, 4), 2)[0]
raw = ctypes.string_at(cfg_ptr, wTotalLength)

print(f"Config descriptor: {wTotalLength} bytes")
print(f"Raw hex (first 256):")
for i in range(0, min(256, wTotalLength), 16):
    hexstr = ' '.join(f'{raw[j]:02X}' for j in range(i, min(i+16, wTotalLength)))
    ascii = ''.join(chr(raw[j]) if 32 <= raw[j] < 127 else '.' for j in range(i, min(i+16, wTotalLength)))
    print(f"  {i:04X}: {hexstr:<48s} {ascii}")

# Parse all descriptors
print(f"\n=== Full descriptor tree ===")
offset = 0
iface_idx = 0
while offset < wTotalLength:
    bLength = raw[offset]
    bType = raw[offset + 1]
    if bLength == 0:
        break
    
    type_names = {1:"DEVICE", 2:"CONFIG", 4:"INTERFACE", 5:"ENDPOINT", 
                  0x0B:"IAD", 0x21:"HID", 0x24:"CS_INTERFACE", 0x25:"CS_ENDPOINT",
                  0x0A:"CS_INTERFACE2"}
    tname = type_names.get(bType, f"0x{bType:02X}")
    
    if bType == 4:  # Interface
        iface_num = raw[offset + 2]
        alt = raw[offset + 3]
        num_eps = raw[offset + 4]
        cls = raw[offset + 5]
        sub = raw[offset + 6]
        proto = raw[offset + 7]
        print(f"\n  IF{offset:04X}: iface={iface_num} alt={alt} cls={cls:02X} sub={sub:02X} proto={proto:02X} eps={num_eps}")
    
    elif bType == 5:  # Endpoint
        ep_addr = raw[offset + 2]
        ep_attr = raw[offset + 3]
        ep_max = struct.unpack_from('<H', raw, offset + 4)[0]
        ep_intv = raw[offset + 6]
        ep_type = ep_attr & 0x03
        dirn = "IN" if ep_addr & 0x80 else "OUT"
        type_names2 = {0:"CTRL", 1:"ISOCH", 2:"BULK", 3:"INTR"}
        marker = " <--- IMU?" if (ep_addr & 0x80 and ep_type == 3) else ""
        print(f"    EP 0x{ep_addr:02X} {dirn} {type_names2.get(ep_type,'?')} max={ep_max} intv={ep_intv}{marker}")
    
    elif bType == 0x24:  # CS_INTERFACE (UVC)
        subtype = raw[offset + 2]
        uvcn = {1:"VC_HEADER", 2:"VC_INPUT_TERMINAL", 3:"VC_OUTPUT_TERMINAL", 
                4:"VC_SELECTOR_UNIT", 5:"VC_PROCESSING_UNIT", 6:"VC_EXTENSION_UNIT",
                1+0x10:"VS_INPUT_HEADER", 2+0x10:"VS_FORMAT", 3+0x10:"VS_FRAME",
                4+0x10:"VS_FRAME_MJPEG", 5+0x10:"VS_STILL_IMAGE_FRAME",
                6+0x10:"VS_COLORFORMAT"}
        uvc_name = uvcn.get(subtype, f"0x{subtype:02X}")
        if subtype == 6:  # Extension Unit
            guid = raw[offset+4:offset+20]
            guid_str = '-'.join(f'{b:02X}' for b in guid)
            ctrls = raw[offset+20]
            print(f"    XU: GUID={guid_str} ctrls={ctrls}")
        else:
            print(f"    CS_IF: {uvc_name} ({bLength}B)")
    
    elif bType == 2:  # Config
        print(f"CONFIG: total={wTotalLength}B")
    
    else:
        print(f"  [{offset:04X}] {tname} ({bLength}B)")
    
    offset += bLength

libusb.libusb_free_config_descriptor(cfg_ptr)
libusb.libusb_free_device_list(dev_list, 1)
libusb.libusb_exit(ctx)
