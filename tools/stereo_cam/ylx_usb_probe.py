#!/usr/bin/env python3
"""
Enhanced USB probe - dump raw config descriptor, try all alt settings
"""
import ctypes
import struct

libusb = ctypes.CDLL("libusb-1.0.so.0")

LIBUSB_DT_INTERFACE = 4
LIBUSB_DT_ENDPOINT = 5
LIBUSB_DT_CS_INTERFACE = 0x24
LIBUSB_DT_CS_ENDPOINT = 0x25

EP_TYPE_NAMES = {0: "CONTROL", 1: "ISO", 2: "BULK", 3: "INTERRUPT"}

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

# Setup all API sigs
libusb.libusb_init.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
libusb.libusb_init.restype = ctypes.c_int
libusb.libusb_exit.argtypes = [ctypes.c_void_p]
libusb.libusb_exit.restype = None
libusb.libusb_get_device_list.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_void_p))]
libusb.libusb_get_device_list.restype = ctypes.c_ssize_t
libusb.libusb_free_device_list.argtypes = [ctypes.c_void_p, ctypes.c_int]
libusb.libusb_get_device_descriptor.argtypes = [ctypes.c_void_p, ctypes.POINTER(libusb_device_descriptor)]
libusb.libusb_get_device_descriptor.restype = ctypes.c_int
libusb.libusb_get_config_descriptor.argtypes = [ctypes.c_void_p, ctypes.c_uint8, ctypes.POINTER(ctypes.c_void_p)]
libusb.libusb_get_config_descriptor.restype = ctypes.c_int
libusb.libusb_free_config_descriptor.argtypes = [ctypes.c_void_p]
libusb.libusb_open.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p)]
libusb.libusb_open.restype = ctypes.c_int
libusb.libusb_close.argtypes = [ctypes.c_void_p]
libusb.libusb_set_auto_detach_kernel_driver.argtypes = [ctypes.c_void_p, ctypes.c_int]
libusb.libusb_set_auto_detach_kernel_driver.restype = ctypes.c_int
libusb.libusb_claim_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
libusb.libusb_claim_interface.restype = ctypes.c_int
libusb.libusb_release_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
libusb.libusb_release_interface.restype = ctypes.c_int
libusb.libusb_set_interface_alt_setting.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
libusb.libusb_set_interface_alt_setting.restype = ctypes.c_int
libusb.libusb_interrupt_transfer.argtypes = [
    ctypes.c_void_p, ctypes.c_uint8, ctypes.c_void_p, ctypes.c_int,
    ctypes.POINTER(ctypes.c_int), ctypes.c_uint
]
libusb.libusb_interrupt_transfer.restype = ctypes.c_int
libusb.libusb_strerror.argtypes = [ctypes.c_int]
libusb.libusb_strerror.restype = ctypes.c_char_p
libusb.libusb_control_transfer.argtypes = [
    ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint16,
    ctypes.c_uint16, ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint
]
libusb.libusb_control_transfer.restype = ctypes.c_int
libusb.libusb_kernel_driver_active.argtypes = [ctypes.c_void_p, ctypes.c_int]
libusb.libusb_kernel_driver_active.restype = ctypes.c_int
libusb.libusb_detach_kernel_driver.argtypes = [ctypes.c_void_p, ctypes.c_int]
libusb.libusb_detach_kernel_driver.restype = ctypes.c_int
libusb.libusb_attach_kernel_driver.argtypes = [ctypes.c_void_p, ctypes.c_int]
libusb.libusb_attach_kernel_driver.restype = ctypes.c_int


def dump_config(raw_data, length):
    """Dump raw config descriptor in detail"""
    print(f"  Config descriptor: {length} bytes")
    
    # Show hex dump of first 128 bytes
    print("  Raw (first 128 bytes):")
    for i in range(0, min(128, length), 16):
        chunk = raw_data[i:i+16]
        hex_str = ' '.join(f'{b:02X}' for b in chunk)
        print(f"    {i:04X}: {hex_str}")
    
    offset = 0
    while offset < length:
        if offset + 2 > length:
            break
        dlen = raw_data[offset]
        dtype = raw_data[offset + 1]
        
        if dlen == 0 or offset + dlen > length:
            break
        
        if dtype == LIBUSB_DT_INTERFACE and dlen >= 9:
            iface = raw_data[offset + 2]
            alt = raw_data[offset + 3]
            eps = raw_data[offset + 4]
            cls = raw_data[offset + 5]
            sub = raw_data[offset + 6]
            proto = raw_data[offset + 7]
            
            cls_name = {0x01: "Audio", 0x03: "HID", 0x0E: "Video", 0xFF: "Vendor"}.get(cls, f"0x{cls:02X}")
            print(f"\n  Interface {iface} Alt {alt}: Class={cls_name} Sub={sub:02X} Proto={proto:02X} EPs={eps}")
        
        elif dtype == LIBUSB_DT_ENDPOINT and dlen >= 7:
            addr = raw_data[offset + 2]
            attr = raw_data[offset + 3]
            maxp = struct.unpack_from("<H", raw_data, offset + 4)[0]
            interval = raw_data[offset + 6]
            
            direction = "IN " if (addr & 0x80) else "OUT"
            etype = attr & 0x03
            type_name = EP_TYPE_NAMES.get(etype, f"0x{etype:X}")
            
            usage = attr >> 4
            usage_names = []
            if usage & 1: usage_names.append("DATA")
            if usage & 2: usage_names.append("FEEDBACK")
            if usage & 4: usage_names.append("IMPLICIT")
            
            print(f"    EP 0x{addr:02X} {direction} {type_name} max={maxp}B interval={interval} "
                  f"usage={'|'.join(usage_names) if usage_names else 'N/A'}")
        
        elif dtype == LIBUSB_DT_CS_INTERFACE:
            subtype = raw_data[offset + 2]
            print(f"    CS_INTERFACE subtype=0x{subtype:02X} len={dlen}")
            if subtype == 0x06:  # EXTENSION_UNIT
                guid = raw_data[offset+4:offset+20]
                guid_hex = ' '.join(f'{b:02X}' for b in guid)
                num_ctrls = raw_data[offset + 20]
                print(f"      XU GUID: {guid_hex} ({num_ctrls} controls)")
        
        offset += dlen


def main():
    print("=" * 60)
    print("YLX Camera Enhanced USB Probe")
    print("=" * 60)
    
    ctx = ctypes.c_void_p()
    libusb.libusb_init(ctypes.byref(ctx))
    
    devs = ctypes.POINTER(ctypes.c_void_p)()
    cnt = libusb.libusb_get_device_list(ctx, ctypes.byref(devs))
    
    VID, PID = 0x1BCF, 0x0B15
    found_dev = None
    
    for i in range(cnt):
        desc = libusb_device_descriptor()
        libusb.libusb_get_device_descriptor(devs[i], ctypes.byref(desc))
        if desc.idVendor == VID and desc.idProduct == PID:
            found_dev = devs[i]
            
            # Dump config
            cfg_ptr = ctypes.c_void_p()
            if libusb.libusb_get_config_descriptor(devs[i], 0, ctypes.byref(cfg_ptr)) == 0:
                wTL = struct.unpack_from("<H", (ctypes.c_uint8 * 4).from_address(cfg_ptr.value), 2)[0]
                raw = bytes((ctypes.c_uint8 * wTL).from_address(cfg_ptr.value))
                dump_config(raw, wTL)
                libusb.libusb_free_config_descriptor(cfg_ptr)
    
    if not found_dev:
        print("Device not found!")
        return
    
    # Now try reading with all possible approaches
    print("\n" + "=" * 60)
    print("Trying EP 0x82 reads...")
    print("=" * 60)
    
    handle = ctypes.c_void_p()
    if libusb.libusb_open(found_dev, ctypes.byref(handle)) != 0:
        print("Failed to open device")
        return
    
    try:
        # Enable auto detach
        libusb.libusb_set_auto_detach_kernel_driver(handle, 1)
        
        for iface in range(2):  # Only 2 interfaces exist
            print(f"\n--- Interface {iface} ---")
            
            # Detach kernel driver
            libusb.libusb_detach_kernel_driver(handle, iface)
            
            # Try all alt settings (up to 16)
            for alt in range(16):
                ret = libusb.libusb_set_interface_alt_setting(handle, iface, alt)
                if ret != 0:
                    if alt == 0:
                        print(f"  Cannot set alt 0: {libusb.libusb_strerror(ret).decode()}")
                        break
                    continue  # alt not available, skip
                
                # Claim interface
                ret = libusb.libusb_claim_interface(handle, iface)
                if ret != 0:
                    continue
                
                # Try reading EP 0x82
                buf = (ctypes.c_uint8 * 64)()
                transferred = ctypes.c_int()
                
                print(f"  Alt {alt}: reading EP 0x82...")
                ret = libusb.libusb_interrupt_transfer(
                    handle, 0x82, buf, 64,
                    ctypes.byref(transferred), 1000
                )
                
                if ret == 0:
                    data = bytes(buf[:transferred.value])
                    print(f"  >>> SUCCESS! Got {transferred.value}B from alt {alt}")
                    print(f"  >>> Data: {data.hex()}")
                    
                    # Try a few more reads
                    for _ in range(5):
                        ret = libusb.libusb_interrupt_transfer(
                            handle, 0x82, buf, 64,
                            ctypes.byref(transferred), 1000
                        )
                        if ret == 0:
                            data = bytes(buf[:transferred.value])
                            print(f"  >>> Additional: [{transferred.value}B] {data.hex()}")
                        else:
                            print(f"  >>> Error on additional: {libusb.libusb_strerror(ret).decode()}")
                            break
                else:
                    err = libusb.libusb_strerror(ret).decode()
                    print(f"  >>> {err}")
                
                libusb.libusb_release_interface(handle, iface)
    
    finally:
        libusb.libusb_close(handle)
        libusb.libusb_exit(ctx)


if __name__ == "__main__":
    main()
