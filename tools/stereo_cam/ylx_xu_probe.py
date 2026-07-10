#!/usr/bin/env python3
"""
Probe XU#4 controls via UVCIOC_CTRL_QUERY - try to activate IMU
"""
import os, struct, fcntl, ctypes, time

UVCIOC_CTRL_QUERY = 0xC0187521  # _IOWR('u', 0x21, struct uvc_xu_control_query)

# struct uvc_xu_control_query {
#   __u8  unit;       // offset 0
#   __u8  selector;   // offset 1
#   __u8  query;      // offset 2  (UVC_GET_CUR=1, UVC_GET_LEN=3, UVC_SET_CUR=2)
#   __u16 size;       // offset 4 (aligned to 2)
#   __u8  *data;      // offset 8 (pointer on 64-bit)
# }
# sizeof = 16

UVC_GET_CUR = 1
UVC_SET_CUR = 2
UVC_GET_LEN = 3
UVC_GET_MIN = 4
UVC_GET_MAX = 5
UVC_GET_RES = 6
UVC_GET_INFO = 8
XU_UNIT = 4  # XU#4

def xu_query(fd, unit, selector, query, data_size=64):
    """Send UVC XU control query"""
    data = (ctypes.c_uint8 * data_size)()
    buf = struct.pack('BBHxxxxQ',
        unit, selector, query,
        ctypes.addressof(data))
    
    try:
        fcntl.ioctl(fd, UVCIOC_CTRL_QUERY, buf)
        return bytes(data[:data_size])
    except OSError as e:
        return None

def xu_get_len(fd, unit, selector):
    """Get the length of a control"""
    result = xu_query(fd, unit, selector, UVC_GET_LEN, 2)
    if result:
        return struct.unpack_from('<H', result, 0)[0]
    return 0

def xu_get_cur(fd, unit, selector, length):
    """Get current value"""
    return xu_query(fd, unit, selector, UVC_GET_CUR, max(length, 64))

def xu_set_cur(fd, unit, selector, data):
    """Set current value"""
    # Build buffer with data inline
    buf_size = 16 + len(data)
    buf = bytearray(buf_size)
    struct.pack_into('BBHH', buf, 0, unit, selector, UVC_SET_CUR, len(data))
    # Pointer to data (inline at offset 16)
    data_ptr_addr = ctypes.addressof((ctypes.c_uint8 * buf_size).from_buffer(buf))
    struct.pack_into('Q', buf, 8, data_ptr_addr + 16)
    buf[16:] = data
    try:
        fcntl.ioctl(fd, UVCIOC_CTRL_QUERY, bytes(buf))
        return True
    except OSError as e:
        print(f"  SET_CUR failed: {e}")
        return False

def main():
    print("=" * 60)
    print("XU#4 Control Probe & IMU Activation")
    print("=" * 60)
    
    # Open video0
    fd = os.open("/dev/video0", os.O_RDWR)
    print(f"Opened /dev/video0")
    
    # Probe all 25 controls of XU#4
    print(f"\nProbing XU#4 (GUID 63610682-5070-49ab-b8cc-b3855e8d221d)...")
    print(f"25 controls total\n")
    
    for sel in range(1, 26):
        length = xu_get_len(fd, XU_UNIT, sel)
        if length == 0:
            print(f"  Control {sel:2d}: GET_LEN returned 0 (unsupported)")
            continue
        
        cur = xu_get_cur(fd, XU_UNIT, sel, length)
        if cur:
            hex_val = cur[:min(length, 16)].hex()
            print(f"  Control {sel:2d}: LEN={length}, CUR={hex_val}")
        else:
            print(f"  Control {sel:2d}: LEN={length}, CUR=(error reading)")
    
    # Try common activation patterns
    print("\n\n=== Activation attempts ===")
    activation_sets = [
        (1, b'\x01'),       # Enable control 1
        (1, b'\x01\x00'*8), # Full enable bytes
        (2, b'\x01'),       # Enable control 2
        (3, b'\x01'),       # Enable control 3
        (1, b'\x01\x00\x00\x00'),
        (1, b'\x03'),       # 0x03 common for enable+something
    ]
    
    for sel, data in activation_sets:
        if xu_set_cur(fd, XU_UNIT, sel, data):
            print(f"  SET Control {sel} = {data.hex()}: OK")
        else:
            print(f"  SET Control {sel} = {data.hex()}: FAILED")
    
    # After activation, try reading USB interrupt endpoint via /dev/video0
    print("\n\n=== After XU activation: read input events ===")
    # Check if any new input events appeared
    for i in range(20):
        path = f"/dev/input/event{i}"
        if os.path.exists(path):
            try:
                # Just check if readable
                pass
            except:
                pass
    
    os.close(fd)
    print("\nDone! Re-run libusb EP 0x87 reader after XU activation")

if __name__ == "__main__":
    main()
