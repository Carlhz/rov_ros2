#!/usr/bin/env python3
"""
在 Linux 上通过 UVC XU ioctl 发送陀螺仪激活命令
用法: sudo python3 ylx_activate_gyro.py [bus=1] [dev=1]

从 USBPcap 抓包分析结果中提取的激活命令序列，通过 UVCIOC_CTRL_SET 发送到 XU#4
"""

import fcntl
import struct
import array
import sys
import os
import time

# UVC ioctl codes
# UVCIOC_CTRL_QUERY = 0x802C5521 (GET)
# UVCIOC_CTRL_SET = 0x402C5521 (SET) - actually, let me check
# From linux/uvcvideo.h:
# #define UVCIOC_CTRL_QUERY    _IOWR('V', 33, struct uvc_xu_control_query)
# #define UVCIOC_CTRL_SET       _IOW('V', 34, struct uvc_xu_control_query)
import ctypes

UVCIOC_CTRL_QUERY = 0xC0105609  # _IOWR('V', 33, ...)
UVCIOC_CTRL_SET   = 0x4018560A  # _IOW('V', 34, ...)

# UVC control selectors
UVC_SET_CUR = 0x01
UVC_GET_CUR = 0x81
UVC_GET_MIN = 0x82
UVC_GET_MAX = 0x83
UVC_GET_RES = 0x84
UVC_GET_LEN = 0x85
UVC_GET_INFO = 0x86
UVC_GET_DEF = 0x87

# XU ID for gyroscope
XU_GYRO_ID = 4

# This will be filled in from pcap analysis
# Format: list of (selector, request_code, data_bytes)
ACTIVATION_SEQUENCE = [
    # EXAMPLE - replace with actual pcap results
    # (0x01, UVC_SET_CUR, bytes([0x01, 0x00, 0x00, 0x00])),
    # (0x02, UVC_SET_CUR, bytes([0x00, 0x00, 0x00, 0x00])),
]


def open_uvc_device(dev_node='/dev/video0'):
    """Open the UVCH metadata device or video device"""
    fd = os.open(dev_node, os.O_RDWR)
    return fd


def uvc_xu_ctrl_query(fd, entity_id, selector, query_code, data=None, data_len=0):
    """
    Send UVC Extension Unit control query.
    
    struct uvc_xu_control_query {
        __u8 unit;        // Extension unit ID
        __u8 selector;    // Control selector
        __u8 query;       // Request code (SET_CUR, GET_CUR, etc.)
        __u16 size;       // Data size
        __u8 *data;       // Data buffer
    };
    """
    if data is None:
        data = array.array('B', [0] * data_len)
    elif isinstance(data, bytes):
        data = array.array('B', data)
    
    buf_size = max(len(data), data_len or 256)
    data_pointer = data.buffer_info()[0]
    
    query_struct = struct.pack('BBBHQ',
        entity_id & 0xFF,
        selector & 0xFF,
        query_code & 0xFF,
        buf_size & 0xFFFF,
        data_pointer
    )
    
    query_arr = array.array('B', query_struct)
    
    result = fcntl.ioctl(fd, UVCIOC_CTRL_QUERY, query_arr)
    
    # Read back the data
    result_bytes = data.tobytes()[:buf_size]
    return result_bytes


def uvc_xu_ctrl_set(fd, entity_id, selector, data):
    """Send UVC Extension Unit SET command"""
    if isinstance(data, (list, tuple)):
        data = bytes(data)
    
    data_arr = array.array('B', data)
    data_len = len(data)
    data_pointer = data_arr.buffer_info()[0]
    
    set_struct = struct.pack('BBBHQ',
        entity_id & 0xFF,
        selector & 0xFF,
        UVC_SET_CUR,
        data_len & 0xFFFF,
        data_pointer
    )
    
    set_arr = array.array('B', set_struct)
    
    try:
        fcntl.ioctl(fd, UVCIOC_CTRL_SET, set_arr)
        return True
    except OSError as e:
        print(f"  SET failed: {e}")
        return False


def probe_xu(fd, entity_id):
    """Probe an Extension Unit: read all selectors"""
    print(f"\n=== Probing XU#{entity_id} ===")
    
    for sel in range(1, 256):  # selector 1-255
        try:
            # Try GET_LEN first
            result = uvc_xu_ctrl_query(fd, entity_id, sel, UVC_GET_LEN, data_len=2)
            data_len = struct.unpack('<H', result[:2])[0]
            
            if data_len > 0:
                print(f"  sel=0x{sel:02X} GET_LEN={data_len}")
                
                # Try GET_CUR
                result = uvc_xu_ctrl_query(fd, entity_id, sel, UVC_GET_CUR, data_len=data_len)
                hex_str = ' '.join(f'{b:02X}' for b in result[:min(data_len, 64)])
                print(f"    GET_CUR: {hex_str}")
        except OSError:
            pass  # Selector doesn't exist


def send_activation_sequence(fd):
    """Send the activation sequence captured from Windows"""
    print("\n=== Sending Activation Sequence ===")
    
    for i, (selector, req_code, data) in enumerate(ACTIVATION_SEQUENCE):
        if req_code == UVC_SET_CUR:
            print(f"\nStep {i+1}: SET_CUR sel=0x{selector:02X} data={data.hex(' ')}")
            success = uvc_xu_ctrl_set(fd, XU_GYRO_ID, selector, data)
            print(f"  Result: {'OK' if success else 'FAIL'}")
            time.sleep(0.1)
        elif req_code == UVC_GET_CUR:
            print(f"\nStep {i+1}: GET_CUR sel=0x{selector:02X}")
            result = uvc_xu_ctrl_query(fd, XU_GYRO_ID, selector, UVC_GET_CUR, data_len=len(data) if data else 64)
            hex_str = ' '.join(f'{b:02X}' for b in result[:min(len(result), 64)])
            print(f"  Result: {hex_str}")


def main():
    if len(sys.argv) >= 3:
        bus = int(sys.argv[1])
        dev = int(sys.argv[2])
        dev_node = f'/dev/bus/usb/{bus:03d}/{dev:03d}'
    else:
        # Auto-detect
        dev_node = '/dev/video0'
    
    print(f"Device: {dev_node}")
    
    try:
        fd = open_uvc_device(dev_node)
        print(f"Opened {dev_node} (fd={fd})")
    except Exception as e:
        print(f"Error opening {dev_node}: {e}")
        print("Trying /dev/bus/usb/...")
        # Find the right device
        import glob
        for path in sorted(glob.glob('/dev/bus/usb/*/*')):
            try:
                fd = open_uvc_device(path)
                print(f"Opened {path} (fd={fd})")
                break
            except:
                pass
        else:
            print("Could not find any USB device")
            sys.exit(1)
    
    # Probe XU#4 to see what's available
    probe_xu(fd, XU_GYRO_ID)
    
    # Send activation sequence (if populated from pcap)
    if ACTIVATION_SEQUENCE:
        send_activation_sequence(fd)
    else:
        print("\n*** No activation sequence defined yet. ***")
        print("Run analyze_pcap.py on the USBPcap capture first,")
        print("then fill in ACTIVATION_SEQUENCE in this script.")
    
    os.close(fd)


if __name__ == '__main__':
    main()
