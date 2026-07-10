#!/usr/bin/env python3
"""
Try to activate IMU via UVC XU#4, then read EP 0x87
Uses /dev/video0 ioctl (UVCIOC_CTRL_QUERY) for XU#4 control
"""
import fcntl
import struct
import os
import array
import time

# UVC ioctl constants
UVCIOC_CTRL_QUERY = 0x8008761c  # _IOWR('u', 0x21, struct uvc_xu_control_query)

# XU#4 GUID: 63610682-5070-49ab-b8cc-b3855e8d221d
XU4_GUID = bytes([0x82, 0x06, 0x61, 0x63, 0x70, 0x50, 0xab, 0x49,
                  0xb8, 0xcc, 0xb3, 0x85, 0x5e, 0x8d, 0x22, 0x1d])

# struct uvc_xu_control_query {
#     __u8  unit;      // XU unit ID = 4
#     __u8  selector;  // control selector
#     __u8  query;     // UVC_GET_CUR(1), UVC_GET_INFO(4), UVC_SET_CUR(2)
#     __u16 size;      // data size
#     __u8  *data;     // data pointer (userspace)
# };
# Total size: 16 bytes on x86_64 (1+1+1+1 padding + 2 + 8 pointer)

UVC_GET_CUR = 1
UVC_SET_CUR = 2
UVC_GET_INFO = 4
UVC_GET_LEN = 5
UVC_GET_MIN = 3
UVC_GET_MAX = 6

def xu_query(fd, unit, selector, query, data=b''):
    """Send a UVC XU control query via ioctl"""
    size = len(data)
    data_arr = array.array('B', data)
    data_ptr, _ = data_arr.buffer_info()
    
    # Pack struct: unit(u8), selector(u8), query(u8), padding(u8), size(u16), data_ptr(u64)
    query_struct = struct.pack('BBBBHQ', unit, selector, query, 0, size, data_ptr)
    
    try:
        fcntl.ioctl(fd, UVCIOC_CTRL_QUERY, query_struct)
        # Read back the data
        return bytes(data_arr[:size])
    except OSError as e:
        return f"ERROR: {e}"

def main():
    fd = os.open('/dev/video0', os.O_RDWR)
    if fd < 0:
        print("Cannot open /dev/video0")
        return
    
    try:
        print("=== Probing XU#4 controls ===")
        
        # XU#4 has 25 controls (bmControls: 0xff 0xff 0x77 0x07 = 25 bits)
        # Unit ID = 4 (from lsusb output)
        XU_ID = 4
        
        for sel in range(25):
            # GET_INFO
            result = xu_query(fd, XU_ID, sel, UVC_GET_INFO, b'\x00')
            if isinstance(result, str):
                print(f"  XU#4 sel={sel:2d}: {result}")
            else:
                info_byte = result[0] if result else 0
                supported = (info_byte & 0x01)
                get_cur = (info_byte & 0x02)
                set_cur = (info_byte & 0x04)
                if supported:
                    print(f"  XU#4 sel={sel:2d}: SUPPORTED get={bool(get_cur)} set={bool(set_cur)}")
                    
                    # Try GET_LEN
                    len_result = xu_query(fd, XU_ID, sel, UVC_GET_LEN, b'\x00\x00')
                    if not isinstance(len_result, str) and len(len_result) >= 2:
                        length = struct.unpack('<H', len_result[:2])[0]
                        print(f"           length={length}")
                        
                        if length > 0 and length <= 256 and get_cur:
                            data = xu_query(fd, XU_ID, sel, UVC_GET_CUR, b'\x00' * length)
                            if not isinstance(data, str):
                                print(f"           GET_CUR: {' '.join(f'{b:02X}' for b in data)}")
                            else:
                                print(f"           GET_CUR: {data}")
        
        # Now try activating IMU
        # Based on Windows behavior, IMU activates with video streaming
        # Let's try setting UVC VS_COMMIT_CONTROL via V4L2
        print("\n=== Trying to start video stream ===")
        
        # Open video device and start streaming to activate IMU
        # Use v4l2 API
        import ctypes
        import ctypes.util
        
        # V4L2 constants
        V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
        VIDIOC_STREAMON = 0x40045612
        VIDIOC_STREAMOFF = 0x40045613
        
        # Open video0 for streaming
        video_fd = os.open('/dev/video0', os.O_RDWR)
        try:
            # Start stream
            buf_type = struct.pack('I', V4L2_BUF_TYPE_VIDEO_CAPTURE)
            try:
                fcntl.ioctl(video_fd, VIDIOC_STREAMON, buf_type)
                print("STREAMON sent to /dev/video0")
                time.sleep(1)
                fcntl.ioctl(video_fd, VIDIOC_STREAMOFF, buf_type)
                print("STREAMOFF sent")
            except OSError as e:
                print(f"STREAMON/OFF: {e}")
        finally:
            os.close(video_fd)
        
        print("\n=== Now trying to read EP 0x87 with stream active ===")
        print("(Need to keep stream open while reading)")
        
    finally:
        os.close(fd)

if __name__ == '__main__':
    main()
