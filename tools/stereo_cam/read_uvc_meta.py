#!/usr/bin/env python3
"""
直接读取 UVC Metadata (/dev/video1)
=================================
使用 V4L2 ioctl 读取 UVCH metadata buffer
"""
import os, fcntl, mmap, array, struct, sys

# V4L2 constants
V4L2_BUF_TYPE_META_CAPTURE = 13  # Metadata capture
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1

VIDIOC_REQBUFS = 0xC0145608
VIDIOC_QUERYBUF = 0xC0585609
VIDIOC_QBUF = 0xC058560F
VIDIOC_DQBUF = 0xC0585611
VIDIOC_STREAMON = 0x40045612
VIDIOC_STREAMOFF = 0x40045613
VIDIOC_S_FMT = 0xC0D05605

# UVCH fourcc
V4L2_META_FMT_UVC = ord('U') | (ord('V') << 8) | (ord('C') << 16) | (ord('H') << 24)


def read_metadata(device="/dev/video1", num_bufs=3, frames=20):
    """Read metadata from UVC metadata device"""
    
    print(f"Opening {device}...")
    fd = os.open(device, os.O_RDWR)
    
    try:
        # Set format to UVCH metadata
        # v4l2_format: type (4) + padding (4) + 
        #   meta.dataformat (8 bytes in: 4) + meta.buffersize (12 bytes in: 4) = total 208
        # Just use a 208-byte buffer and set the right fields
        raw_fmt = bytearray(208)
        struct.pack_into('I', raw_fmt, 0, V4L2_BUF_TYPE_META_CAPTURE)
        struct.pack_into('I', raw_fmt, 8, V4L2_META_FMT_UVC)  # dataformat
        struct.pack_into('I', raw_fmt, 12, 1024)  # buffersize
        fmt_arr = array.array('B', raw_fmt)
        
        try:
            fcntl.ioctl(fd, VIDIOC_S_FMT, fmt_arr, True)
            result = fmt_arr.tobytes()
            fmt_type = struct.unpack_from('I', result, 0)[0]
            dataformat = struct.unpack_from('I', result, 8)[0]
            buffersize = struct.unpack_from('I', result, 12)[0]
            pf_str = ''.join(chr((dataformat >> (8*i)) & 0xFF) for i in range(4))
            print(f"  Format set: type={fmt_type}, buffersize={buffersize}, fourcc='{pf_str}'")
        except OSError as e:
            print(f"  S_FMT failed: {e}")
            print("  Trying with default format...")
        
        # Request buffers
        req = struct.pack('I I I', V4L2_BUF_TYPE_META_CAPTURE, V4L2_MEMORY_MMAP, num_bufs)
        req_arr = array.array('B', req)
        
        try:
            fcntl.ioctl(fd, VIDIOC_REQBUFS, req_arr, True)
            _, _, count = struct.unpack('I I I', req_arr.tobytes())
            print(f"  REQBUFS: count={count}")
        except OSError as e:
            print(f"  REQBUFS failed: {e}")
            print("  (Maybe metadata stream needs video stream to be active?)")
            os.close(fd)
            return
        
        if count == 0:
            print("  0 buffers allocated")
            os.close(fd)
            return
        
        # Query buffers and mmap
        buffers = []
        for i in range(count):
            qbuf = struct.pack('I I I 4x I I I', V4L2_BUF_TYPE_META_CAPTURE, i, V4L2_MEMORY_MMAP, 0, 0, 0)
            qbuf_arr = array.array('B', qbuf)
            fcntl.ioctl(fd, VIDIOC_QUERYBUF, qbuf_arr, True)
            _, _, _, _, offset, length, _ = struct.unpack('I I I 4x I I I', qbuf_arr.tobytes())
            buf = mmap.mmap(fd, length, offset=offset, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ)
            buffers.append((buf, length, offset))
            print(f"  Buffer {i}: offset={offset}, length={length}")
        
        # Queue all buffers
        for i in range(count):
            qbuf = struct.pack('I I I 4x I', V4L2_BUF_TYPE_META_CAPTURE, i, V4L2_MEMORY_MMAP, 0)
            qbuf_arr = array.array('B', qbuf)
            fcntl.ioctl(fd, VIDIOC_QBUF, qbuf_arr, True)
        print("  Buffers queued")
        
        # Start stream
        fcntl.ioctl(fd, VIDIOC_STREAMON, 
                    array.array('B', struct.pack('I', V4L2_BUF_TYPE_META_CAPTURE)), True)
        print("  Stream started")
        
        # Capture frames
        import select, time
        
        print(f"\n  Capturing {frames} metadata frames...")
        print(f"  {'#'}  {'bytesused'}  {'data (first 48 bytes)'}")
        print(f"  {'-'*60}")
        
        for n in range(frames):
            dqbuf = struct.pack('I I I 4x I I I', V4L2_BUF_TYPE_META_CAPTURE, 0, V4L2_MEMORY_MMAP, 0, 0, 0)
            dqbuf_arr = array.array('B', dqbuf)
            
            # Wait with timeout
            r, _, _ = select.select([fd], [], [], 2.0)
            if not r:
                print(f"  {n:2d}  TIMEOUT (no metadata available)")
                # Maybe metadata only comes when video is streaming
                # Queue a dummy buffer
                continue
            
            fcntl.ioctl(fd, VIDIOC_DQBUF, dqbuf_arr, True)
            _, idx, _, _, bytesused, _, seq = struct.unpack('I I I 4x I I I', dqbuf_arr.tobytes())
            
            if bytesused == 0:
                print(f"  {n:2d}  {bytesused:5d}  (empty)")
            else:
                raw = bytes(buffers[idx][0][:bytesused])
                hex_str = ' '.join(f'{b:02X}' for b in raw[:48])
                print(f"  {n:2d}  {bytesused:5d}  {hex_str}")
                
                # Check for gyro pattern
                for j in range(0, len(raw) - 6, 2):
                    v1 = (raw[j] << 8) | raw[j+1]
                    v2 = (raw[j+2] << 8) | raw[j+3]
                    v3 = (raw[j+4] << 8) | raw[j+5]
                    if (v1 & 0x0F) == 0 and (v2 & 0x0F) == 0 and (v3 & 0x0F) == 0:
                        x, y, z = v1 >> 4, v2 >> 4, v3 >> 4
                        if max(x, y, z) > 0 and max(x, y, z) < 4096:
                            h = ' '.join(f'{b:02X}' for b in raw[j:j+8])
                            print(f"      → GYRO@+{j}: {h} (X={x},Y={y},Z={z})")
            
            # Re-queue
            qbuf = struct.pack('I I I 4x I', V4L2_BUF_TYPE_META_CAPTURE, idx, V4L2_MEMORY_MMAP, 0)
            qbuf_arr = array.array('B', qbuf)
            fcntl.ioctl(fd, VIDIOC_QBUF, qbuf_arr, True)
        
        # Stop stream
        fcntl.ioctl(fd, VIDIOC_STREAMOFF,
                    array.array('B', struct.pack('I', V4L2_BUF_TYPE_META_CAPTURE)), True)
        print("  Stream stopped")
        
    finally:
        os.close(fd)


def main():
    print("=" * 60)
    print("  UVC Metadata Direct Read (/dev/video1)")
    print("=" * 60)
    
    if not os.path.exists("/dev/video1"):
        print("/dev/video1 not found")
        return
    
    read_metadata("/dev/video1", num_bufs=3, frames=20)
    
    print("\nDone!")


if __name__ == "__main__":
    main()
