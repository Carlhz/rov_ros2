#!/usr/bin/env python3
"""
终极元数据探测: 多种方式尝试从 /dev/video1 读取 UVC metadata
"""
import os, fcntl, mmap, array, struct, time, select

V4L2_BUF_TYPE_META_CAPTURE = 13
V4L2_MEMORY_MMAP = 1
VIDIOC_REQBUFS = 0xC0145608
VIDIOC_QUERYBUF = 0xC0585609
VIDIOC_QBUF = 0xC058560F
VIDIOC_DQBUF = 0xC0585611
VIDIOC_STREAMON = 0x40045612
VIDIOC_STREAMOFF = 0x40045613
VIDIOC_S_FMT = 0xC0D05604
VIDIOC_G_FMT = 0xC0D05604  # same as S_FMT

def try_meta_stream(buf_count=3, timeout=1.0):
    """尝试从 /dev/video1 流式传输 metadata"""
    print(f"  Trying metadata stream with {buf_count} buffers...")
    
    fd = os.open("/dev/video1", os.O_RDWR)
    try:
        # Try 1: REQBUFS without setting format (default should be UVCH)
        req = struct.pack('I I I', V4L2_BUF_TYPE_META_CAPTURE, V4L2_MEMORY_MMAP, buf_count)
        req_arr = array.array('B', req)
        try:
            fcntl.ioctl(fd, VIDIOC_REQBUFS, req_arr, True)
            count = struct.unpack('I I I', req_arr.tobytes())[2]
            print(f"  REQBUFS OK, allocated {count} buffers")
        except OSError as e:
            print(f"  REQBUFS failed: {e}")
            os.close(fd)
            return
        
        if count == 0:
            print("  No buffers allocated")
            os.close(fd)
            return
        
        # Query+mmap buffers
        buffers = []
        for i in range(count):
            qbuf = struct.pack('I I I 4x I I I', V4L2_BUF_TYPE_META_CAPTURE, i, V4L2_MEMORY_MMAP, 0,0,0)
            qbuf_arr = array.array('B', qbuf)
            fcntl.ioctl(fd, VIDIOC_QUERYBUF, qbuf_arr, True)
            _, _, _, _, offset, length, _ = struct.unpack('I I I 4x I I I', qbuf_arr.tobytes())
            buf = mmap.mmap(fd, length, offset=offset, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ)
            buffers.append((buf, length, offset))
            print(f"  Buffer {i}: offset={offset}, length={length}")
        
        # Queue all
        for i in range(count):
            qbuf = struct.pack('I I I 4x I', V4L2_BUF_TYPE_META_CAPTURE, i, V4L2_MEMORY_MMAP, 0)
            fcntl.ioctl(fd, VIDIOC_QBUF, array.array('B', qbuf), True)
        
        # Stream on
        fcntl.ioctl(fd, VIDIOC_STREAMON, 
                    array.array('B', struct.pack('I', V4L2_BUF_TYPE_META_CAPTURE)), True)
        print("  Stream ON")
        
        # Read frames
        for n in range(10):
            r, _, _ = select.select([fd], [], [], timeout)
            if not r:
                print(f"  [{n}] TIMEOUT (no metadata)")
                break
            
            dqbuf = struct.pack('I I I 4x I I I', V4L2_BUF_TYPE_META_CAPTURE, 0, V4L2_MEMORY_MMAP, 0,0,0)
            dqbuf_arr = array.array('B', dqbuf)
            fcntl.ioctl(fd, VIDIOC_DQBUF, dqbuf_arr, True)
            _, idx, _, _, bytesused, _, _ = struct.unpack('I I I 4x I I I', dqbuf_arr.tobytes())
            
            if bytesused == 0:
                print(f"  [{n}] EMPTY")
            else:
                raw = bytes(buffers[idx][0][:bytesused])
                hex_str = ' '.join(f'{b:02X}' for b in raw[:64])
                print(f"  [{n}] {bytesused} bytes: {hex_str}")
                
                # Gyro pattern check
                for j in range(0, len(raw) - 6, 2):
                    v1 = (raw[j] << 8) | raw[j+1]
                    v2 = (raw[j+2] << 8) | raw[j+3]
                    v3 = (raw[j+4] << 8) | raw[j+5]
                    if (v1 & 0x0F) == 0 and (v2 & 0x0F) == 0 and (v3 & 0x0F) == 0:
                        x, y, z = v1 >> 4, v2 >> 4, v3 >> 4
                        if max(x, y, z) > 0 and max(x, y, z) < 4096:
                            h = ' '.join(f'{b:02X}' for b in raw[j:j+6])
                            print(f"    → GYRO@+{j}: {h} (X={x},Y={y},Z={z})")
            
            # Re-queue
            qbuf = struct.pack('I I I 4x I', V4L2_BUF_TYPE_META_CAPTURE, idx, V4L2_MEMORY_MMAP, 0)
            fcntl.ioctl(fd, VIDIOC_QBUF, array.array('B', qbuf), True)
        
        # Stream off
        fcntl.ioctl(fd, VIDIOC_STREAMOFF,
                    array.array('B', struct.pack('I', V4L2_BUF_TYPE_META_CAPTURE)), True)
        print("  Stream OFF")
        
    finally:
        os.close(fd)


def main():
    print("=" * 60)
    print("  UVC Metadata Final Probe")
    print("=" * 60)
    
    if not os.path.exists("/dev/video1"):
        print("/dev/video1 not found")
        return
    
    # Try different buffer counts
    for bc in [4, 2, 8, 1]:
        print()
        try_meta_stream(buf_count=bc, timeout=2.0)
        break  # just try first viable one for now


if __name__ == "__main__":
    main()
