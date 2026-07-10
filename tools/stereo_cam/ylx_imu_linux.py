#!/usr/bin/env python3
"""
YLX IMU Reader V5 - Correct V4L2 sizes for kernel 5.15
sizeof(v4l2_format) = 204, sizeof(v4l2_buffer) = 88
"""
import struct
import fcntl
import os
import mmap
import time

# IOCTL (corrected)
_IOC_NONE, _IOC_WRITE, _IOC_READ = 0, 1, 2
def _IOC(d,t,n,s): return (d<<30)|(ord(t)<<8)|n|(s<<16)
def _IOWR(t,n,s): return _IOC(_IOC_READ|_IOC_WRITE, t, n, s)
def _IOW(t,n,s): return _IOC(_IOC_WRITE, t, n, s)

# On kernel 5.15: S_FMT=204, S_BUF=88
S_FMT = 204
S_BUF = 88

V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_BUF_TYPE_META_CAPTURE = 13
V4L2_MEMORY_MMAP = 1
V4L2_FIELD_NONE = 1

VIDIOC_S_FMT = _IOWR('V', 5, S_FMT)
VIDIOC_REQBUFS = _IOWR('V', 8, 20)
VIDIOC_QUERYBUF = _IOWR('V', 9, S_BUF)
VIDIOC_QBUF = _IOWR('V', 15, S_BUF)
VIDIOC_DQBUF = _IOWR('V', 17, S_BUF)
VIDIOC_STREAMON = _IOW('V', 18, 4)
VIDIOC_STREAMOFF = _IOW('V', 19, 4)

print("=" * 60)
print("YLX IMU V5 - Test V4L2 Metadata + Video")
print(f"VIDIOC_S_FMT = 0x{VIDIOC_S_FMT:08X}")
print("=" * 60)

# Test 1: Open video0 and start MJPG streaming
print("\n--- Test 1: Video stream ---")
vid_fd = os.open("/dev/video0", os.O_RDWR)
print(f"video0 opened")

# Set format
fmt = bytearray(S_FMT)
struct.pack_into('I', fmt, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE);
struct.pack_into('I', fmt, 4, 640)   # width
struct.pack_into('I', fmt, 8, 480)   # height
struct.pack_into('4s', fmt, 12, b'MJPG')  # pixelformat
struct.pack_into('I', fmt, 16, V4L2_FIELD_NONE)  # field

try:
    fcntl.ioctl(vid_fd, VIDIOC_S_FMT, bytes(fmt))
    print(f"S_FMT OK: {struct.unpack_from('I', fmt, 0)[0]}={struct.unpack_from('4s', fmt, 12)[0]}")
except OSError as e:
    print(f"S_FMT failed: {e}")

# Even if S_FMT fails, try loading uvcvideo properly first
# Let me use v4l2-ctl in subprocess for the video part

# Instead of low-level V4L2, let me combine approaches:
# Use v4l2-ctl subprocess to stream video
# Use Python to read metadata

print("\n--- Test 2: Metadata stream ---")
meta_fd = os.open("/dev/video1", os.O_RDWR)
print(f"video1 opened")

# Set metadata format
mfmt = bytearray(S_FMT)
struct.pack_into('I', mfmt, 0, V4L2_BUF_TYPE_META_CAPTURE)
try:
    fcntl.ioctl(meta_fd, VIDIOC_S_FMT, bytes(mfmt))
    print(f"Meta S_FMT OK")
except OSError as e:
    print(f"Meta S_FMT failed: {e}")

# Request buffers for metadata
req = struct.pack('IIII', 4, V4L2_BUF_TYPE_META_CAPTURE, V4L2_MEMORY_MMAP, 0)
try:
    fcntl.ioctl(meta_fd, VIDIOC_REQBUFS, req)
    count = struct.unpack_from('I', req, 0)[0]
    print(f"Meta REQBUFS: {count} buffers")
except OSError as e:
    print(f"Meta REQBUFS failed: {e}")

# Query buffer
buf = bytearray(S_BUF)
struct.pack_into('II', buf, 0, 0, V4L2_BUF_TYPE_META_CAPTURE)
try:
    fcntl.ioctl(meta_fd, VIDIOC_QUERYBUF, bytes(buf))
    offset = struct.unpack_from('I', buf, 64)[0]  # m.offset
    length = struct.unpack_from('I', buf, 72)[0]
    print(f"QUERYBUF: offset=0x{offset:X} length={length}")
    
    # mmap
    mm = mmap.mmap(meta_fd, length, offset=offset)
    print(f"mmap OK: {len(mm)} bytes")
    
    # Queue
    qb = bytearray(S_BUF)
    struct.pack_into('II', qb, 0, 0, V4L2_BUF_TYPE_META_CAPTURE)
    struct.pack_into('I', qb, 60, V4L2_MEMORY_MMAP)
    fcntl.ioctl(meta_fd, VIDIOC_QBUF, bytes(qb))
    print("QBUF OK")
    
    # Stream on
    fcntl.ioctl(meta_fd, VIDIOC_STREAMON, struct.pack('I', V4L2_BUF_TYPE_META_CAPTURE))
    print("STREAMON OK")
    
    # Now dequeue
    print("\nDequeuing metadata buffers...")
    start = time.time()
    count = 0
    
    while time.time() - start < 8:
        db = bytearray(S_BUF)
        struct.pack_into('II', db, 0, 0, V4L2_BUF_TYPE_META_CAPTURE)
        struct.pack_into('I', db, 60, V4L2_MEMORY_MMAP)
        
        try:
            fcntl.ioctl(meta_fd, VIDIOC_DQBUF, bytes(db))
            idx = struct.unpack_from('I', db, 0)[0]
            bytesused = struct.unpack_from('I', db, 8)[0]
            seq = struct.unpack_from('I', db, 56)[0]
            
            if bytesused > 0:
                count += 1
                elapsed = time.time() - start
                mm.seek(0)
                data = mm.read(bytesused)
                h = data[:64].hex()
                
                if count <= 10:
                    rate = count / elapsed if elapsed > 0 else 0
                    print(f"[{elapsed:6.1f}s] #{count:4d} rate={rate:.1f}Hz seq={seq} [{bytesused}B] {h}")
            
            # Requeue
            struct.pack_into('II', db, 0, idx, V4L2_BUF_TYPE_META_CAPTURE)
            struct.pack_into('I', db, 60, V4L2_MEMORY_MMAP)
            fcntl.ioctl(meta_fd, VIDIOC_QBUF, bytes(db))
        
        except OSError as e:
            print(f"DQBUF error: {e}")
            break
    
    elapsed = time.time() - start
    print(f"\n{count} metadata frames in {elapsed:.1f}s")
    
    fcntl.ioctl(meta_fd, VIDIOC_STREAMOFF, struct.pack('I', V4L2_BUF_TYPE_META_CAPTURE))
    
except OSError as e:
    print(f"mmap/queue failed: {e}")

os.close(meta_fd)
os.close(vid_fd)

print("\nDone!")
