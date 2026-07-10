#!/usr/bin/env python3
"""
Step 3: UVC Extension Unit (XU) IMU/Gyro reader
Probes the stereo camera's IMU data channel via UVC Extension Unit.

Usage:
  sudo python3 step3_imu_probe.py

Note: Root/sudo required for raw USB access.
Depends on: python3-usb (sudo apt install python3-usb)
            OR fcntl ioctl via V4L2 XU controls
"""

import os
import sys
import struct
import time
import ctypes
import fcntl
import array

# ============================================================
#  Method A: V4L2 IOCTL approach (no extra deps, common)
# ============================================================

# V4L2 IOCTL constants
VIDIOC_S_EXT_CTRLS = 0xC0180570
VIDIOC_G_EXT_CTRLS = 0xC0180571
VIDIOC_QUERYCTRL   = 0xC0445600

# UVC XU control class
V4L2_CTRL_CLASS_CAMERA = 0x009A0000
V4L2_CID_CAMERA_CLASS  = 0x009A0001

# These are the ioctl numbers for UVC XU
UVCIOC_CTRL_MAP   = 0xC0246F20
UVCIOC_CTRL_QUERY = 0xC0186F21

UVC_SET_CUR = 0x01
UVC_GET_CUR = 0x81
UVC_GET_LEN = 0x85
UVC_GET_INFO= 0x86


def v4l2_xu_query(dev_fd, unit_id, selector, query_type, data_len):
    """Send a UVC XU control query via ioctl."""
    # struct uvc_xu_control_query
    # uint8_t  unit
    # uint8_t  selector
    # uint16_t query
    # uint16_t size
    # uint8_t* data
    buf = (ctypes.c_uint8 * data_len)()
    # Pack the query struct: unit(1) selector(1) query(2) size(2) data_ptr(8 on 64bit)
    # Layout: unit=1B, selector=1B, query=2B, size=2B, reserved=2B, data=pointer
    class XUQuery(ctypes.Structure):
        _fields_ = [
            ('unit',     ctypes.c_uint8),
            ('selector', ctypes.c_uint8),
            ('query',    ctypes.c_uint16),
            ('size',     ctypes.c_uint16),
            ('reserved', ctypes.c_uint16),
            ('data',     ctypes.c_void_p),
        ]
    xu = XUQuery()
    xu.unit     = unit_id
    xu.selector = selector
    xu.query    = query_type
    xu.size     = data_len
    xu.reserved = 0
    xu.data     = ctypes.cast(buf, ctypes.c_void_p)

    try:
        fcntl.ioctl(dev_fd, UVCIOC_CTRL_QUERY, xu)
        return bytes(buf)
    except Exception as e:
        return None


def probe_xu_units(video_dev="/dev/video0"):
    """Try common XU unit IDs and selectors to find IMU data."""
    print(f"\n[Probing XU on {video_dev}]")
    print("  Trying unit IDs 1-8, selectors 1-8, GET_CUR...")
    
    try:
        fd = os.open(video_dev, os.O_RDWR | os.O_NONBLOCK)
    except Exception as e:
        print(f"  Cannot open {video_dev}: {e}")
        return None

    found = []
    for unit in range(1, 9):
        for sel in range(1, 9):
            # Try to get length first
            data = v4l2_xu_query(fd, unit, sel, UVC_GET_LEN, 2)
            if data:
                length = struct.unpack("<H", data[:2])[0]
                if 1 < length < 256:
                    print(f"  Unit {unit}, Selector {sel}: length={length} bytes")
                    # Get current value
                    val = v4l2_xu_query(fd, unit, sel, UVC_GET_CUR, length)
                    if val:
                        hex_str = ' '.join(f'{b:02X}' for b in val)
                        print(f"    Data: {hex_str}")
                        found.append((unit, sel, length, val))

    os.close(fd)
    if not found:
        print("  No XU controls responded — camera may not have standard XU IMU")
    return found


# ============================================================
#  Method B: libuvc / pyuvc approach (if installed)
# ============================================================
def try_pyuvc():
    """Try reading IMU via pyuvc if available."""
    try:
        import usb.core
        import usb.util
        print("\n[Method B: pyusb found — scanning for UVC devices]")
        devices = list(usb.core.find(find_all=True))
        for dev in devices:
            try:
                mfr = usb.util.get_string(dev, dev.iManufacturer) if dev.iManufacturer else ""
                prd = usb.util.get_string(dev, dev.iProduct) if dev.iProduct else ""
                if mfr or prd:
                    print(f"  VID:{dev.idVendor:04X} PID:{dev.idProduct:04X}  {mfr} {prd}")
            except Exception:
                pass
    except ImportError:
        print("\n[Method B: pyusb not installed]")
        print("  Install: pip3 install pyusb")


# ============================================================
#  Method C: Read IMU stream as separate V4L2 metadata device
# ============================================================
def try_metadata_devices():
    """Some cameras expose IMU as V4L2 metadata node."""
    print("\n[Method C: Checking V4L2 metadata devices]")
    import glob
    devs = sorted(glob.glob("/dev/video*"))
    for dev in devs:
        try:
            import subprocess
            out = subprocess.check_output(
                ["v4l2-ctl", "-d", dev, "--info"],
                stderr=subprocess.DEVNULL, timeout=3
            ).decode()
            if "Meta" in out or "meta" in out or "IMU" in out or "imu" in out:
                print(f"  {dev}: METADATA device found!")
                print(f"  {out[:300]}")
        except Exception:
            pass


# ============================================================
#  Main
# ============================================================
def main():
    print("=" * 55)
    print("  UVC IMU/Gyro Probe  (V1)")
    print("=" * 55)

    if os.geteuid() != 0:
        print("\nWARNING: Not running as root. XU ioctl may fail.")
        print("  Re-run: sudo python3 step3_imu_probe.py\n")

    import glob
    video_devs = sorted(glob.glob("/dev/video*"))
    if not video_devs:
        print("No /dev/video* devices found. Connect camera first.")
        return

    print(f"\nFound devices: {video_devs}")

    # Try each video device
    for dev in video_devs[:4]:
        found = probe_xu_units(dev)

    # Method B
    try_pyuvc()

    # Method C
    try_metadata_devices()

    print("\n" + "=" * 55)
    print("  If no IMU data found above:")
    print("  1. Check camera datasheet for XU GUID")
    print("  2. Run: sudo lsusb -v | grep -A20 'Extension Unit'")
    print("  3. Some cameras use vendor SDK, not standard XU")
    print("=" * 55)


if __name__ == "__main__":
    main()
