#!/usr/bin/env python3
"""Quick USB probe for YLX camera IMU endpoint"""
import usb.core
import usb.util
import struct
import sys

VID, PID = 0x1BCF, 0x0B15

dev = usb.core.find(idVendor=VID, idProduct=PID)
if dev is None:
    print("DEVICE_NOT_FOUND")
    sys.exit(1)

print(f"Device: {dev}")
print(f"Configs: {dev.bNumConfigurations}")

# Show all endpoints
for cfg in dev:
    for i, iface in enumerate(cfg):
        for alt in iface:
            cls = alt.bInterfaceClass
            sub = alt.bInterfaceSubClass
            proto = alt.bInterfaceProtocol
            print(f"  IF{i}/Alt{alt.bAlternateSetting}: cls={cls:02X} sub={sub:02X} proto={proto:02X}")
            for ep in alt:
                addr = ep.bEndpointAddress
                dirn = "IN" if addr & 0x80 else "OUT"
                eptype = ["CTRL","ISO","BULK","INTR"][ep.bmAttributes & 0x03]
                print(f"    EP 0x{addr:02X} {dirn} {eptype} max={ep.wMaxPacketSize} intv={ep.bInterval}")

# Find IMU interface
imu_iface = None
for cfg in dev:
    for i, iface in enumerate(cfg):
        for alt in iface:
            for ep in alt:
                if ep.bEndpointAddress == 0x87 and (ep.bmAttributes & 0x03) == 3:
                    imu_iface = i
                    print(f"\nIMU endpoint found on interface {i}")

if imu_iface is None:
    # Try EP 0x82
    for cfg in dev:
        for i, iface in enumerate(cfg):
            for alt in iface:
                for ep in alt:
                    if ep.bEndpointAddress == 0x82 and (ep.bmAttributes & 0x03) == 3:
                        imu_iface = i
                        print(f"\nIMU endpoint (0x82) found on interface {i}")

if imu_iface is None:
    print("NO_IMU_ENDPOINT")
    sys.exit(1)

# Detach and claim
print(f"\nKernel driver active: {dev.is_kernel_driver_active(imu_iface)}")
if dev.is_kernel_driver_active(imu_iface):
    dev.detach_kernel_driver(imu_iface)
    print("Detached")

try:
    dev.set_configuration()
    print("Config set")
except Exception as e:
    print(f"Config: {e}")

usb.util.claim_interface(dev, imu_iface)
print(f"Claimed iface {imu_iface}")

# Try reading
ep_addr = 0x87
max_pkt = 8
print(f"\nReading EP 0x{ep_addr:02X}...")
for attempt in range(50):
    try:
        data = dev.read(ep_addr, max_pkt, timeout=1000)
        if data and len(data) >= 8:
            hdr = struct.unpack('<H', data[0:2])[0]
            x = struct.unpack('<h', data[2:4])[0]
            y = struct.unpack('<h', data[4:6])[0]
            z = struct.unpack('<h', data[6:8])[0]
            raw = ' '.join(f'{b:02X}' for b in data[:8])
            print(f"  #{attempt:3d}: hdr=0x{hdr:04X} X={x:+6d} Y={y:+6d} Z={z:+6d}  [{raw}]")
        else:
            print(f"  #{attempt:3d}: short {len(data) if data else 0}b")
    except usb.core.USBTimeoutError:
        if attempt == 0:
            print("  timeout (no data from EP)")
        pass
    except usb.core.USBError as e:
        print(f"  USB error: {e}")
        break

print("Done")
