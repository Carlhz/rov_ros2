#!/usr/bin/env python3
"""
YLX Camera IMU Reader - Linux bypass uvcvideo
Reads gyroscope data directly from EP 0x87 (interrupt endpoint) via libusb.

IMU data format (8 bytes):
  [0:2] Header = 0x0002
  [2:4] X-axis gyro (int16 LE)
  [4:6] Y-axis gyro (int16 LE)  
  [6:8] Z-axis gyro (int16 LE)

Strategy:
  1. Find device by VID:PID (1BCF:0B15)
  2. Detach kernel driver from IMU interface (NOT video interface)
  3. Read interrupt endpoint EP 0x87
"""

import usb.core
import usb.util
import struct
import time
import sys

VID = 0x1BCF
PID = 0x0B15
IMU_EP_IN = 0x87  # Linux descriptor shows EP 0x87 as interrupt IN

def find_device():
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        print("ERROR: Device not found!")
        return None
    print(f"Device: {dev}")
    print(f"  Configurations: {dev.bNumConfigurations}")
    return dev

def analyze_configs(dev):
    """Show all configurations, interfaces, and endpoints"""
    for cfg in dev:
        print(f"\nConfig {cfg.bConfigurationValue}:")
        for iface_idx, iface in enumerate(cfg):
            for alt in iface:
                print(f"  Interface {iface_idx}, Alt {alt.bAlternateSetting}:")
                print(f"    Class={alt.bInterfaceClass:02X} Sub={alt.bInterfaceSubClass:02X} Proto={alt.bInterfaceProtocol:02X}")
                for ep in alt:
                    ep_addr = ep.bEndpointAddress
                    ep_type = ep.bmAttributes & 0x03
                    type_names = {0: "CTRL", 1: "ISOCH", 2: "BULK", 3: "INTR"}
                    direction = "IN" if ep_addr & 0x80 else "OUT"
                    print(f"    EP 0x{ep_addr:02X} ({direction}) {type_names.get(ep_type, '?')} max={ep.wMaxPacketSize}b interval={ep.bInterval}")
    return dev

def try_read_imu(dev):
    """Try multiple approaches to read IMU data"""
    
    # First, try to claim the interface with the interrupt endpoint
    # On Linux, EP 0x87 is on interface that has bInterfaceClass=0xFF (vendor-specific)
    # We need to find which interface number that is
    
    target_iface = None
    target_cfg = None
    
    for cfg in dev:
        for iface_idx, iface in enumerate(cfg):
            for alt in iface:
                for ep in alt:
                    if ep.bEndpointAddress == IMU_EP_IN:
                        target_iface = iface_idx
                        target_cfg = cfg.bConfigurationValue
                        print(f"\nFound EP 0x{IMU_EP_IN:02X} on config {target_cfg}, interface {target_iface}")
                        break
                if target_iface is not None:
                    break
            if target_iface is not None:
                break
        if target_iface is not None:
            break
    
    if target_iface is None:
        print(f"ERROR: EP 0x{IMU_EP_IN:02X} not found in any interface!")
        return False
    
    # Set configuration
    try:
        dev.set_configuration(target_cfg)
        print(f"Set config {target_cfg} OK")
    except Exception as e:
        print(f"Set config: {e}")
    
    # Try detaching kernel driver
    try:
        if dev.is_kernel_driver_active(target_iface):
            dev.detach_kernel_driver(target_iface)
            print(f"Detached kernel driver from interface {target_iface}")
    except Exception as e:
        print(f"Detach driver: {e}")
    
    # Claim interface
    try:
        usb.util.claim_interface(dev, target_iface)
        print(f"Claimed interface {target_iface}")
    except Exception as e:
        print(f"Claim interface failed: {e}")
        # Try alternate approach - claim with pyusb directly
        try:
            dev.claim_interface(target_iface)
            print(f"Claimed interface {target_iface} (direct)")
        except Exception as e2:
            print(f"Direct claim also failed: {e2}")
            return False
    
    # Now try reading interrupt endpoint
    print(f"\nReading IMU data from EP 0x{IMU_EP_IN:02X}...")
    print("(Press Ctrl+C to stop)")
    
    count = 0
    errors = 0
    start = time.time()
    last_print = start
    
    try:
        while True:
            try:
                data = dev.read(IMU_EP_IN, 8, timeout=1000)
                if data and len(data) == 8:
                    header = struct.unpack('<H', data[0:2])[0]
                    x = struct.unpack('<h', data[2:4])[0]
                    y = struct.unpack('<h', data[4:6])[0]
                    z = struct.unpack('<h', data[6:8])[0]
                    count += 1
                    
                    now = time.time()
                    if now - last_print >= 1.0:
                        elapsed = now - start
                        rate = count / elapsed if elapsed > 0 else 0
                        print(f"  [{elapsed:5.1f}s] #{count:5d} rate={rate:5.1f}Hz  "
                              f"X={x:+6d} Y={y:+6d} Z={z:+6d}  "
                              f"header=0x{header:04X}  "
                              f"raw={' '.join(f'{b:02X}' for b in data)}")
                        last_print = now
                else:
                    print(f"Short read: {len(data) if data else 0} bytes")
                    
            except usb.core.USBTimeoutError:
                pass  # normal timeout, retry
            except usb.core.USBError as e:
                errors += 1
                if errors > 10:
                    print(f"Too many USB errors, stopping")
                    break
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        print("\nStopped by user")
    
    elapsed = time.time() - start
    rate = count / elapsed if elapsed > 0 else 0
    print(f"\nTotal: {count} packets in {elapsed:.1f}s, rate={rate:.1f}Hz, errors={errors}")
    
    # Cleanup
    try:
        usb.util.release_interface(dev, target_iface)
        dev.attach_kernel_driver(target_iface)
    except:
        pass
    
    return count > 0


def try_claim_all(dev):
    """Try detaching ALL interfaces, then reading"""
    print("\n=== Alternative: claim all interfaces ===")
    
    # Set config first
    for cfg in dev:
        try:
            dev.set_configuration(cfg.bConfigurationValue)
            print(f"Set config {cfg.bConfigurationValue}")
            break
        except:
            continue
    
    # Detach ALL kernel drivers
    for cfg in dev:
        for iface_idx, iface in enumerate(cfg):
            try:
                if dev.is_kernel_driver_active(iface_idx):
                    dev.detach_kernel_driver(iface_idx)
                    print(f"  Detached iface {iface_idx}")
            except Exception as e:
                print(f"  Skip iface {iface_idx}: {e}")
    
    # Try reading from all IN interrupt endpoints
    for cfg in dev:
        for iface_idx, iface in enumerate(cfg):
            for alt in iface:
                for ep in alt:
                    if (ep.bEndpointAddress & 0x80) and (ep.bmAttributes & 0x03) == 3:
                        ep_addr = ep.bEndpointAddress
                        max_pkt = ep.wMaxPacketSize
                        print(f"\nTrying EP 0x{ep_addr:02X} (max={max_pkt})...")
                        try:
                            usb.util.claim_interface(dev, iface_idx)
                            data = dev.read(ep_addr, min(max_pkt, 64), timeout=2000)
                            if data:
                                print(f"  GOT {len(data)} bytes: {' '.join(f'{b:02X}' for b in data)}")
                            usb.util.release_interface(dev, iface_idx)
                        except usb.core.USBTimeoutError:
                            print(f"  Timeout (no data)")
                            usb.util.release_interface(dev, iface_idx)
                        except Exception as e:
                            print(f"  Error: {e}")
    
    # Re-attach
    for cfg in dev:
        for iface_idx in range(cfg.bNumInterfaces if hasattr(cfg, 'bNumInterfaces') else 5):
            try:
                dev.attach_kernel_driver(iface_idx)
            except:
                pass


if __name__ == '__main__':
    dev = find_device()
    if dev is None:
        sys.exit(1)
    
    analyze_configs(dev)
    
    # Try main approach
    ok = try_read_imu(dev)
    
    if not ok:
        print("\nMain approach failed, trying alternatives...")
        try_claim_all(dev)
