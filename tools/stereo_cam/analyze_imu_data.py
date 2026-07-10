#!/usr/bin/env python3
"""Analyze interrupt EP 0x82 data from USBPcap to find IMU values"""

import struct, sys

filepath = sys.argv[1] if len(sys.argv) > 1 else r"D:\Carl_WorkStation\rov_ros2\tools\stereo_cam\ylx_capture.pcapng"

with open(filepath, "rb") as f:
    data = f.read()

print(f"File: {filepath}")
print(f"Size: {len(data)} bytes")

# Parse pcap global header
magic = struct.unpack_from("<I", data, 0)[0]
ver_major, ver_minor = struct.unpack_from("<HH", data, 4)
tz = struct.unpack_from("<i", data, 8)[0]
linktype = struct.unpack_from("<I", data, 20)[0]

print(f"Magic: 0x{magic:08X}, Version: {ver_major}.{ver_minor}")
print(f"LinkType: {linktype} ({'USB_2_0' if linktype == 249 else 'USB_FULL_SPEED' if linktype == 220 else 'unknown'})")

pos = 24
frame_num = 0
interrupt_frames = []

while pos < len(data) - 16:
    ts_sec = struct.unpack_from("<I", data, pos)[0]
    ts_usec = struct.unpack_from("<I", data, pos + 4)[0]
    incl_len = struct.unpack_from("<I", data, pos + 8)[0]
    orig_len = struct.unpack_from("<I", data, pos + 12)[0]

    frame_start = pos + 16
    if frame_start + incl_len > len(data):
        break

    frame_data = data[frame_start : frame_start + incl_len]

    # USB 2.0 header = 27 bytes (USBPcap specific)
    # Fields: header_len(u16), irp_id(u64), status(u32), function(u16), info(u8),
    #         bus(u16), device(u16), endpoint(u8), transfer_type(u8), data_len(u32)
    if len(frame_data) >= 27:
        header_len = struct.unpack_from("<H", frame_data, 0)[0]
        if header_len >= 27 and incl_len > header_len:
            ep = frame_data[21]  # endpoint
            transfer_type = frame_data[23]  # transfer_type

            if transfer_type == 3 and ep == 0x82:
                payload = frame_data[header_len:]
                if len(payload) >= 8:
                    interrupt_frames.append(payload[:8])

    frame_num += 1
    pos += 16 + incl_len

print(f"\nTotal frames: {frame_num}")
print(f"Interrupt EP 0x82 packets: {len(interrupt_frames)}")

if not interrupt_frames:
    print("No interrupt EP 0x82 packets found!")
    sys.exit(0)

# Show first 20 packets
print("\n=== First 20 Interrupt Packets ===")
for i, pkt in enumerate(interrupt_frames[:20]):
    hex_str = " ".join(f"{b:02X}" for b in pkt)
    print(f"  #{i:3d}: {hex_str}")

# Count unique values
unique = set(bytes(p) for p in interrupt_frames)
print(f"\nUnique packet values: {len(unique)} out of {len(interrupt_frames)}")

# Show non-zero packets
non_zero = [p for p in interrupt_frames if any(b != 0 for b in p)]
print(f"Non-zero packets: {len(non_zero)}")

if non_zero:
    print("\n=== Non-zero packets (first 20) ===")
    for i, pkt in enumerate(non_zero[:20]):
        hex_str = " ".join(f"{b:02X}" for b in pkt)
        print(f"  #{i}: {hex_str}")

# Parse as IMU data: 3 axes x 2 bytes, high 12 bits valid (low 4 bits unused)
print("\n=== Parsing as IMU (high 12-bit, signed?) ===")
# Try both unsigned and signed
sample_count = 0
for pkt in interrupt_frames:
    if len(pkt) < 6:
        continue
    x_raw = struct.unpack_from("<H", pkt, 0)[0]
    y_raw = struct.unpack_from("<H", pkt, 2)[0]
    z_raw = struct.unpack_from("<H", pkt, 4)[0]

    # Method 1: high 12 bits unsigned
    xu = (x_raw >> 4) & 0xFFF
    yu = (y_raw >> 4) & 0xFFF
    zu = (z_raw >> 4) & 0xFFF

    # Method 2: high 12 bits signed
    xs = (x_raw >> 4) & 0xFFF
    if xs >= 0x800:
        xs -= 0x1000
    ys = (y_raw >> 4) & 0xFFF
    if ys >= 0x800:
        ys -= 0x1000
    zs = (z_raw >> 4) & 0xFFF
    if zs >= 0x800:
        zs -= 0x1000

    if any(v != 0 for v in [xu, yu, zu]):
        print(f"  #{sample_count}: unsigned=[{xu:4d}, {yu:4d}, {zu:4d}]  signed=[{xs:+5d}, {ys:+5d}, {zs:+5d}]  raw: {x_raw:04X} {y_raw:04X} {z_raw:04X}")
        sample_count += 1
        if sample_count >= 20:
            break

if sample_count == 0:
    print("  ALL ZERO - IMU not active on Windows either")
    # Check if maybe data is in different format
    print("\n=== Alternative format analysis ===")
    # Check last 2 bytes
    for i, pkt in enumerate(interrupt_frames[:5]):
        rest = pkt[6:]
        print(f"  #{i}: bytes 6-7: {' '.join(f'{b:02X}' for b in rest)}")
else:
    print(f"\nFound {sample_count} non-zero samples!")
    # Show value ranges
    all_xu, all_yu, all_zu = [], [], []
    for pkt in interrupt_frames:
        if len(pkt) >= 6:
            x_raw = struct.unpack_from("<H", pkt, 0)[0]
            y_raw = struct.unpack_from("<H", pkt, 2)[0]
            z_raw = struct.unpack_from("<H", pkt, 4)[0]
            all_xu.append((x_raw >> 4) & 0xFFF)
            all_yu.append((y_raw >> 4) & 0xFFF)
            all_zu.append((z_raw >> 4) & 0xFFF)

    non_zero_xu = [v for v in all_xu if v != 0]
    non_zero_yu = [v for v in all_yu if v != 0]
    non_zero_zu = [v for v in all_zu if v != 0]

    if non_zero_xu:
        print(f"  X range: {min(non_zero_xu)} ~ {max(non_zero_xu)} ({len(non_zero_xu)} non-zero)")
    if non_zero_yu:
        print(f"  Y range: {min(non_zero_yu)} ~ {max(non_zero_yu)} ({len(non_zero_yu)} non-zero)")
    if non_zero_zu:
        print(f"  Z range: {min(non_zero_zu)} ~ {max(non_zero_zu)} ({len(non_zero_zu)} non-zero)")

# Also try 8-byte format: frame_num(u16) + 3 axis(u16 each)?
print("\n=== 8-byte format: frame_num(u16) + X(u16) + Y(u16) + Z(u16) ===")
for i, pkt in enumerate(interrupt_frames[:10]):
    fn = struct.unpack_from("<H", pkt, 0)[0]
    xf = struct.unpack_from("<H", pkt, 2)[0]
    yf = struct.unpack_from("<H", pkt, 4)[0]
    zf = struct.unpack_from("<H", pkt, 6)[0]
    print(f"  #{i}: frame={fn:5d} X={xf:5d} Y={yf:5d} Z={zf:5d}")
