#!/usr/bin/env python3
"""
INS Driver with ROS2 Control Service
Runs on RK3588

Features:
1. Receive INS data via UDP (8008) and publish to ROS2
2. Provide /ins/control service to accept commands from VM
3. Forward commands to INS via UDP (8007)

Commands:
- STOP: Enter monitor mode
- SET_POS: Set initial position (lat/lon/alt)
- START: Start alignment
"""

import socket
import struct
import threading
import time
import sys

sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')
sys.path.insert(0, '/opt/ros/foxy/lib/python3.8/site-packages')

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32, String, UInt8, Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

# Import service interface
try:
    from rov_ins_interface.srv import INSCommand
except ImportError:
    print("Warning: rov_ins_interface not found, service will not be available")
    INSCommand = None

# Network config
LOCAL_IP = '192.168.0.99'
LOCAL_PORT = 8008
INS_IP = '192.168.0.7'
INS_CMD_PORT = 8007

# INS Commands
START_CMD = b'\x5A\xA5\x47\x01\x01\x00\x00\x47\x55'
STOP_CMD = b'\x5A\xA5\x47\x00\x01\x00\x00\x46\x55'

# Frame constants
FRAME_LEN = 202
FRAME_HEADER = b'\x5A\xA5'
FRAME_TAIL = 0x55

# Status mapping
STATUS_NAMES = {
    0: '监控',
    1: '粗对准',
    2: '精对准',
    3: 'INS导航'
}


def unpack_f(data, offset):
    return struct.unpack('<f', data[offset:offset+4])[0]


def unpack_i32(data, offset):
    return struct.unpack('<i', data[offset:offset+4])[0]


def pack_i32(value):
    return struct.pack('<i', int(value))


def pack_f(value):
    return struct.pack('<f', float(value))


class INSDriverControlled(Node):
    def __init__(self):
        super().__init__('ins_driver_controlled')
        self.get_logger().info('INS Driver with Control Service starting...')

        # Current INS state
        self.current_align_status = 0  # 0=monitor, 1=coarse, 2=fine, 3=nav
        self.current_work_status = 0
        self.ins_latitude = 0.0
        self.ins_longitude = 0.0

        # Setup publishers (same as before)
        self._setup_publishers()

        # Setup UDP sockets
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((LOCAL_IP, LOCAL_PORT))
        self.sock.settimeout(1.0)

        # Statistics
        self.frame_count = 0
        self.error_count = 0
        self.t_start = time.time()

        # Start receive thread
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        # Setup control service
        if INSCommand:
            self.srv = self.create_service(
                INSCommand, '/ins/control', self._handle_control_command)
            self.get_logger().info('Control service /ins/control ready')
        else:
            self.get_logger().warn('INSCommand service not available')

        # Stats timer
        self.create_timer(5.0, self._log_stats)

        self.get_logger().info(f'Driver ready, listening on {LOCAL_IP}:{LOCAL_PORT}')
        self.get_logger().info('NOTE: INS control commands must be sent manually via /ins/control service')

    def _setup_publishers(self):
        """Setup all ROS2 publishers"""
        # Legacy topics
        self.pub_lat = self.create_publisher(Float64, '/ins/latitude', 10)
        self.pub_lon = self.create_publisher(Float64, '/ins/longitude', 10)
        self.pub_alt = self.create_publisher(Float64, '/ins/altitude', 10)
        self.pub_pose = self.create_publisher(Vector3, '/ins/pose', 10)
        self.pub_twist = self.create_publisher(Vector3, '/ins/twist', 10)
        self.pub_imu = self.create_publisher(Imu, '/ins/imu', 10)
        self.pub_status = self.create_publisher(Int32, '/ins/status', 10)
        self.pub_raw = self.create_publisher(String, '/ins/raw', 10)

        # Extended topics
        self.pub_work_status = self.create_publisher(Int32, '/ins/work_status', 10)
        self.pub_align_status = self.create_publisher(Int32, '/ins/align_status', 10)
        self.pub_gnss_fix = self.create_publisher(Int32, '/ins/gnss_fix_type', 10)
        self.pub_gnss_sats = self.create_publisher(Int32, '/ins/gnss_satellites', 10)
        self.pub_gnss_hdop = self.create_publisher(Float64, '/ins/gnss_hdop', 10)
        self.pub_gnss_heading = self.create_publisher(Float64, '/ins/gnss_heading', 10)
        self.pub_gnss_lat = self.create_publisher(Float64, '/ins/gnss_latitude', 10)
        self.pub_gnss_lon = self.create_publisher(Float64, '/ins/gnss_longitude', 10)
        self.pub_gnss_alt = self.create_publisher(Float64, '/ins/gnss_altitude', 10)
        self.pub_gnss_speed = self.create_publisher(Float64, '/ins/gnss_speed', 10)
        self.pub_gnss_track = self.create_publisher(Float64, '/ins/gnss_track_angle', 10)
        self.pub_track_angle = self.create_publisher(Float64, '/ins/track_angle', 10)
        self.pub_heave = self.create_publisher(Vector3, '/ins/heave', 10)
        self.pub_dvl_vel = self.create_publisher(Vector3, '/ins/dvl_velocity', 10)
        self.pub_dvl_depth = self.create_publisher(Float64, '/ins/dvl_depth', 10)
        self.pub_temperature = self.create_publisher(Int32, '/ins/temperature', 10)
        self.pub_comb_status = self.create_publisher(Int32, '/ins/combined_status', 10)
        self.pub_calib_seq = self.create_publisher(Int32, '/ins/calib_sequence', 10)
        self.pub_gnss_std = self.create_publisher(Vector3, '/ins/gnss_std', 10)

    def _handle_control_command(self, request, response):
        """Handle control commands from VM"""
        cmd = request.command
        self.get_logger().info(f'Received command: {cmd}')

        try:
            if cmd == INSCommand.Request.CMD_STOP:
                # Stop INS, enter monitor mode
                self._send_udp_command(STOP_CMD)
                response.success = True
                response.message = "STOP command sent to INS"
                self.get_logger().info('Sent STOP_CMD to INS')

            elif cmd == INSCommand.Request.CMD_START:
                # Start INS alignment
                self._send_udp_command(START_CMD)
                response.success = True
                response.message = "START command sent to INS"
                self.get_logger().info('Sent START_CMD to INS')

            elif cmd == INSCommand.Request.CMD_SET_POS:
                # Set initial position
                lat = request.latitude
                lon = request.longitude
                alt = request.altitude

                # Build position set command
                # Format: 0x5A 0xA5 <cmd> <len> <lat> <lon> <alt> <checksum> 0x55
                cmd_bytes = self._build_setpos_command(lat, lon, alt)
                self._send_udp_command(cmd_bytes)

                response.success = True
                response.message = f"SET_POS command sent: lat={lat}, lon={lon}, alt={alt}"
                self.get_logger().info(f'Sent SET_POS: {lat}, {lon}, {alt}')

            elif cmd == INSCommand.Request.CMD_GET_STATUS:
                # Return current status
                response.success = True
                response.message = f"Current status: {STATUS_NAMES.get(self.current_align_status, 'Unknown')}"
                response.current_status = self.current_align_status

            else:
                response.success = False
                response.message = f"Unknown command: {cmd}"

        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
            self.get_logger().error(f'Command failed: {e}')

        return response

    def _build_setpos_command(self, lat, lon, alt):
        """Build position set command for INS

        Note: This is a placeholder implementation.
        Actual command format depends on INS protocol documentation.
        Common formats:
        - 0x5A 0xA5 0x48 <len> <lat_i32> <lon_i32> <alt_f> <chk> 0x55
        """
        # Convert lat/lon to int32 (1e-7 degrees)
        lat_i32 = int(lat * 1e7)
        lon_i32 = int(lon * 1e7)

        # Build command (placeholder - adjust per your INS protocol)
        cmd = bytearray()
        cmd.extend(b'\x5A\xA5')  # Header
        cmd.append(0x48)        # Command ID for set position
        cmd.append(12)          # Data length (3 x 4 bytes)
        cmd.extend(pack_i32(lat_i32))
        cmd.extend(pack_i32(lon_i32))
        cmd.extend(pack_f(alt))

        # Calculate checksum (XOR of bytes 2 to end-1)
        chk = 0
        for b in cmd[2:]:
            chk ^= b
        cmd.append(chk)
        cmd.append(0x55)        # Tail

        return bytes(cmd)

    def _send_udp_command(self, cmd_bytes):
        """Send command to INS via UDP"""
        try:
            self.sock.sendto(cmd_bytes, (INS_IP, INS_CMD_PORT))
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')
            raise

    def _recv_loop(self):
        """Receive INS data loop"""
        buf = b''
        while rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(4096)
                buf += data

                # Process complete frames
                while len(buf) >= FRAME_LEN:
                    idx = buf.find(FRAME_HEADER)
                    if idx == -1:
                        buf = buf[-1:]
                        break
                    if idx > 0:
                        buf = buf[idx:]
                    if len(buf) < FRAME_LEN:
                        break

                    frame = buf[:FRAME_LEN]

                    # Validate frame
                    if frame[201] == FRAME_TAIL:
                        chk = 0
                        for b in frame[2:200]:
                            chk ^= b
                        if chk == frame[200]:
                            try:
                                self._publish_frame(frame)
                                self.frame_count += 1
                            except Exception as e:
                                self.get_logger().warn(f'Parse error: {e}')
                                self.error_count += 1
                        else:
                            self.error_count += 1
                        buf = buf[FRAME_LEN:]
                    else:
                        buf = buf[1:]

            except socket.timeout:
                continue
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().warn(f'Recv error: {e}')

    def _publish_frame(self, f):
        """Parse and publish frame data"""
        now = self.get_clock().now().to_msg()

        # Parse key fields
        work_byte = f[2]
        gnss_fix = f[4]
        gnss_sats = f[5]

        align_status = work_byte & 0x03
        self.current_align_status = align_status
        self.current_work_status = work_byte

        # IMU
        wx = unpack_f(f, 9)
        wy = unpack_f(f, 13)
        wz = unpack_f(f, 17)
        ax = unpack_f(f, 21)
        ay = unpack_f(f, 25)
        az = unpack_f(f, 29)

        # Attitude
        pitch = unpack_f(f, 33)
        roll = unpack_f(f, 37)
        yaw = unpack_f(f, 41)

        # Velocity
        ve = unpack_f(f, 45)
        vn = unpack_f(f, 49)
        vd = unpack_f(f, 53)
        track_angle = unpack_f(f, 57)

        # GNSS
        gnss_alt = unpack_f(f, 77)
        gnss_speed = unpack_f(f, 81)
        gnss_track = unpack_f(f, 85)
        gnss_hdop = unpack_f(f, 89)
        gnss_hdg = unpack_f(f, 93)

        # Position
        ins_lat = unpack_i32(f, 177) * 0.0000001
        ins_lon = unpack_i32(f, 181) * 0.0000001
        gnss_lat = unpack_i32(f, 185) * 0.0000001
        gnss_lon = unpack_i32(f, 189) * 0.0000001

        self.ins_latitude = ins_lat
        self.ins_longitude = ins_lon

        # Combined status
        comb_status = f[197]
        temperature = f[198]
        tx_seq = f[199]

        # Heave/Sway/Surge
        sway_vel = unpack_f(f, 105)
        surge_vel = unpack_f(f, 109)
        heave_vel = unpack_f(f, 113)
        sway = unpack_f(f, 117)
        surge = unpack_f(f, 121)
        heave = unpack_f(f, 125)

        # DVL
        dvl_lon_vel = unpack_f(f, 137)
        dvl_lat_vel = unpack_f(f, 141)
        dvl_down_vel = unpack_f(f, 145)
        dvl_depth = unpack_f(f, 153)

        # GNSS std
        gnss_std_lat = f[193]
        gnss_std_lon = f[194]
        gnss_std_alt = f[195]

        # Publish legacy topics
        m = Float64()
        m.data = ins_lat
        self.pub_lat.publish(m)

        m = Float64()
        m.data = ins_lon
        self.pub_lon.publish(m)

        m = Float64()
        m.data = float(gnss_alt)
        self.pub_alt.publish(m)

        v = Vector3()
        v.x = float(roll)
        v.y = float(pitch)
        v.z = float(yaw)
        self.pub_pose.publish(v)

        v = Vector3()
        v.x = float(vn)
        v.y = float(ve)
        v.z = float(vd)
        self.pub_twist.publish(v)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'ins'
        imu.angular_velocity.x = float(wx)
        imu.angular_velocity.y = float(wy)
        imu.angular_velocity.z = float(wz)
        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)
        self.pub_imu.publish(imu)

        st = Int32()
        st.data = int(comb_status)
        self.pub_status.publish(st)

        # Extended topics
        ws = Int32()
        ws.data = int(work_byte)
        self.pub_work_status.publish(ws)

        al = Int32()
        al.data = int(align_status)
        self.pub_align_status.publish(al)

        gf = Int32()
        gf.data = int(gnss_fix)
        self.pub_gnss_fix.publish(gf)

        gs = Int32()
        gs.data = int(gnss_sats)
        self.pub_gnss_sats.publish(gs)

        m = Float64()
        m.data = float(gnss_hdop)
        self.pub_gnss_hdop.publish(m)

        m = Float64()
        m.data = float(gnss_hdg)
        self.pub_gnss_heading.publish(m)

        m = Float64()
        m.data = gnss_lat
        self.pub_gnss_lat.publish(m)

        m = Float64()
        m.data = gnss_lon
        self.pub_gnss_lon.publish(m)

        m = Float64()
        m.data = float(gnss_alt)
        self.pub_gnss_alt.publish(m)

        m = Float64()
        m.data = float(gnss_speed)
        self.pub_gnss_speed.publish(m)

        m = Float64()
        m.data = float(gnss_track)
        self.pub_gnss_track.publish(m)

        m = Float64()
        m.data = float(track_angle)
        self.pub_track_angle.publish(m)

        v = Vector3()
        v.x = float(sway)
        v.y = float(surge)
        v.z = float(heave)
        self.pub_heave.publish(v)

        v = Vector3()
        v.x = float(dvl_lon_vel)
        v.y = float(dvl_lat_vel)
        v.z = float(dvl_down_vel)
        self.pub_dvl_vel.publish(v)

        m = Float64()
        m.data = float(dvl_depth)
        self.pub_dvl_depth.publish(m)

        t = Int32()
        t.data = int(temperature)
        self.pub_temperature.publish(t)

        c = Int32()
        c.data = int(comb_status)
        self.pub_comb_status.publish(c)

        cs = Int32()
        cs.data = int(f[196])
        self.pub_calib_seq.publish(cs)

        v = Vector3()
        v.x = float(gnss_std_lat)
        v.y = float(gnss_std_lon)
        v.z = float(gnss_std_alt)
        self.pub_gnss_std.publish(v)

        # Raw summary
        FIX_NAMES = {0: '未定位', 1: '单点SPS', 2: '差分DGNSS', 4: 'RTK固定', 5: 'RTK浮动'}
        raw = (f'seq:{tx_seq} align:{STATUS_NAMES.get(align_status, str(align_status))} '
               f'fix:{FIX_NAMES.get(gnss_fix, str(gnss_fix))} sats:{gnss_sats} hdop:{gnss_hdop:.1f} '
               f'lat:{ins_lat:.7f} lon:{ins_lon:.7f} alt:{gnss_alt:.2f} '
               f'r:{roll:.2f} p:{pitch:.2f} y:{yaw:.2f} '
               f'vn:{vn:.3f} ve:{ve:.3f} vd:{vd:.3f} '
               f'temp:{temperature}C comb:{comb_status:#04x}')
        s = String()
        s.data = raw
        self.pub_raw.publish(s)

    def _log_stats(self):
        elapsed = time.time() - self.t_start
        rate = self.frame_count / elapsed if elapsed > 0 else 0
        self.get_logger().info(
            f'Frames: {self.frame_count} Rate: {rate:.1f}Hz Errors: {self.error_count} '
            f'Status: {STATUS_NAMES.get(self.current_align_status, "Unknown")}')

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = INSDriverControlled()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
