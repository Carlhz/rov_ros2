#!/usr/bin/env python3
"""
INS Driver - Full 202-byte frame parser
Frame layout (bytes 0-201):
  0-1:   Frame header 0x5A 0xA5
  2:     Work status byte
  3:     DVL calibration status
  4:     GNSS fix type
  5:     GNSS satellite count
  6:     GNSS position update seq
  7:     GNSS heading update seq
  8:     DVL update seq
  9-12:  Wx angular rate (deg/s)
 13-16:  Wy angular rate (deg/s)
 17-20:  Wz angular rate (deg/s)
 21-24:  Ax acceleration (m/s^2)
 25-28:  Ay acceleration (m/s^2)
 29-32:  Az acceleration (m/s^2)
 33-36:  Pitch (deg)
 37-40:  Roll (deg)
 41-44:  Yaw/Heading (deg)
 45-48:  East velocity (m/s)
 49-52:  North velocity (m/s)
 53-56:  Down velocity (m/s)
 57-60:  Track angle (deg)
 61-64:  GNSS UTC date ddmmyy
 65-68:  GNSS UTC time hhmmss
 69-72:  GNSS east velocity (m/s)
 73-76:  GNSS north velocity (m/s)
 77-80:  GNSS altitude (m)
 81-84:  GNSS speed (m/s)
 85-88:  GNSS track angle (deg)
 89-92:  GNSS HDOP
 93-96:  GNSS dual-antenna heading (deg)
 97-100: GNSS position update period (s)
101-104: GNSS heading update period (s)
105-108: Sway velocity (m/s)
109-112: Surge velocity (m/s)
113-116: Heave velocity (m/s)
117-120: Sway (m)
121-124: Surge (m)
125-128: Heave (m)
129-132: Horizontal acceleration (m/s^2)
133-136: Vertical acceleration (m/s^2)
137-140: DVL longitudinal velocity (m/s)
141-144: DVL lateral velocity (m/s)
145-148: DVL downward velocity (m/s)
149-152: DVL odometry (m)
153-156: DVL depth (m)
157-160: DVL height (m)
161-164: DVL-INS X angle (deg)
165-168: DVL-INS Y angle (deg)
169-172: DVL-INS Z angle (deg)
173-176: DVL update period (s)
177-180: INS latitude (x0.0000001 deg, int32 LE)
181-184: INS longitude (x0.0000001 deg, int32 LE)
185-188: GNSS latitude (x0.0000001 deg, int32 LE)
189-192: GNSS longitude (x0.0000001 deg, int32 LE)
193:     GNSS std lat (m)
194:     GNSS std lon (m)
195:     GNSS std alt (m)
196:     Calibration sequence
197:     Combined status bits
198:     Internal temperature (C)
199:     TX sequence (0-255)
200:     Checksum (XOR bytes 2-199)
201:     Frame tail 0x55
"""

import socket, struct, threading, time, sys

sys.path.insert(0, '/opt/ros/foxy/lib/python3.8/site-packages')
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32, String, UInt8, Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

LOCAL_IP  = '192.168.0.99'
LOCAL_PORT = 8008
INS_IP    = '192.168.0.7'
INS_CMD_PORT = 8007
START_CMD = b'\x5A\xA5\x47\x01\x01\x00\x00\x47\x55'
STOP_CMD  = b'\x5A\xA5\x47\x00\x01\x00\x00\x46\x55'
FRAME_LEN = 202
FRAME_HEADER = b'\x5A\xA5'
FRAME_TAIL   = 0x55


def unpack_f(data, offset):
    return struct.unpack('<f', data[offset:offset+4])[0]

def unpack_i32(data, offset):
    return struct.unpack('<i', data[offset:offset+4])[0]


class INSDriverFull(Node):
    def __init__(self):
        super().__init__('ins_driver_full')
        self.get_logger().info('INS Full Driver starting...')

        # Publishers
        self.pub_lat       = self.create_publisher(Float64, '/ins/latitude',    10)
        self.pub_lon       = self.create_publisher(Float64, '/ins/longitude',   10)
        self.pub_alt       = self.create_publisher(Float64, '/ins/altitude',    10)
        self.pub_pose      = self.create_publisher(Vector3, '/ins/pose',        10)
        self.pub_twist     = self.create_publisher(Vector3, '/ins/twist',       10)
        self.pub_imu       = self.create_publisher(Imu,     '/ins/imu',         10)
        self.pub_status    = self.create_publisher(Int32,   '/ins/status',      10)
        self.pub_raw       = self.create_publisher(String,  '/ins/raw',         10)

        # Extended publishers
        self.pub_work_status    = self.create_publisher(Int32,   '/ins/work_status',      10)
        self.pub_align_status   = self.create_publisher(Int32,   '/ins/align_status',     10)
        self.pub_gnss_fix       = self.create_publisher(Int32,   '/ins/gnss_fix_type',    10)
        self.pub_gnss_sats      = self.create_publisher(Int32,   '/ins/gnss_satellites',  10)
        self.pub_gnss_hdop      = self.create_publisher(Float64, '/ins/gnss_hdop',        10)
        self.pub_gnss_heading   = self.create_publisher(Float64, '/ins/gnss_heading',     10)
        self.pub_gnss_lat       = self.create_publisher(Float64, '/ins/gnss_latitude',    10)
        self.pub_gnss_lon       = self.create_publisher(Float64, '/ins/gnss_longitude',   10)
        self.pub_gnss_alt       = self.create_publisher(Float64, '/ins/gnss_altitude',    10)
        self.pub_gnss_speed     = self.create_publisher(Float64, '/ins/gnss_speed',       10)
        self.pub_gnss_track     = self.create_publisher(Float64, '/ins/gnss_track_angle', 10)
        self.pub_track_angle    = self.create_publisher(Float64, '/ins/track_angle',      10)
        self.pub_heave          = self.create_publisher(Vector3, '/ins/heave',            10)
        self.pub_dvl_vel        = self.create_publisher(Vector3, '/ins/dvl_velocity',     10)
        self.pub_dvl_depth      = self.create_publisher(Float64, '/ins/dvl_depth',        10)
        self.pub_temperature    = self.create_publisher(Int32,   '/ins/temperature',      10)
        self.pub_comb_status    = self.create_publisher(Int32,   '/ins/combined_status',  10)
        self.pub_calib_seq      = self.create_publisher(Int32,   '/ins/calib_sequence',   10)
        self.pub_gnss_std       = self.create_publisher(Vector3, '/ins/gnss_std',         10)

        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((LOCAL_IP, LOCAL_PORT))
        self.sock.settimeout(1.0)

        self.frame_count = 0
        self.error_count = 0
        self.t_start = time.time()

        # Send start command immediately
        self._send_start()

        # Start receive thread
        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

        # Keepalive timer every 10s
        self.create_timer(10.0, self._send_start)
        # Stats log every 5s
        self.create_timer(5.0, self._log_stats)

        self.get_logger().info(f'INS Full Driver ready, listening on {LOCAL_IP}:{LOCAL_PORT}')

    def _send_start(self):
        try:
            self.sock.sendto(START_CMD, (INS_IP, INS_CMD_PORT))
            self.get_logger().info('START_CMD sent to INS')
        except Exception as e:
            self.get_logger().warn(f'Failed to send START_CMD: {e}')

    def _recv_loop(self):
        buf = b''
        while rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(4096)
                buf += data
                # Search for frame header
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
                    # Validate tail
                    if frame[201] == FRAME_TAIL:
                        # Validate checksum
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
                            self.get_logger().debug(f'Checksum mismatch: expected {chk:#x}, got {frame[200]:#x}')
                            self.error_count += 1
                        buf = buf[FRAME_LEN:]
                    else:
                        # Bad tail, skip one byte
                        buf = buf[1:]
            except socket.timeout:
                continue
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().warn(f'Recv error: {e}')

    def _publish_frame(self, f):
        now = self.get_clock().now().to_msg()

        # --- Status bytes ---
        work_byte   = f[2]
        dvl_calib   = f[3]
        gnss_fix    = f[4]
        gnss_sats   = f[5]
        gnss_pos_seq= f[6]
        gnss_hdg_seq= f[7]
        dvl_seq     = f[8]

        align_status = work_byte & 0x03   # bit1..0
        gyro_overrange = (work_byte >> 7) & 1
        gyro_temp_err  = (work_byte >> 6) & 1
        acc_overrange  = (work_byte >> 5) & 1
        acc_temp_err   = (work_byte >> 4) & 1
        param_fail     = (work_byte >> 3) & 1
        calib_fail     = (work_byte >> 2) & 1

        # --- IMU ---
        wx = unpack_f(f, 9)
        wy = unpack_f(f, 13)
        wz = unpack_f(f, 17)
        ax = unpack_f(f, 21)
        ay = unpack_f(f, 25)
        az = unpack_f(f, 29)

        # --- Attitude ---
        pitch = unpack_f(f, 33)
        roll  = unpack_f(f, 37)
        yaw   = unpack_f(f, 41)

        # --- INS velocity ---
        ve = unpack_f(f, 45)
        vn = unpack_f(f, 49)
        vd = unpack_f(f, 53)
        track_angle = unpack_f(f, 57)

        # --- GNSS time ---
        gnss_date = unpack_f(f, 61)   # ddmmyy
        gnss_time = unpack_f(f, 65)   # hhmmss

        # --- GNSS velocity & position ---
        gnss_ve    = unpack_f(f, 69)
        gnss_vn    = unpack_f(f, 73)
        gnss_alt   = unpack_f(f, 77)
        gnss_speed = unpack_f(f, 81)
        gnss_track = unpack_f(f, 85)
        gnss_hdop  = unpack_f(f, 89)
        gnss_hdg   = unpack_f(f, 93)
        gnss_pos_period = unpack_f(f, 97)
        gnss_hdg_period = unpack_f(f, 101)

        # --- Heave/Sway/Surge ---
        sway_vel  = unpack_f(f, 105)
        surge_vel = unpack_f(f, 109)
        heave_vel = unpack_f(f, 113)
        sway      = unpack_f(f, 117)
        surge     = unpack_f(f, 121)
        heave     = unpack_f(f, 125)

        # --- Acceleration ---
        horiz_acc = unpack_f(f, 129)
        vert_acc  = unpack_f(f, 133)

        # --- DVL ---
        dvl_lon_vel = unpack_f(f, 137)
        dvl_lat_vel = unpack_f(f, 141)
        dvl_down_vel= unpack_f(f, 145)
        dvl_odometry= unpack_f(f, 149)
        dvl_depth   = unpack_f(f, 153)
        dvl_height  = unpack_f(f, 157)
        dvl_ang_x   = unpack_f(f, 161)
        dvl_ang_y   = unpack_f(f, 165)
        dvl_ang_z   = unpack_f(f, 169)
        dvl_period  = unpack_f(f, 173)

        # --- Position ---
        ins_lat = unpack_i32(f, 177) * 0.0000001
        ins_lon = unpack_i32(f, 181) * 0.0000001
        gnss_lat= unpack_i32(f, 185) * 0.0000001
        gnss_lon= unpack_i32(f, 189) * 0.0000001

        # --- GNSS standard deviations ---
        gnss_std_lat = f[193]
        gnss_std_lon = f[194]
        gnss_std_alt = f[195]

        # --- Combined status ---
        calib_seq    = f[196]
        comb_status  = f[197]
        temperature  = f[198]
        tx_seq       = f[199]

        # comb_status bits
        gnss_fused = (comb_status >> 0) & 1
        odo_fused  = (comb_status >> 1) & 1
        dvl_fused  = (comb_status >> 2) & 1
        zupt_active= (comb_status >> 3) & 1

        # ===== Publish all topics =====

        # Legacy topics (keep backward compat)
        m = Float64(); m.data = ins_lat;  self.pub_lat.publish(m)
        m = Float64(); m.data = ins_lon;  self.pub_lon.publish(m)
        m = Float64(); m.data = float(gnss_alt); self.pub_alt.publish(m)

        v = Vector3(); v.x = float(roll); v.y = float(pitch); v.z = float(yaw)
        self.pub_pose.publish(v)

        v = Vector3(); v.x = float(vn); v.y = float(ve); v.z = float(vd)
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

        st = Int32(); st.data = int(comb_status); self.pub_status.publish(st)

        # Extended topics
        ws = Int32(); ws.data = int(work_byte);   self.pub_work_status.publish(ws)
        al = Int32(); al.data = int(align_status);self.pub_align_status.publish(al)
        gf = Int32(); gf.data = int(gnss_fix);    self.pub_gnss_fix.publish(gf)
        gs = Int32(); gs.data = int(gnss_sats);   self.pub_gnss_sats.publish(gs)

        m = Float64(); m.data = float(gnss_hdop); self.pub_gnss_hdop.publish(m)
        m = Float64(); m.data = float(gnss_hdg);  self.pub_gnss_heading.publish(m)
        m = Float64(); m.data = gnss_lat;         self.pub_gnss_lat.publish(m)
        m = Float64(); m.data = gnss_lon;         self.pub_gnss_lon.publish(m)
        m = Float64(); m.data = float(gnss_alt);  self.pub_gnss_alt.publish(m)
        m = Float64(); m.data = float(gnss_speed);self.pub_gnss_speed.publish(m)
        m = Float64(); m.data = float(gnss_track);self.pub_gnss_track.publish(m)
        m = Float64(); m.data = float(track_angle);self.pub_track_angle.publish(m)

        v = Vector3(); v.x = float(sway); v.y = float(surge); v.z = float(heave)
        self.pub_heave.publish(v)

        v = Vector3(); v.x = float(dvl_lon_vel); v.y = float(dvl_lat_vel); v.z = float(dvl_down_vel)
        self.pub_dvl_vel.publish(v)

        m = Float64(); m.data = float(dvl_depth); self.pub_dvl_depth.publish(m)

        t = Int32(); t.data = int(temperature);   self.pub_temperature.publish(t)
        c = Int32(); c.data = int(comb_status);   self.pub_comb_status.publish(c)
        cs= Int32(); cs.data= int(calib_seq);     self.pub_calib_seq.publish(cs)

        v = Vector3(); v.x = float(gnss_std_lat); v.y = float(gnss_std_lon); v.z = float(gnss_std_alt)
        self.pub_gnss_std.publish(v)

        # Raw summary string
        ALIGN_NAMES = {0:'监控', 1:'粗对准', 2:'精对准', 3:'INS导航'}
        FIX_NAMES   = {0:'未定位', 1:'单点SPS', 2:'差分DGNSS', 4:'RTK固定', 5:'RTK浮动'}
        raw = (f'seq:{tx_seq} align:{ALIGN_NAMES.get(align_status,str(align_status))} '
               f'fix:{FIX_NAMES.get(gnss_fix,str(gnss_fix))} sats:{gnss_sats} hdop:{gnss_hdop:.1f} '
               f'lat:{ins_lat:.7f} lon:{ins_lon:.7f} alt:{gnss_alt:.2f} '
               f'r:{roll:.2f} p:{pitch:.2f} y:{yaw:.2f} '
               f'vn:{vn:.3f} ve:{ve:.3f} vd:{vd:.3f} '
               f'temp:{temperature}C comb:{comb_status:#04x}')
        s = String(); s.data = raw; self.pub_raw.publish(s)

    def _log_stats(self):
        elapsed = time.time() - self.t_start
        rate = self.frame_count / elapsed if elapsed > 0 else 0
        self.get_logger().info(
            f'Frames: {self.frame_count}  Rate: {rate:.1f}Hz  Errors: {self.error_count}')

    def destroy_node(self):
        try:
            self.sock.sendto(STOP_CMD, (INS_IP, INS_CMD_PORT))
            self.get_logger().info('STOP_CMD sent')
        except Exception:
            pass
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = INSDriverFull()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
