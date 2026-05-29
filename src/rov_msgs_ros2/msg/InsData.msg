# InsData.msg — INS 202字节帧解析后的完整数据
std_msgs/Header header

# ── 系统状态 ───────────────────────────────
uint8 work_status              # 工作状态
uint8 dvl_calib_status         # DVL 校准状态
uint8 gnss_pos_status          # GNSS 定位状态
uint8 gnss_satellites          # GNSS 卫星数
uint8 gnss_heading_status      # GNSS 航向状态
uint8 dvl_status               # DVL 状态

# ── IMU 数据 ───────────────────────────────
float32 angular_velocity_x     # 角速度 X (deg/s)
float32 angular_velocity_y     # 角速度 Y (deg/s)
float32 angular_velocity_z     # 角速度 Z (deg/s)

float32 linear_acceleration_x  # 加速度 X (m/s²)
float32 linear_acceleration_y  # 加速度 Y (m/s²)
float32 linear_acceleration_z  # 加速度 Z (m/s²)

# ── 姿态角 (deg) ───────────────────────────
float32 pitch
float32 roll
float32 yaw

# ── 导航速度 ───────────────────────────────
float32 velocity_east          # 东向速度 (m/s)
float32 velocity_north         # 北向速度 (m/s)
float32 speed_over_ground      # 对地速度 (m/s)
float32 track_angle            # 航迹角 (deg)

# ── GNSS ──────────────────────────────────
int32   gnss_date
int32   gnss_time
float32 gnss_velocity_east
float32 gnss_velocity_north
float32 gnss_altitude
float32 gnss_speed
float32 gnss_track_angle
float32 gnss_hdop
float32 gnss_dual_ant_yaw
float32 gnss_pos_update_period

# ── DVL ───────────────────────────────────
float32 dvl_sway_velocity      # 横移速度 (m/s)
float32 dvl_surge_velocity     # 纵移速度 (m/s)
float32 dvl_heave_velocity     # 升沉速度 (m/s)
float32 dvl_sway_distance      # 横移位移 (m)
float32 dvl_surge_distance     # 纵移位移 (m)
float32 dvl_heave_distance     # 升沉位移 (m)
float32 dvl_distance_to_bottom # 距底距离 (m)

# ── 组合导航状态 ───────────────────────────
uint8 combination_status
bool  gnss_valid
bool  velocity_valid
bool  dvl_valid
bool  pressure_valid

# ── 帧计数 ────────────────────────────────
uint64 frame_count
