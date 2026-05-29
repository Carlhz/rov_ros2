# INS 解析数据消息 - ROS2 Foxy
# 对应 ins_demo_v1.9.py 解析的 0x50 帧数据

std_msgs/Header header

# 位置信息
float64 latitude      # 纬度（度）
float64 longitude     # 经度（度）
float32 altitude      # 高度（米）

# 姿态信息
float32 roll          # 横滚角（度）
float32 pitch         # 俯仰角（度）
float32 yaw           # 航向角（度）

# 速度信息
float32 north_vel     # 北向速度（m/s）
float32 east_vel      # 东向速度（m/s）
float32 down_vel      # 向下速度（m/s）

# 角速度
float32 gyro_x        # X轴角速度（deg/s）
float32 gyro_y        # Y轴角速度（deg/s）
float32 gyro_z        # Z轴角速度（deg/s）

# 加速度
float32 acc_x         # X轴加速度（m/s²）
float32 acc_y         # Y轴加速度（m/s²）
float32 acc_z         # Z轴加速度（m/s²）

# 状态信息
uint8   work_status           # 工作状态字节
uint8   dvl_calib_status      # DVL校准状态
uint8   gnss_pos_status       # GNSS定位状态
uint8   combination_status    # 组合导航状态

# 状态描述（人类可读）
string  work_status_desc      # 工作状态描述
string  dvl_calib_desc        # DVL校准描述
string  gnss_pos_desc         # GNSS定位描述
string  combination_desc      # 组合导航描述

# 原始数据
uint8[] raw_frame             # 原始 0x50 帧数据（可选）
bool    valid                 # 数据是否有效
