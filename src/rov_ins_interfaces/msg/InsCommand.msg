# INS 控制命令消息
# 用于从上位机发送控制指令到 RK3588

std_msgs/Header header

string command    # 命令类型: "start", "stop", "set_lat", "set_lon", "pause", "resume"
float64 latitude  # 用于 set_lat 命令
float64 longitude # 用于 set_lon 命令
