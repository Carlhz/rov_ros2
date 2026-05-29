# InsCommand.msg — 上位机发给 RK3588 的 INS 控制命令
uint8 CMD_START = 1
uint8 CMD_STOP  = 2
uint8 CMD_RESET = 3

uint8 command    # 命令类型
string param     # 可选参数
