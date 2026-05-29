/*
 * can_demo_v1.7.c
 * Kinco iSMK CANopen 双电机 PDO 实时控制
 *
 * 相较 v1.6 的改进：
 *   - 速度控制命令由 SDO 方式改为 PDO 实时方式（RPDO1）
 *   - 新增 PDO 映射初始化（通过 SDO 在线配置）
 *   - 新增 TPDO1 接收：读取电机状态字 + 实际速度
 *   - NMT 报文管理（Pre-op / Operational）
 *
 * PDO 布局（异步模式，传输类型 254）：
 *   RPDO1  COB-ID = 0x200 + NodeID  →  [控制字 2B | 目标速度 4B] = 6B
 *   TPDO1  COB-ID = 0x180 + NodeID  ←  [状态字 2B | 实际速度 4B] = 6B
 *
 * NodeID:  NODE_LEFT=1 (0x601)  NODE_RIGHT=2 (0x602)
 *
 * 编译: gcc -O2 -o can_demo can_demo_v1.7.c
 * 运行需要 root 或 cap_net_admin 权限
 *
 * 用法:
 *   ./can_demo init              初始化 PDO（NMT + SDO 配置 + 使能）
 *   ./can_demo check             检查电机状态（是否就绪）
 *   ./can_demo move <L> <R>      左右电机转速(rpm)，纯 PDO
 *   ./can_demo forward 500       前进
 *   ./can_demo backward 300      后退
 *   ./can_demo stop              停止
 *   ./can_demo status            读取电机反馈（TPDO1）
 *   ./can_demo turn left|right <L> <R>  转弯
 *   ./can_demo pivot left|right <speed>  原地转向
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

/* ─────────────────────────────────────────────
   全局配置
   ───────────────────────────────────────────── */
#define CAN_INTERFACE   "can0"
#define CAN_BITRATE     500000
#define NODE_LEFT       1       /* COB-ID 基: 0x601 / 0x581 */
#define NODE_RIGHT      2       /* COB-ID 基: 0x602 / 0x582 */

/* 速度单位换算：DEC = RPM * 512 * 65536 / 1875  (分辨率 65536) */
#define RPM_TO_DEC(rpm)   ((int32_t)((long)(rpm) * 512L * 65536L / 1875L))

/* SDO 超时 ms */
#define SDO_TIMEOUT_MS  500

/* ─────────────────────────────────────────────
   SDO 帧构建
   ───────────────────────────────────────────── */
static void sdo_write_u8(uint8_t *f, uint16_t idx, uint8_t sub, uint8_t v)
{
    f[0] = 0x2F;
    f[1] = idx & 0xFF; f[2] = (idx >> 8) & 0xFF;
    f[3] = sub;
    f[4] = v; f[5] = 0; f[6] = 0; f[7] = 0;
}

static void sdo_write_u16(uint8_t *f, uint16_t idx, uint8_t sub, uint16_t v)
{
    f[0] = 0x2B;
    f[1] = idx & 0xFF; f[2] = (idx >> 8) & 0xFF;
    f[3] = sub;
    f[4] = v & 0xFF; f[5] = (v >> 8) & 0xFF; f[6] = 0; f[7] = 0;
}

static void sdo_write_u32(uint8_t *f, uint16_t idx, uint8_t sub, uint32_t v)
{
    f[0] = 0x23;
    f[1] = idx & 0xFF; f[2] = (idx >> 8) & 0xFF;
    f[3] = sub;
    f[4] = (v >>  0) & 0xFF; f[5] = (v >>  8) & 0xFF;
    f[6] = (v >> 16) & 0xFF; f[7] = (v >> 24) & 0xFF;
}

static void sdo_read_req(uint8_t *f, uint16_t idx, uint8_t sub)
{
    f[0] = 0x40;
    f[1] = idx & 0xFF; f[2] = (idx >> 8) & 0xFF;
    f[3] = sub;
    f[4] = 0; f[5] = 0; f[6] = 0; f[7] = 0;
}

/* ─────────────────────────────────────────────
   CAN 收发
   ───────────────────────────────────────────── */
static int send_frame(int sock, uint32_t id, const uint8_t *data, uint8_t dlc)
{
    struct can_frame f = {0};
    f.can_id  = id;
    f.can_dlc = dlc;
    memcpy(f.data, data, dlc);
    return write(sock, &f, sizeof(f));
}

/* 等待指定 COB-ID 的 SDO 响应，超时返回 -1 */
static int wait_sdo_resp(int sock, uint32_t rx_id, uint8_t *out)
{
    struct can_frame f;
    struct timespec t0, tn;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &tn);
        long ms = (tn.tv_sec - t0.tv_sec) * 1000
                + (tn.tv_nsec - t0.tv_nsec) / 1000000;
        if (ms > SDO_TIMEOUT_MS) return -1;

        fd_set fds; FD_ZERO(&fds); FD_SET(sock, &fds);
        struct timeval tv = {0, 2000};
        if (select(sock + 1, &fds, NULL, NULL, &tv) <= 0) continue;

        ssize_t n = read(sock, &f, sizeof(f));
        if (n > 0 && f.can_id == rx_id && f.can_dlc == 8) {
            uint8_t cs = f.data[0] & 0xE0;
            /* 0x60=写成功, 0x40=读响应, 0x80=abort */
            if (cs == 0x60 || cs == 0x40) {
                if (out) memcpy(out, f.data, 8);
                return 0;
            }
            if (f.data[0] == 0x80) {
                fprintf(stderr, "[SDO] Abort from 0x%03X: %02X%02X%02X%02X\n",
                        rx_id, f.data[7], f.data[6], f.data[5], f.data[4]);
                return -2;
            }
        }
    }
}

/* SDO 写并等待 ACK；失败打印警告但不退出 */
static int sdo_write_ack(int sock, uint8_t node, uint8_t *sdo_buf, const char *desc)
{
    uint32_t tx = 0x600 + node;
    uint32_t rx = 0x580 + node;
    uint16_t idx = (uint16_t)(sdo_buf[1] | (sdo_buf[2] << 8));
    uint8_t  sub = sdo_buf[3];
    int ret = send_frame(sock, tx, sdo_buf, 8);
    if (ret < 0) {
        fprintf(stderr, "  [SDO] TX fail 0x%03X 0x%04X:%02d (%s)\n", tx, idx, sub, desc);
        return -1;
    }
    ret = wait_sdo_resp(sock, rx, NULL);
    if (ret < 0) {
        fprintf(stderr, "  [SDO] Timeout/Abort 0x%03X 0x%04X:%02d (%s)\n", rx, idx, sub, desc);
    }
    return ret;
}

/* ─────────────────────────────────────────────
   CAN 接口管理
   ───────────────────────────────────────────── */
static int is_can_up(void)
{
    char cmd[256], line[256];
    snprintf(cmd, sizeof(cmd), "ip -br link show %s 2>/dev/null", CAN_INTERFACE);
    FILE *fp = popen(cmd, "r");
    if (!fp) return 0;
    int up = 0;
    if (fgets(line, sizeof(line), fp)) up = (strstr(line, "UP") != NULL);
    pclose(fp);
    return up;
}

static int setup_can(void)
{
    if (is_can_up()) {
        printf("[CAN] %s already UP.\n", CAN_INTERFACE);
        return 0;
    }
    char cmd[256];
    snprintf(cmd, sizeof(cmd),
             "ip link set %s type can bitrate %d", CAN_INTERFACE, CAN_BITRATE);
    if (system(cmd) != 0) { fprintf(stderr, "[CAN] set bitrate failed\n"); return -1; }
    snprintf(cmd, sizeof(cmd), "ip link set %s up", CAN_INTERFACE);
    if (system(cmd) != 0) { fprintf(stderr, "[CAN] bring up failed\n"); return -1; }
    printf("[CAN] %s configured @ %d bps, UP.\n", CAN_INTERFACE, CAN_BITRATE);
    return 0;
}

/* ─────────────────────────────────────────────
   NMT 管理报文
   COB-ID=0x000, Byte0=CS, Byte1=NodeID(0=广播)
   CS: 0x01=Start  0x02=Stop  0x80=Pre-op  0x81=Reset-node  0x82=Reset-comm
   ───────────────────────────────────────────── */
static void nmt_send(int sock, uint8_t cs, uint8_t node_id)
{
    uint8_t d[2] = { cs, node_id };
    send_frame(sock, 0x000, d, 2);
    usleep(50000); /* 给驱动器 50ms 处理时间 */
}

/* NMT Reset Communication（彻底复位从站通信参数） */
static void nmt_reset_comm(int sock)
{
    uint8_t d[2] = { 0x82, 0x00 }; /* CS=0x82 Reset Communication, 广播 */
    send_frame(sock, 0x000, d, 2);
    printf("[NMT] Reset Communication broadcast, waiting 2s...\n");
    sleep(2); /* 手册建议复位后等待较长时间 */
}

/* ─────────────────────────────────────────────
   PDO 映射配置（通过 SDO 在 Pre-operational 状态下配置）
   RPDO1 (0x1400/0x1600): 控制字(0x6040,00,16bit) + 目标速度(0x60FF,00,32bit)
   TPDO1 (0x1800/0x1A00): 状态字(0x6041,00,16bit) + 实际速度(0x606C,00,32bit)
   ───────────────────────────────────────────── */
static int configure_pdo(int sock, uint8_t node)
{
    uint8_t sdo[8];
    int ret = 0;

    printf("[PDO] Configuring PDO for NodeID=%d ...\n", node);

    /* ---- 配置 RPDO1 ---- */
    /* 1. 先禁用 RPDO1 COB-ID（bit31=1 表示无效） */
    sdo_write_u32(sdo, 0x1400, 0x01, 0x80000200 + node);
    ret |= sdo_write_ack(sock, node, sdo, "RPDO1 disable");

    /* 2. 清除 RPDO1 映射数量 = 0 */
    sdo_write_u8(sdo, 0x1600, 0x00, 0x00);
    ret |= sdo_write_ack(sock, node, sdo, "RPDO1 map count=0");

    /* 3. 映射对象1: 控制字 0x6040 sub00 16bit → 0x60400010 */
    sdo_write_u32(sdo, 0x1600, 0x01, 0x60400010);
    ret |= sdo_write_ack(sock, node, sdo, "RPDO1 map[1]=ctrlword");

    /* 4. 映射对象2: 目标速度 0x60FF sub00 32bit → 0x60FF0020 */
    sdo_write_u32(sdo, 0x1600, 0x02, 0x60FF0020);
    ret |= sdo_write_ack(sock, node, sdo, "RPDO1 map[2]=target_vel");

    /* 5. 设置映射数量 = 2 */
    sdo_write_u8(sdo, 0x1600, 0x00, 0x02);
    ret |= sdo_write_ack(sock, node, sdo, "RPDO1 map count=2");

    /* 6. 设置传输类型 = 254（异步，收到即生效） */
    sdo_write_u8(sdo, 0x1400, 0x02, 0xFE);
    ret |= sdo_write_ack(sock, node, sdo, "RPDO1 tx_type=254");

    /* 7. 使能 RPDO1 COB-ID */
    sdo_write_u32(sdo, 0x1400, 0x01, 0x00000200 + node);
    ret |= sdo_write_ack(sock, node, sdo, "RPDO1 enable");

    /* ---- 配置 TPDO1 ---- */
    /* 1. 禁用 TPDO1 */
    sdo_write_u32(sdo, 0x1800, 0x01, 0x80000180 + node);
    ret |= sdo_write_ack(sock, node, sdo, "TPDO1 disable");

    /* 2. 清除映射数量 */
    sdo_write_u8(sdo, 0x1A00, 0x00, 0x00);
    ret |= sdo_write_ack(sock, node, sdo, "TPDO1 map count=0");

    /* 3. 映射对象1: 状态字 0x6041 sub00 16bit → 0x60410010 */
    sdo_write_u32(sdo, 0x1A00, 0x01, 0x60410010);
    ret |= sdo_write_ack(sock, node, sdo, "TPDO1 map[1]=statusword");

    /* 4. 映射对象2: 实际速度 0x606C sub00 32bit → 0x606C0020 */
    sdo_write_u32(sdo, 0x1A00, 0x02, 0x606C0020);
    ret |= sdo_write_ack(sock, node, sdo, "TPDO1 map[2]=actual_vel");

    /* 5. 设置映射数量 = 2 */
    sdo_write_u8(sdo, 0x1A00, 0x00, 0x02);
    ret |= sdo_write_ack(sock, node, sdo, "TPDO1 map count=2");

    /* 6. 传输类型 = 254（异步） */
    sdo_write_u8(sdo, 0x1800, 0x02, 0xFE);
    ret |= sdo_write_ack(sock, node, sdo, "TPDO1 tx_type=254");

    /* 7. 禁止时间 = 0 (单位 0.1ms) */
    sdo_write_u16(sdo, 0x1800, 0x03, 0x0000);
    ret |= sdo_write_ack(sock, node, sdo, "TPDO1 inhibit=0");

    /* 8. 事件时间 = 10ms（10ms 主动上报一次） */
    sdo_write_u16(sdo, 0x1800, 0x05, 10);
    ret |= sdo_write_ack(sock, node, sdo, "TPDO1 event=10ms");

    /* 9. 使能 TPDO1 COB-ID */
    sdo_write_u32(sdo, 0x1800, 0x01, 0x00000180 + node);
    ret |= sdo_write_ack(sock, node, sdo, "TPDO1 enable");

    if (ret < 0)
        fprintf(stderr, "[PDO] Warning: some SDO steps failed for NodeID=%d\n", node);
    else
        printf("[PDO] NodeID=%d PDO configured OK.\n", node);

    return ret;
}

/* ─────────────────────────────────────────────
   电机使能（SDO 状态机：Shutdown→SwitchOn→EnableOp）
   ───────────────────────────────────────────── */
static int pdo_set_velocity(int sock, uint8_t node, int32_t rpm); /* 前向声明 */

static int enable_motor(int sock, uint8_t node)
{
    uint8_t sdo[8];
    uint32_t tx = 0x600 + node;
    uint32_t rx = 0x580 + node;

    printf("[MOTOR] Enabling NodeID=%d ...\n", node);

    /* Shutdown (0x0006) */
    sdo_write_u16(sdo, 0x6040, 0x00, 0x0006);
    send_frame(sock, tx, sdo, 8);
    wait_sdo_resp(sock, rx, NULL);
    usleep(50000);

    /* Switch On (0x0007) */
    sdo_write_u16(sdo, 0x6040, 0x00, 0x0007);
    send_frame(sock, tx, sdo, 8);
    wait_sdo_resp(sock, rx, NULL);
    usleep(50000);

    /* 设置速度模式 (Mode=3) */
    sdo_write_u8(sdo, 0x6060, 0x00, 0x03);
    send_frame(sock, tx, sdo, 8);
    wait_sdo_resp(sock, rx, NULL);
    usleep(50000);

    /* Enable Operation (0x000F) */
    sdo_write_u16(sdo, 0x6040, 0x00, 0x000F);
    send_frame(sock, tx, sdo, 8);
    wait_sdo_resp(sock, rx, NULL);
    usleep(50000);

    /* RPDO1 初始速度 = 0（确保电机静止） */
    pdo_set_velocity(sock, node, 0);

    printf("[MOTOR] NodeID=%d enabled.\n", node);
    return 0;
}

/* ─────────────────────────────────────────────
   停止电机（通过 RPDO1 发控制字 0x0006 = Shutdown，PDO 方式）
   ───────────────────────────────────────────── */
static int stop_motor(int sock, uint8_t node)
{
    uint8_t d[6];
    /* 控制字 = 0x0006 (Shutdown) + 目标速度 = 0 */
    d[0] = 0x06; d[1] = 0x00;
    d[2] = 0x00; d[3] = 0x00; d[4] = 0x00; d[5] = 0x00;
    return send_frame(sock, 0x200 + node, d, 6);
}

/* 通过 RPDO1 使能电机（控制字 0x000F），PDO 方式 */
static int enable_motor_pdo(int sock, uint8_t node)
{
    uint8_t d[6];
    /* 控制字 = 0x000F (Enable Operation) + 目标速度 = 0 */
    d[0] = 0x0F; d[1] = 0x00;
    d[2] = 0x00; d[3] = 0x00; d[4] = 0x00; d[5] = 0x00;
    return send_frame(sock, 0x200 + node, d, 6);
}

/* ─────────────────────────────────────────────
   PDO 速度命令发送（RPDO1：控制字 0x000F + 目标速度）
   数据布局（小端，共 6 字节）：
     Byte[0-1]: 控制字  0x000F (Enable Operation)
     Byte[2-5]: 目标速度 DEC（有符号 32bit）
   ───────────────────────────────────────────── */
static int pdo_set_velocity(int sock, uint8_t node, int32_t rpm)
{
    int32_t dec = RPM_TO_DEC(rpm);
    uint8_t d[6];
    uint16_t ctrl = 0x000F; /* Enable Operation */

    d[0] = (ctrl >>  0) & 0xFF;
    d[1] = (ctrl >>  8) & 0xFF;
    d[2] = (dec  >>  0) & 0xFF;
    d[3] = (dec  >>  8) & 0xFF;
    d[4] = (dec  >> 16) & 0xFF;
    d[5] = (dec  >> 24) & 0xFF;

    uint32_t cob_id = 0x200 + node; /* RPDO1 COB-ID */
    return send_frame(sock, cob_id, d, 6);
}

/* ─────────────────────────────────────────────
   尝试接收一帧 TPDO1（非阻塞，超时 timeout_us 微秒）
   返回 0=收到并解析, -1=超时/未收到
   ───────────────────────────────────────────── */
static int pdo_recv_status(int sock, uint8_t node,
                           uint16_t *statusword, int32_t *actual_vel,
                           int timeout_us)
{
    struct can_frame f;
    uint32_t tpdo1_id = 0x180 + node;

    fd_set fds; FD_ZERO(&fds); FD_SET(sock, &fds);
    struct timeval tv = { 0, timeout_us };
    if (select(sock + 1, &fds, NULL, NULL, &tv) <= 0) return -1;

    ssize_t n = read(sock, &f, sizeof(f));
    if (n <= 0 || f.can_id != tpdo1_id || f.can_dlc < 6) return -1;

    *statusword = (uint16_t)(f.data[0] | (f.data[1] << 8));
    *actual_vel = (int32_t)( f.data[2]        |
                            (f.data[3] <<  8)  |
                            (f.data[4] << 16)  |
                            (f.data[5] << 24) );
    return 0;
}

/* ─────────────────────────────────────────────
   检查电机是否需要重新初始化（通过 SDO 读状态字）
   ───────────────────────────────────────────── */
static int motor_needs_init(int sock, uint8_t node)
{
    uint8_t req[8], resp[8];
    sdo_read_req(req, 0x6041, 0x00);
    if (send_frame(sock, 0x600 + node, req, 8) < 0) return 1;
    if (wait_sdo_resp(sock, 0x580 + node, resp) < 0) return 1;

    uint16_t sw = (uint16_t)(resp[4] | (resp[5] << 8));
    /* bit3=fault, bit5-0: 检查是否处于 Operation enabled (0x37 mask = 0x27) */
    if ((sw & 0x006F) != 0x0027) return 1;

    /* 读工作模式 */
    sdo_read_req(req, 0x6061, 0x00);
    if (send_frame(sock, 0x600 + node, req, 8) < 0) return 1;
    if (wait_sdo_resp(sock, 0x580 + node, resp) < 0) return 1;
    return (resp[4] != 0x03); /* 3=速度模式 */
}

/* ─────────────────────────────────────────────
   参数解析辅助
   ───────────────────────────────────────────── */
static long parse_rpm_signed(const char *s)
{
    char *ep; errno = 0;
    long v = strtol(s, &ep, 10);
    if (errno || *ep) { fprintf(stderr, "Invalid number: '%s'\n", s); exit(1); }
    return v;
}

static long parse_rpm_positive(const char *s)
{
    long v = parse_rpm_signed(s);
    if (v <= 0) { fprintf(stderr, "Speed must be positive: '%s'\n", s); exit(1); }
    return v;
}

/* ─────────────────────────────────────────────
   上层运动控制（PDO 版）
   ───────────────────────────────────────────── */
static void motion_custom(int sock, long left_rpm, long right_rpm)
{
    pdo_set_velocity(sock, NODE_LEFT,  (int32_t)left_rpm);
    pdo_set_velocity(sock, NODE_RIGHT, (int32_t)right_rpm);
    printf("[MOVE] Custom: Left=%+ld rpm, Right=%+ld rpm\n", left_rpm, right_rpm);
}

static void motion_forward(int sock, long speed)
{
    motion_custom(sock, speed, -speed);
}

static void motion_backward(int sock, long speed)
{
    motion_custom(sock, -speed, speed);
}

static void motion_turn_left(int sock, long l, long r)
{
    if (l <= 0 || r <= 0 || r <= l) {
        fprintf(stderr, "turn left: both > 0 and right > left\n"); exit(1);
    }
    motion_custom(sock, l, r);
}

static void motion_turn_right(int sock, long l, long r)
{
    if (l <= 0 || r <= 0 || l <= r) {
        fprintf(stderr, "turn right: both > 0 and left > right\n"); exit(1);
    }
    motion_custom(sock, l, r);
}

static void motion_pivot_left(int sock, long speed)
{
    motion_custom(sock, 0, speed);
}

static void motion_pivot_right(int sock, long speed)
{
    motion_custom(sock, speed, 0);
}

/* ─────────────────────────────────────────────
   完整初始化流程（PDO 版）
   
   流程：
   1. NMT Reset Communication（复位所有通信参数到默认值）
   2. NMT Pre-operational（允许 SDO 配置）
   3. SDO 配置 PDO 映射（RPDO1 + TPDO1）
   4. SDO 设置工作模式=速度模式(3)
   5. SDO 控制字状态机切换（Shutdown→SwitchOn→Enable）
   6. PDO 发速度=0 确保静止
   7. NMT Start（进入 Operational，PDO 生效）
   ───────────────────────────────────────────── */
static int full_init(int sock)
{
    int pdo_ok_left = 0, pdo_ok_right = 0;
    int attempt;

    for (attempt = 1; attempt <= 3; attempt++) {
        printf("\n[INIT] === Attempt %d/3 ===\n", attempt);

        /* 1. NMT Reset Communication（复位从站通信参数到出厂默认） */
        printf("[NMT] Reset Communication (broadcast)...\n");
        nmt_reset_comm(sock);

        /* 2. NMT Pre-operational */
        printf("[NMT] Enter Pre-operational...\n");
        nmt_send(sock, 0x80, 0x00);
        usleep(200000); /* 等 200ms 让从站稳定在 Pre-op */

        /* 3. 配置 PDO 映射 */
        int r_left  = configure_pdo(sock, NODE_LEFT);
        int r_right = configure_pdo(sock, NODE_RIGHT);

        if (r_left == 0 && r_right == 0) {
            pdo_ok_left = pdo_ok_right = 1;
            printf("[INIT] PDO configuration successful for both motors.\n");
            break;
        }
        fprintf(stderr, "[INIT] PDO config failed (attempt %d), retrying...\n", attempt);
        usleep(500000);
    }

    if (!pdo_ok_left || !pdo_ok_right) {
        fprintf(stderr,
            "\n[FATAL] PDO configuration failed after 3 attempts!\n"
            "  - Check CAN cable connection\n"
            "  - Check motor NodeID is 1 and 2\n"
            "  - Check CAN bitrate is 500K\n"
            "  - Try power cycling the motors\n");
        return -1;
    }

    /* 4. SDO 设置工作模式 = 速度模式(3) */
    {
        uint8_t sdo[8];
        sdo_write_u8(sdo, 0x6060, 0x00, 0x03);
        sdo_write_ack(sock, NODE_LEFT, sdo, "mode=velocity");
        sdo_write_ack(sock, NODE_RIGHT, sdo, "mode=velocity");
        usleep(50000);
    }

    /* 5. SDO 控制字状态机使能（Shutdown → Switch On → Enable Operation） */
    enable_motor(sock, NODE_LEFT);
    enable_motor(sock, NODE_RIGHT);

    /* 6. NMT Start → 进入 Operational（PDO 开始生效） */
    printf("[NMT] Enter Operational (Start)...\n");
    nmt_send(sock, 0x01, 0x00);
    usleep(100000);

    /* 7. PDO 发送速度=0，确保电机静止 */
    pdo_set_velocity(sock, NODE_LEFT, 0);
    pdo_set_velocity(sock, NODE_RIGHT, 0);
    usleep(50000);

    printf("[INIT] Both motors ready. PDO control active.\n");
    return 0;
}

/* ─────────────────────────────────────────────
   MAIN
   ───────────────────────────────────────────── */
int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr,
            "Usage:\n"
            "  %s init                      # 强制重新初始化（PDO配置+使能）\n"
            "  %s move <L> <R>              # 自定义速度（rpm，可负）\n"
            "  %s forward  <speed>          # 前进\n"
            "  %s backward <speed>          # 后退\n"
            "  %s turn left  <L> <R>        # 左转 (R>L>0)\n"
            "  %s turn right <L> <R>        # 右转 (L>R>0)\n"
            "  %s pivot left  <speed>       # 左原地旋转\n"
            "  %s pivot right <speed>       # 右原地旋转\n"
            "  %s stop                      # 停车\n"
            "  %s status                    # 读取 TPDO1 状态（500ms 窗口）\n",
            argv[0], argv[0], argv[0], argv[0], argv[0],
            argv[0], argv[0], argv[0], argv[0], argv[0]);
        return 1;
    }

    /* CAN 接口 UP */
    if (setup_can() != 0) return 1;

    /* 打开 socket */
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) { perror("socket"); return 1; }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, CAN_INTERFACE, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); close(s); return 1; }

    struct sockaddr_can addr = {0};
    addr.can_family   = AF_CAN;
    addr.can_ifindex  = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind"); close(s); return 1;
    }

    /* ---- CAN ID 过滤器 ----
     * 只接收 Kinco 电机相关的帧，屏蔽其他设备（如策海推进器 0x300~0x30f）的报文
     * 在内核层丢弃，零 CPU 开销，避免干扰
     */
    {
        struct can_filter rfilter[4];
        rfilter[0].can_id   = 0x581;  /* SDO 响应 Node LEFT */
        rfilter[0].can_mask = 0x7FF;
        rfilter[1].can_id   = 0x582;  /* SDO 响应 Node RIGHT */
        rfilter[1].can_mask = 0x7FF;
        rfilter[2].can_id   = 0x181;  /* TPDO1 Node LEFT */
        rfilter[2].can_mask = 0x7FF;
        rfilter[3].can_id   = 0x182;  /* TPDO1 Node RIGHT */
        rfilter[3].can_mask = 0x7FF;
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
    }

    /* ---- stop ---- */
    if (strcmp(argv[1], "stop") == 0) {
        stop_motor(s, NODE_LEFT);
        stop_motor(s, NODE_RIGHT);
        printf("[STOP] Both motors stopped.\n");
        close(s); return 0;
    }

    /* ---- init（强制重新配置） ---- */
    if (strcmp(argv[1], "init") == 0) {
        full_init(s);
        close(s); return 0;
    }

    /* ---- status（读取 TPDO1） ---- */
    if (strcmp(argv[1], "status") == 0) {
        printf("[STATUS] Listening for TPDO1 (500ms)...\n");
        struct timespec t0, tn;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        int got_l = 0, got_r = 0;
        while (!got_l || !got_r) {
            clock_gettime(CLOCK_MONOTONIC, &tn);
            long ms = (tn.tv_sec - t0.tv_sec) * 1000
                    + (tn.tv_nsec - t0.tv_nsec) / 1000000;
            if (ms > 500) break;

            struct can_frame f;
            fd_set fds; FD_ZERO(&fds); FD_SET(s, &fds);
            struct timeval tv = {0, 5000};
            if (select(s + 1, &fds, NULL, NULL, &tv) <= 0) continue;
            ssize_t n = read(s, &f, sizeof(f));
            if (n <= 0) continue;

            uint8_t node = 0;
            if (f.can_id == 0x180 + NODE_LEFT)       node = NODE_LEFT;
            else if (f.can_id == 0x180 + NODE_RIGHT)  node = NODE_RIGHT;
            else continue;

            if (f.can_dlc < 6) continue;
            uint16_t sw  = (uint16_t)(f.data[0] | (f.data[1] << 8));
            int32_t  vel = (int32_t)( f.data[2]       |
                                     (f.data[3] <<  8) |
                                     (f.data[4] << 16) |
                                     (f.data[5] << 24) );
            /* 反换算：RPM = DEC * 1875 / (512 * 65536) */
            long rpm_actual = (long)vel * 1875L / (512L * 65536L);

            printf("  Node %d: StatusWord=0x%04X  ActualVel=%+ld rpm (DEC=%d)\n",
                   node, sw, rpm_actual, vel);

            if (node == NODE_LEFT)  got_l = 1;
            if (node == NODE_RIGHT) got_r = 1;
        }
        if (!got_l)  printf("  Node %d: No TPDO1 received (timeout)\n", NODE_LEFT);
        if (!got_r)  printf("  Node %d: No TPDO1 received (timeout)\n", NODE_RIGHT);
        close(s); return 0;
    }

    /* ---- check（SDO 检查电机状态，需要时自动 init） ---- */
    if (strcmp(argv[1], "check") == 0) {
        int need = 0;
        if (motor_needs_init(s, NODE_LEFT)) {
            printf("[CHECK] Node %d: NOT ready\n", NODE_LEFT); need = 1;
        } else {
            printf("[CHECK] Node %d: OK (Operation Enabled, Velocity Mode)\n", NODE_LEFT);
        }
        if (motor_needs_init(s, NODE_RIGHT)) {
            printf("[CHECK] Node %d: NOT ready\n", NODE_RIGHT); need = 1;
        } else {
            printf("[CHECK] Node %d: OK (Operation Enabled, Velocity Mode)\n", NODE_RIGHT);
        }
        if (need) {
            printf("[CHECK] Motors need init. Run './can_demo init' first.\n");
        } else {
            printf("[CHECK] All motors ready for PDO control.\n");
        }
        close(s); return 0;
    }

    /* ---- 运动命令解析 ---- */
    if (strcmp(argv[1], "move") == 0) {
        if (argc != 4) {
            fprintf(stderr, "Usage: %s move <left_rpm> <right_rpm>\n", argv[0]);
            close(s); return 1;
        }
        motion_custom(s, parse_rpm_signed(argv[2]), parse_rpm_signed(argv[3]));
    }
    else if (strcmp(argv[1], "forward") == 0) {
        if (argc != 3) { fprintf(stderr, "Missing speed\n"); close(s); return 1; }
        motion_forward(s, parse_rpm_positive(argv[2]));
    }
    else if (strcmp(argv[1], "backward") == 0) {
        if (argc != 3) { fprintf(stderr, "Missing speed\n"); close(s); return 1; }
        motion_backward(s, parse_rpm_positive(argv[2]));
    }
    else if (strcmp(argv[1], "turn") == 0) {
        if (argc != 5) { fprintf(stderr, "Usage: turn left|right <L> <R>\n"); close(s); return 1; }
        long L = parse_rpm_positive(argv[3]);
        long R = parse_rpm_positive(argv[4]);
        if (strcmp(argv[2], "left") == 0)       motion_turn_left(s, L, R);
        else if (strcmp(argv[2], "right") == 0) motion_turn_right(s, L, R);
        else { fprintf(stderr, "Direction must be 'left' or 'right'\n"); close(s); return 1; }
    }
    else if (strcmp(argv[1], "pivot") == 0) {
        if (argc != 4) { fprintf(stderr, "Usage: pivot left|right <speed>\n"); close(s); return 1; }
        long sp = parse_rpm_positive(argv[3]);
        if (strcmp(argv[2], "left") == 0)       motion_pivot_left(s, sp);
        else if (strcmp(argv[2], "right") == 0) motion_pivot_right(s, sp);
        else { fprintf(stderr, "Direction must be 'left' or 'right'\n"); close(s); return 1; }
    }
    else {
        fprintf(stderr, "Unknown command: %s\n", argv[1]);
        close(s); return 1;
    }

    close(s);
    return 0;
}
