/*
 * ============================================================================
 *  motor_controller.c — ROV 电机控制器 (C 重写 v9.0)
 * ============================================================================
 *
 *  架构 (双进程):
 *    [本程序 C]  CAN 控制 + B+伪逆推力分配 + 深度/Roll/Pitch/Yaw PID
 *               + 档位增益 + 安全逻辑 + pitch防翻覆 + 超时停机
 *    [motor_ros_bridge.py]  ROS2 订阅 /rov/cmd_vel, /rov/joy_state,
 *               /ins 各话题, /rov/depth → 写入 mmap 共享内存;
 *               读 mmap 状态 → 发布 /rov/motor_state
 *
 *  通信: mmap 共享内存 /dev/shm/rov_motor_shm (4096 字节)
 *    [  0 .. 215]  INPUT  (桥→C): 指令 + 传感器 (27 个 double, 每个 8B)
 *    [216 .. 351]  OUTPUT (C→桥): 电机RPM + PID状态 (17 个 double)
 *    每个 double 字段, 单写者模型 (桥写input / C写output), volatile 读
 *
 *  频率: 10Hz 主循环 (与原 Python 版一致)
 *
 *  档位 (v9.0 新设计):
 *    新1档 = 原3档全速; 2/3档在1档基础上递增到硬件上限
 *    满杆 RPM: 尾推 1235/1400/1550, 垂推下潜 1480/1520/1550,
 *              垂推上浮 1400/1470/1550, ID7转向 1280/1340/1400
 *    档位增益只在手动模式应用, 定深/定航向完全不受影响
 *
 *  编译 (交叉编译, 新VM 172.16.31.177):
 *    source /home/carl/RK3588/environment
 *    aarch64-linux-gnu-gcc -O2 -Wall -o motor_controller motor_controller.c \
 *        -lm -lpthread
 *  本地逻辑验证 (当前VM x86):
 *    gcc -O2 -Wall -o motor_controller_x86 motor_controller.c -lm -lpthread
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

/* ========================================================================
 *  常量定义 (与 motor_controller.py v8.6 对齐)
 * ======================================================================== */

/* ── CAN ──────────────────────────────────────────────────────────────── */
#define CAN_IFACE        "can0"
#define CAN_FRAME_200    0x200    /* 电机 0~3 */
#define CAN_FRAME_201    0x201    /* 电机 4~7 (实际 5,6,7) */
#define CAN_MAX_FAILS    3        /* 连续失败次数 → 重建 socket */

/* ── RPM 范围 ──────────────────────────────────────────────────────────── */
#define MAX_RPM          2000     /* 绝对上限 */
#define MIN_RPM          1100     /* 最小启动转速 (电机<1100不转) */
#define TAIL_RPM_MIN     1100
#define TAIL_RPM_MAX     1550
#define VERT_RPM_MIN     1100
#define VERT_RPM_MAX     1550
#define YAW_RPM_MIN      1100
#define YAW_RPM_MAX      1400     /* v7.1 定航向大误差回正 */

/* ── 超时 ──────────────────────────────────────────────────────────────── */
#define TIMEOUT_SEC          5.0    /* 命令超时 → 全停 */
#define ATT_TIMEOUT           2.0    /* 姿态数据超时 */
#define YAW_ATT_TIMEOUT       1.0    /* Yaw PID 专用姿态超时 */
#define DEPTH_TIMEOUT         3.0    /* 深度 PID 超时 */
#define DEPTH_TIMEOUT_SENSOR  3.0    /* 深度传感器超时 */

/* ── 推力增益 (定深 PID 阶段2 + 两阶段固定推力) ────────────────────────── */
#define FZ_GAIN_TAIL     0.178    /* 尾推垂直增益 (fz=1→1180RPM, 倾角22.5°) */
#define FZ_GAIN_VERT     0.667    /* 垂推增益 (fz=1→1400RPM, 纯垂直) */
#define TAIL_DIVE_MIN    0.223    /* 尾推下潜最低norm (→1200RPM) */
#define VERT_DIVE_MIN    0.556    /* 垂推下潜最低norm (→1350RPM) */

/* ── 手动模式垂直增益 (v8.6 方向不对称) ────────────────────────────────── */
/* 下潜满量程与定深阶段1一致 (尾推1250/垂推1480), 上浮沿用 FZ_GAIN (1180/1400) */
#define MANUAL_DIVE_FZ_TAIL  0.334   /* 手动下潜尾推 → 1250 RPM */
#define MANUAL_DIVE_FZ_VERT  0.845   /* 手动下潜垂推 → 1480 RPM */

/* ── 两阶段深度控制 ───────────────────────────────────────────────────── */
#define DEPTH_FIXED_THRESHOLD  0.10   /* 误差超此值 → 固定推力阶段 */
#define DIVE_TAIL_NORM  0.334    /* 下潜尾推 → 1250 RPM (增推克服浮力) */
#define DIVE_VERT_NORM  0.845    /* 下潜垂推 → 1480 RPM */
#define SURF_TAIL_NORM  0.178    /* 上浮尾推 → 1180 RPM (上浮更容易) */
#define SURF_VERT_NORM  0.667    /* 上浮垂推 → 1400 RPM */

/* ── 深度 PID ──────────────────────────────────────────────────────────── */
#define DEPTH_KP        2.5
#define DEPTH_KI        0.10
#define DEPTH_I_MAX     0.40
#define DEPTH_I_GATE    0.50
#define DEPTH_I_DECAY   0.85
#define DEPTH_DEADBAND  0.05     /* 5cm 死区 */

/* ── Roll PID ───────────────────────────────────────────────────────────── */
#define ROLL_KP         0.10
#define ROLL_KI         0.02
#define ROLL_I_MAX      0.20
#define ROLL_DBAND      1.0
#define ROLL_I_GATE     3.0
#define ROLL_I_DECAY    0.80

/* ── Pitch PID ─────────────────────────────────────────────────────────── */
#define PITCH_KP        0.10
#define PITCH_KI        0.02
#define PITCH_I_MAX     0.20
#define PITCH_DBAND     1.5
#define PITCH_I_GATE    5.0
#define PITCH_I_DECAY   0.85

/* ── Yaw PD (v8.4 可调PD, KI=0) ────────────────────────────────────────── */
#define YAW_KP          0.5
#define YAW_KD          0.3
#define YAW_KI          0.0
#define YAW_I_MAX       0.50
#define YAW_DEADBAND    0.15
#define YAW_I_GATE      2.0
#define YAW_I_DECAY     0.85
#define YAW_HOLD_THRESHOLD  10.0   /* 定航向大误差阈值(度) */
#define YAW_DIRECTION       (-1)   /* ID7物理方向修正 */
#define YAW_MANUAL_TRIM_MANUAL  0.60   /* 手动模式转向增益 */
#define YAW_MANUAL_TRIM_AUTO    0.10   /* 定深模式偏置微调 */
#define TAIL_YAW_RATIO_AUTO     0.5    /* 尾推Yaw比例 (定深/定航向) */
#define TAIL_YAW_RATIO_MANUAL   0.6    /* 尾推Yaw比例 (手动, 增强) */

/* ── Pitch 安全 (防翻覆) ──────────────────────────────────────────────── */
#define PITCH_SAFE      30.0     /* 开始线性降推 */
#define PITCH_KILL      55.0     /* 推力归零 */

/* ── 前馈补偿 (v8.0, 当前禁用) ─────────────────────────────────────────── */
#define FF_GAIN         0.0


/* ========================================================================
 *  新档位增益表 (v9.0: 新1档=原3档全速, 2/3档递增到硬件上限)
 * ========================================================================
 *  满杆 RPM 目标 (手动模式, 摇杆推到顶):
 *    通道        新1档   新2档   新3档(上限)
 *    尾推前进    1235    1400    1550
 *    垂推下潜    1480    1520    1550
 *    垂推上浮    1400    1470    1550
 *    ID7转向     1280    1340    1400
 *
 *  推导 (norm_to_rpm: RPM = min + clamp(|norm|)*(max-min)):
 *    尾推: B+ Fx列系数 ~0.30, fx=1.0 → u=0.30 → 1235 RPM
 *      gear2 目标u=0.667 → gain=2.22; gear3 目标u=1.0 → gain=3.33
 *    垂推下潜: MANUAL_DIVE_FZ_VERT=0.845 → 1480
 *      gear2 norm=0.933 → gain=1.104; gear3 norm=1.0 → gain=1.183
 *    垂推上浮: FZ_GAIN_VERT=0.667 → 1400
 *      gear2 norm=0.822 → gain=1.232; gear3 norm=1.0 → gain=1.50
 *    ID7: YAW_MANUAL_TRIM_MANUAL=0.60 → 1280
 *      gear2 mz=0.80 → gain=1.333; gear3 mz=1.0 → gain=1.667
 *
 *  注意: 增益放大后 norm 可能>1, norm_to_rpm 内部 clamp 到 max, 安全
 * ======================================================================== */
static const double GEAR_FX_GAIN[3]   = {1.0, 2.22,  3.33};   /* 前进 fx */
static const double GEAR_DIVE_GAIN[3] = {1.0, 1.104, 1.183};   /* 下潜 fz>0 */
static const double GEAR_SURF_GAIN[3] = {1.0, 1.232, 1.50};    /* 上浮 fz<0 */
static const double GEAR_YAW_GAIN[3]  = {1.0, 1.333, 1.667};   /* 转向 manual_yaw */

#define GEAR_INDEX(g)  (((g)<1?0:((g)>3?2:((g)-1))))  /* gear 1..3 → 0..2 */


/* ========================================================================
 *  B+ 伪逆推力分配矩阵 (7x6), 来自 thrust_allocator.py v1.5
 * ========================================================================
 *  行顺序: ID0, ID1, ID2, ID3, ID5, ID6, ID7
 *  列: Fx, Fy, Fz, Mx, My, Mz
 *  u[i] = sum_j BPLUS[i][j] * tau[j], 均匀饱和 (max|u|>1 → 等比缩放)
 * ======================================================================== */
static const double BPLUS[7][6] = {
    {+0.285976, +0.208295, +0.179844, +0.781942, +1.257653, -0.698634}, /* ID0 */
    {+0.299810, +0.185590, -0.179844, -0.781942, -1.257653, -0.718219}, /* ID1 */
    {+0.299810, -0.185590, -0.179844, +0.781942, -1.257653, +0.718219}, /* ID2 */
    {+0.285976, -0.208295, +0.179844, -0.781942, +1.257653, +0.698634}, /* ID3 */
    {+0.005294, -0.035425, +0.362353, -2.440002, -0.962566, -0.030557}, /* ID5 */
    {+0.005294, +0.035425, +0.362353, +2.440002, -0.962566, +0.030557}, /* ID6 */
    {+0.000000, +0.721481, +0.000000, +0.000000, +0.000000, +1.001866}, /* ID7 */
};


/* ========================================================================
 *  共享内存布局 (mmap /dev/shm/rov_motor_shm)
 * ========================================================================
 *  全部 double (8B), 无对齐问题. 单写者: 桥写 INPUT, C 写 OUTPUT.
 *  Python 桥用 struct.pack/unpack '<Nd' 按相同 offset 读写.
 * ======================================================================== */
#define SHM_PATH  "/dev/shm/rov_motor_shm"
#define SHM_SIZE  4096

/* INPUT 区 (桥→C), 27 doubles = 216 bytes */
#define IN_MOVE             (0*8)     /* double */
#define IN_UP               (1*8)
#define IN_YAW              (2*8)
#define IN_DIVE_FLAG        (3*8)
#define IN_TARGET_DEPTH     (4*8)
#define IN_YAW_HOLD_ACTIVE  (5*8)     /* 0/1 as double */
#define IN_YAW_HOLD_TARGET  (6*8)
#define IN_LAST_CMD_TIME    (7*8)
#define IN_E_STOPPED        (8*8)     /* 0/1 */
#define IN_GEAR             (9*8)     /* 1/2/3 */
#define IN_INS_YAW          (10*8)
#define IN_INS_PITCH        (11*8)
#define IN_INS_ROLL         (12*8)
#define IN_INS_ATT_VALID    (13*8)   /* 0/1 */
#define IN_INS_AX           (14*8)
#define IN_INS_AY           (15*8)
#define IN_INS_AZ          (16*8)
#define IN_INS_WX           (17*8)
#define IN_INS_WY           (18*8)
#define IN_INS_WZ           (19*8)
#define IN_INS_VE           (20*8)
#define IN_INS_VN           (21*8)
#define IN_INS_VD           (22*8)
#define IN_CURRENT_DEPTH    (23*8)
#define IN_DEPTH_VALID      (24*8)   /* 0/1 */
#define IN_LAST_ATT_TIME    (25*8)
#define IN_LAST_DEPTH_TIME  (26*8)
#define INPUT_END           (27*8)   /* = 216 */

/* OUTPUT 区 (C→桥), 17 doubles = 136 bytes, 起始 216 */
#define OUT_MOTOR0          (INPUT_END + 0*8)
#define OUT_MOTOR1          (INPUT_END + 1*8)
#define OUT_MOTOR2          (INPUT_END + 2*8)
#define OUT_MOTOR3          (INPUT_END + 3*8)
#define OUT_MOTOR4          (INPUT_END + 4*8)   /* unused, 恒0 */
#define OUT_MOTOR5          (INPUT_END + 5*8)
#define OUT_MOTOR6          (INPUT_END + 6*8)
#define OUT_MOTOR7          (INPUT_END + 7*8)
#define OUT_DEPTH_PID       (INPUT_END + 8*8)
#define OUT_ROLL_PID        (INPUT_END + 9*8)
#define OUT_PITCH_PID       (INPUT_END + 10*8)
#define OUT_YAW_PID         (INPUT_END + 11*8)
#define OUT_FZ_FF           (INPUT_END + 12*8)
#define OUT_INITIALIZED     (INPUT_END + 13*8)  /* 0/1 */
#define OUT_FIXED_STAGE     (INPUT_END + 14*8)  /* 0/1 */
#define OUT_TS              (INPUT_END + 15*8)
#define OUT_MODE            (INPUT_END + 16*8)  /* 0=MANUAL,1=FIXED,2=FIXED_UP,3=PID,4=YAW_HOLD */
#define OUTPUT_END          (INPUT_END + 17*8)  /* = 352 */


/* ========================================================================
 *  全局状态
 * ======================================================================== */
static volatile int g_running = 1;
static int g_can_sock = -1;
static int g_can_fail_count = 0;
static volatile int g_can_recovering = 0;
static pthread_mutex_t g_can_lock = PTHREAD_MUTEX_INITIALIZER;
static unsigned char *g_shm = NULL;   /* mmap 指针 */

/* 运行时状态 (对应 Python 版 MotorController 成员) */
typedef struct {
    /* 指令 */
    double last_move, last_up, last_yaw;
    double last_dive_flag;
    double target_depth;
    double last_cmd_time;
    int    yaw_hold_active;
    double yaw_hold_target;
    int    yaw_captured;
    int    yaw_first_msg;
    int    e_stopped;
    int    gear;

    /* 传感器 */
    double ins_yaw, ins_pitch, ins_roll;
    int    ins_att_valid;
    double ins_ax, ins_ay, ins_az;
    double ins_wx, ins_wy, ins_wz;
    double ins_ve, ins_vn, ins_vd;
    double current_depth;
    int    depth_valid;
    double last_att_time, last_depth_time;
    double filtered_depth;

    /* PID 状态 */
    double depth_err_i, depth_pid_out, fz_ff;
    double roll_err_i, roll_pid_out;
    double pitch_err_i, pitch_pid_out;
    double yaw_err_i, yaw_pid_out;
    double yaw_target;
    double last_yaw_err;
    int    last_yaw_err_valid;
    double last_yaw_pid_time;

    /* 输出 */
    int    motors[8];
    int    fixed_stage;
    int    initialized;
    int    hb_log_count;
} MotorState;

static MotorState S;


/* ========================================================================
 *  工具函数
 * ======================================================================== */

static double wall_now_sec(void) {
    /* 用于与桥的时间戳对齐 (桥用 time.time() = wall clock) */
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static double clampd(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static double angle_diff(double target, double current) {
    /* 归一化到 [-180, 180] */
    double d = target - current;
    while (d > 180.0)  d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

/* norm → rpm, 与 Python norm_to_rpm 一致
 *   |norm|<0.001 → 0
 *   否则 sign * clamp(min + |norm|*(max-min), min, max) */
static int norm_to_rpm(double norm, int min_rpm, int max_rpm) {
    if (fabs(norm) < 0.001) return 0;
    double rpm = min_rpm + fabs(norm) * (double)(max_rpm - min_rpm);
    if (rpm > (double)max_rpm) rpm = (double)max_rpm;
    int sign = (norm < 0) ? -1 : 1;
    return sign * (int)rpm;
}

/* rpm → CAN 命令字段 (11位绝对值 + bit11 反转标志)
 *   bit[0:10] = |rpm|, bit[11] = 1 反转
 *   与 Python rpm_to_cmd 一致 */
static unsigned short rpm_to_cmd(int rpm) {
    unsigned short a = (unsigned short)(abs(rpm) & 0x07FF);
    if (rpm < 0) a |= 0x0800;
    return a;
}


/* ========================================================================
 *  CAN 发送 (与 Python 版 build_ctrl_200/201 一致)
 * ========================================================================
 *  帧 0x200: 电机 0,1,2,3 (ID1/ID3 反相, 即 rpm 取反后编码)
 *  帧 0x201: 电机 4,5,6,7 (ID6 反相)
 *    4 字段 x 2 字节 = 8 字节, 小端
 * ======================================================================== */
static void build_ctrl_200(const int g[8], unsigned char *out) {
    /* ID0 直接, ID1 取反, ID2 直接, ID3 取反 */
    unsigned short c0 = rpm_to_cmd(g[0]);
    unsigned short c1 = rpm_to_cmd(-g[1]);
    unsigned short c2 = rpm_to_cmd(g[2]);
    unsigned short c3 = rpm_to_cmd(-g[3]);
    /* 小端打包 (4 个 uint16) */
    out[0] = c0 & 0xFF;        out[1] = (c0 >> 8) & 0xFF;
    out[2] = c1 & 0xFF;        out[3] = (c1 >> 8) & 0xFF;
    out[4] = c2 & 0xFF;        out[5] = (c2 >> 8) & 0xFF;
    out[6] = c3 & 0xFF;        out[7] = (c3 >> 8) & 0xFF;
}

static void build_ctrl_201(const int g[8], unsigned char *out) {
    /* ID4=0(无), ID5 直接, ID6 取反, ID7 直接 */
    unsigned short c4 = 0;
    unsigned short c5 = rpm_to_cmd(g[5]);
    unsigned short c6 = rpm_to_cmd(-g[6]);
    unsigned short c7 = rpm_to_cmd(g[7]);
    out[0] = c4 & 0xFF;        out[1] = (c4 >> 8) & 0xFF;
    out[2] = c5 & 0xFF;        out[3] = (c5 >> 8) & 0xFF;
    out[4] = c6 & 0xFF;        out[5] = (c6 >> 8) & 0xFF;
    out[6] = c7 & 0xFF;        out[7] = (c7 >> 8) & 0xFF;
}

static int send_can_frame(int can_id, const unsigned char *data) {
    pthread_mutex_lock(&g_can_lock);
    if (g_can_sock < 0) {
        g_can_fail_count++;
        pthread_mutex_unlock(&g_can_lock);
        return -1;
    }
    struct can_frame frame;
    frame.can_id  = can_id & 0x1FFFFFFF;
    frame.can_dlc = 8;
    memcpy(frame.data, data, 8);
    int n = write(g_can_sock, &frame, sizeof(frame));
    if (n < 0) {
        g_can_fail_count++;
        pthread_mutex_unlock(&g_can_lock);
        return -1;
    }
    g_can_fail_count = 0;
    pthread_mutex_unlock(&g_can_lock);
    return 0;
}

static int can_init(void) {
    pthread_mutex_lock(&g_can_lock);
    g_can_fail_count = 0;
    g_can_recovering = 0;
    if (g_can_sock >= 0) { close(g_can_sock); g_can_sock = -1; }

    g_can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (g_can_sock < 0) {
        fprintf(stderr, "[FATAL] CAN socket 失败: %s\n", strerror(errno));
        pthread_mutex_unlock(&g_can_lock);
        return 0;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, CAN_IFACE, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(g_can_sock, SIOCGIFINDEX, &ifr) < 0) {
        fprintf(stderr, "[FATAL] CAN 接口 %s 不存在: %s\n", CAN_IFACE, strerror(errno));
        close(g_can_sock); g_can_sock = -1;
        pthread_mutex_unlock(&g_can_lock);
        return 0;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(g_can_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "[FATAL] CAN bind 失败: %s\n", strerror(errno));
        close(g_can_sock); g_can_sock = -1;
        pthread_mutex_unlock(&g_can_lock);
        return 0;
    }
    pthread_mutex_unlock(&g_can_lock);

    /* 发送使能零帧 */
    unsigned char zero[8] = {0,0,0,0,0,0,0,0};
    send_can_frame(CAN_FRAME_200, zero);
    usleep(50000);
    send_can_frame(CAN_FRAME_201, zero);
    usleep(50000);
    send_can_frame(CAN_FRAME_200, zero);
    usleep(50000);
    send_can_frame(CAN_FRAME_201, zero);
    return 1;
}

static void can_recover(void) {
    pthread_mutex_lock(&g_can_lock);
    if (g_can_recovering) { pthread_mutex_unlock(&g_can_lock); return; }
    g_can_recovering = 1;
    pthread_mutex_unlock(&g_can_lock);

    fprintf(stderr, "[CAN] 尝试恢复 CAN 通信...\n");
    if (g_can_sock >= 0) { close(g_can_sock); g_can_sock = -1; }

    /* 重启 CAN 接口 (bus-off 恢复) */
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "ip link set %s restart", CAN_IFACE);
    int rc = system(cmd);
    (void)rc;
    usleep(200000);

    can_init();

    pthread_mutex_lock(&g_can_lock);
    g_can_recovering = 0;
    pthread_mutex_unlock(&g_can_lock);
}

static void send_motor_rpm(const int g[8]) {
    unsigned char buf200[8], buf201[8];
    build_ctrl_200(g, buf200);
    build_ctrl_201(g, buf201);
    if (send_can_frame(CAN_FRAME_200, buf200) < 0 ||
        send_can_frame(CAN_FRAME_201, buf201) < 0) {
        if (g_can_fail_count >= CAN_MAX_FAILS && !g_can_recovering) {
            /* 新线程恢复, 不阻塞心跳 */
            pthread_t t;
            if (pthread_create(&t, NULL, (void*(*)(void*))can_recover, NULL) == 0) {
                pthread_detach(t);
            }
        }
    }
}

static void can_close_all(void) {
    pthread_mutex_lock(&g_can_lock);
    if (g_can_sock >= 0) {
        unsigned char zero[8] = {0,0,0,0,0,0,0,0};
        struct can_frame f;
        f.can_id = CAN_FRAME_200 & 0x1FFFFFFF; f.can_dlc = 8; memcpy(f.data, zero, 8);
        ssize_t _w1 = write(g_can_sock, &f, sizeof(f)); (void)_w1;
        f.can_id = CAN_FRAME_201 & 0x1FFFFFFF; f.can_dlc = 8; memcpy(f.data, zero, 8);
        ssize_t _w2 = write(g_can_sock, &f, sizeof(f)); (void)_w2;
        close(g_can_sock);
        g_can_sock = -1;
    }
    pthread_mutex_unlock(&g_can_lock);
}


/* ========================================================================
 *  B+ 伪逆推力分配 (与 thrust_allocator.allocate 一致)
 * ========================================================================
 *  返回 7 个电机归一化命令 u[7], 索引对应 [0,1,2,3,5,6,7]→[0..6]
 *  均匀饱和: max|u|>1 → 全部等比缩小
 * ======================================================================== */
static void thrust_allocate(double fx, double fy, double fz,
                            double mx, double my, double mz,
                            double u[7]) {
    double tau[6] = {fx, fy, fz, mx, my, mz};
    double max_abs = 0.0;
    int i, j;
    for (i = 0; i < 7; i++) {
        double s = 0.0;
        for (j = 0; j < 6; j++) s += BPLUS[i][j] * tau[j];
        u[i] = s;
        if (fabs(s) > max_abs) max_abs = fabs(s);
    }
    /* 均匀饱和 */
    if (max_abs > 1.0) {
        double scale = 1.0 / max_abs;
        for (i = 0; i < 7; i++) u[i] *= scale;
    }
}


/* ========================================================================
 *  共享内存读写 (mmap)
 * ======================================================================== */
static int shm_init(void) {
    int fd = open(SHM_PATH, O_RDWR | O_CREAT, 0666);
    if (fd < 0) {
        fprintf(stderr, "[FATAL] 无法创建共享内存 %s: %s\n", SHM_PATH, strerror(errno));
        return 0;
    }
    if (ftruncate(fd, SHM_SIZE) < 0) {
        fprintf(stderr, "[WARN] ftruncate 失败: %s\n", strerror(errno));
    }
    g_shm = mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (g_shm == MAP_FAILED) {
        fprintf(stderr, "[FATAL] mmap 失败: %s\n", strerror(errno));
        g_shm = NULL;
        return 0;
    }
    memset(g_shm, 0, SHM_SIZE);
    return 1;
}

/* 读取一个 double (从指定 offset) */
static double shm_read_d(int off) {
    if (!g_shm) return 0.0;
    double v;
    memcpy(&v, g_shm + off, sizeof(double));
    return v;
}

/* 写入一个 double (到指定 offset) */
static void shm_write_d(int off, double v) {
    if (!g_shm) return;
    memcpy(g_shm + off, &v, sizeof(double));
}

/* 从共享内存同步 INPUT → MotorState */
static void sync_from_shm(void) {
    S.last_move        = shm_read_d(IN_MOVE);
    S.last_up          = shm_read_d(IN_UP);
    S.last_yaw         = shm_read_d(IN_YAW);
    S.last_dive_flag   = shm_read_d(IN_DIVE_FLAG);
    S.target_depth     = shm_read_d(IN_TARGET_DEPTH);
    S.yaw_hold_active  = (shm_read_d(IN_YAW_HOLD_ACTIVE) > 0.5) ? 1 : 0;
    S.yaw_hold_target  = shm_read_d(IN_YAW_HOLD_TARGET);
    S.last_cmd_time    = shm_read_d(IN_LAST_CMD_TIME);
    S.e_stopped         = (shm_read_d(IN_E_STOPPED) > 0.5) ? 1 : 0;
    S.gear              = (int)shm_read_d(IN_GEAR);
    if (S.gear < 1) S.gear = 1;
    if (S.gear > 4) S.gear = 4;

    /* 传感器 (桥已处理时间戳更新) */
    S.ins_yaw       = shm_read_d(IN_INS_YAW);
    S.ins_pitch     = shm_read_d(IN_INS_PITCH);
    S.ins_roll      = shm_read_d(IN_INS_ROLL);
    S.ins_att_valid = (shm_read_d(IN_INS_ATT_VALID) > 0.5) ? 1 : 0;
    S.ins_ax = shm_read_d(IN_INS_AX); S.ins_ay = shm_read_d(IN_INS_AY); S.ins_az = shm_read_d(IN_INS_AZ);
    S.ins_wx = shm_read_d(IN_INS_WX); S.ins_wy = shm_read_d(IN_INS_WY); S.ins_wz = shm_read_d(IN_INS_WZ);
    S.ins_ve = shm_read_d(IN_INS_VE); S.ins_vn = shm_read_d(IN_INS_VN); S.ins_vd = shm_read_d(IN_INS_VD);

    double raw_depth = shm_read_d(IN_CURRENT_DEPTH);
    int depth_valid = (shm_read_d(IN_DEPTH_VALID) > 0.5) ? 1 : 0;
    if (depth_valid) {
        if (S.depth_valid) {
            S.filtered_depth = 0.5 * raw_depth + 0.5 * S.filtered_depth;
        } else {
            S.filtered_depth = raw_depth;
        }
        S.current_depth = S.filtered_depth;
        S.depth_valid = 1;
        S.last_depth_time = shm_read_d(IN_LAST_DEPTH_TIME);
    }
    /* 姿态时间戳 */
    S.last_att_time = shm_read_d(IN_LAST_ATT_TIME);
}

/* 把 OUTPUT 写回共享内存 (供桥发布 /rov/motor_state) */
static void sync_to_shm(void) {
    shm_write_d(OUT_MOTOR0, (double)S.motors[0]);
    shm_write_d(OUT_MOTOR1, (double)S.motors[1]);
    shm_write_d(OUT_MOTOR2, (double)S.motors[2]);
    shm_write_d(OUT_MOTOR3, (double)S.motors[3]);
    shm_write_d(OUT_MOTOR4, 0.0);
    shm_write_d(OUT_MOTOR5, (double)S.motors[5]);
    shm_write_d(OUT_MOTOR6, (double)S.motors[6]);
    shm_write_d(OUT_MOTOR7, (double)S.motors[7]);
    shm_write_d(OUT_DEPTH_PID,   S.depth_pid_out);
    shm_write_d(OUT_ROLL_PID,    S.roll_pid_out);
    shm_write_d(OUT_PITCH_PID,   S.pitch_pid_out);
    shm_write_d(OUT_YAW_PID,     S.yaw_pid_out);
    shm_write_d(OUT_FZ_FF,       S.fz_ff);
    shm_write_d(OUT_INITIALIZED, S.initialized ? 1.0 : 0.0);
    shm_write_d(OUT_FIXED_STAGE, S.fixed_stage ? 1.0 : 0.0);
    shm_write_d(OUT_TS,          wall_now_sec());
    /* 模式标记 */
    double mode = 0.0; /* MANUAL */
    if (S.yaw_hold_active) mode = 4.0;
    else if (S.last_dive_flag > 0.1 && S.depth_valid) {
        double err = S.target_depth - S.current_depth;
        if (fabs(err) > DEPTH_FIXED_THRESHOLD) mode = (err > 0) ? 1.0 : 2.0;
        else mode = 3.0;
    }
    shm_write_d(OUT_MODE, mode);
}


/* ========================================================================
 *  PID 计算 (与 Python 版 _compute_*_pid 一致)
 * ======================================================================== */

static void compute_depth_pid(void) {
    /* 输出 fz in [-1,+1], + = 下潜 */
    if (!S.depth_valid) { S.depth_pid_out = 0.0; return; }
    if ((wall_now_sec() - S.last_depth_time) > DEPTH_TIMEOUT_SENSOR) {
        S.depth_pid_out = 0.0; return;
    }

    double err = S.target_depth - S.current_depth;
    double dt = 0.1; /* 10Hz */

    /* P (带死区) */
    double p_error = (fabs(err) < DEPTH_DEADBAND) ? 0.0 : err;
    double p = DEPTH_KP * p_error;

    /* I */
    if (fabs(err) > DEPTH_I_GATE) {
        /* 不积分 */
    } else if ((err * S.depth_err_i) < -0.01) {
        S.depth_err_i = 0.0;
    } else if (fabs(err) < DEPTH_DEADBAND) {
        S.depth_err_i *= DEPTH_I_DECAY;
    } else if (fabs(S.depth_pid_out) < 0.95 || (err * S.depth_err_i) < 0) {
        S.depth_err_i = clampd(S.depth_err_i + DEPTH_KI * err * dt,
                               -DEPTH_I_MAX, DEPTH_I_MAX);
    }
    S.depth_pid_out = clampd(p + S.depth_err_i, -1.0, 1.0);
}

static void compute_depth_ff(void) {
    /* 前馈补偿 (FF_GAIN=0 → 无效果) */
    if (FF_GAIN <= 0.0) { S.fz_ff = 0.0; return; }
    double pitch_rad = (S.ins_att_valid) ? (S.ins_pitch * M_PI / 180.0) : 0.0;
    double roll_rad  = (S.ins_att_valid) ? (S.ins_roll  * M_PI / 180.0) : 0.0;
    /* FF_BIAS/系数均为0, 结果=0; 保留结构供后续训练 */
    double raw = 0.0
               + 0.0 * S.target_depth
               + 0.0 * sin(pitch_rad)
               + 0.0 * sin(roll_rad);
    S.fz_ff = clampd(raw * FF_GAIN, -0.5, 0.5);
}

static void compute_roll_pid(void) {
    double now = wall_now_sec();
    int att_valid = (S.ins_att_valid && (now - S.last_att_time) < ATT_TIMEOUT);
    if (!att_valid) { S.roll_pid_out = 0.0; return; }

    double dt = 0.1;
    double roll_err = 0.0 - S.ins_roll;
    double r_p = (fabs(roll_err) < ROLL_DBAND) ? 0.0 : (ROLL_KP * roll_err);

    if (fabs(roll_err) > ROLL_I_GATE) {
        /* 不积分 */
    } else if ((roll_err * S.roll_err_i) < -0.01) {
        S.roll_err_i = 0.0;
    } else if (fabs(roll_err) < ROLL_DBAND) {
        S.roll_err_i *= ROLL_I_DECAY;
    } else {
        S.roll_err_i = clampd(S.roll_err_i + ROLL_KI * roll_err * dt,
                               -ROLL_I_MAX, ROLL_I_MAX);
    }
    S.roll_pid_out = clampd(r_p + S.roll_err_i, -1.0, 1.0);
}

static void compute_pitch_pid(void) {
    double now = wall_now_sec();
    int att_valid = (S.ins_att_valid && (now - S.last_att_time) < ATT_TIMEOUT);
    if (!att_valid) { S.pitch_pid_out = 0.0; return; }

    double dt = 0.1;
    double pitch_err = 0.0 - S.ins_pitch;
    double p_p = (fabs(pitch_err) < PITCH_DBAND) ? 0.0 : (PITCH_KP * pitch_err);

    if (fabs(pitch_err) > PITCH_I_GATE) {
        /* 不积分 */
    } else if ((pitch_err * S.pitch_err_i) < -0.01) {
        S.pitch_err_i = 0.0;
    } else if (fabs(pitch_err) < PITCH_DBAND) {
        S.pitch_err_i *= PITCH_I_DECAY;
    } else {
        S.pitch_err_i = clampd(S.pitch_err_i + PITCH_KI * pitch_err * dt,
                               -PITCH_I_MAX, PITCH_I_MAX);
    }
    S.pitch_pid_out = clampd(p_p + S.pitch_err_i, -1.0, 1.0);
}

static void compute_yaw_pid(void) {
    /* Yaw PD: mz = KP*err + KD*(err-last_err)/dt (+ I if KI>0) */
    double now = wall_now_sec();
    int valid = (S.ins_att_valid && (now - S.last_att_time) < YAW_ATT_TIMEOUT);
    if (!valid) { S.yaw_pid_out = 0.0; return; }

    double yaw_err = angle_diff(S.yaw_target, S.ins_yaw);
    double dt = now - S.last_yaw_pid_time;
    if (dt < 0.01) dt = 0.01;
    S.last_yaw_pid_time = now;

    /* P */
    double p_out = (fabs(yaw_err) < YAW_DEADBAND) ? 0.0 : (YAW_KP * yaw_err);

    /* D */
    double d_out = 0.0;
    if (S.last_yaw_err_valid) {
        double err_dot = (yaw_err - S.last_yaw_err) / dt;
        d_out = YAW_KD * err_dot;
    }
    S.last_yaw_err = yaw_err;
    S.last_yaw_err_valid = 1;

    /* I (KI=0 时禁用) */
    double i_out = 0.0;
    if (YAW_KI > 0.0) {
        if (fabs(yaw_err) > YAW_I_GATE) {
            /* 不积分 */
        } else if ((yaw_err * S.yaw_err_i) < -0.01) {
            S.yaw_err_i = 0.0;
        } else if (fabs(yaw_err) < YAW_DEADBAND) {
            S.yaw_err_i *= YAW_I_DECAY;
        } else {
            S.yaw_err_i = clampd(S.yaw_err_i + YAW_KI * yaw_err * dt,
                                 -YAW_I_MAX, YAW_I_MAX);
        }
        i_out = S.yaw_err_i;
    }
    S.yaw_pid_out = clampd(p_out + d_out + i_out, -1.0, 1.0);
}


/* ========================================================================
 *  10Hz 心跳 (核心逻辑, 与 Python heartbeat_tick 一致)
 * ======================================================================== */
static void heartbeat_tick(void) {
    S.hb_log_count++;

    /* 从共享内存读最新指令 + 传感器 */
    sync_from_shm();

    /* 首帧就绪标志 */
    if (!S.initialized && S.ins_att_valid && S.depth_valid) {
        S.initialized = 1;
        fprintf(stderr, "[MC] 传感器就绪: yaw=%.2f pitch=%.2f roll=%.2f depth=%.3f\n",
                S.ins_yaw, S.ins_pitch, S.ins_roll, S.current_depth);
    }

    /* 急停 */
    if (S.e_stopped) {
        int g[8] = {0,0,0,0,0,0,0,0};
        memcpy(S.motors, g, sizeof(g));
        send_motor_rpm(g);
        sync_to_shm();
        return;
    }

    if (!S.initialized) {
        /* 传感器未就绪, 不输出 */
        sync_to_shm();
        return;
    }

    /* ── 定深模式安全检查 ── */
    int att_ok;
    if (S.last_dive_flag > 0.1) {
        if (!S.depth_valid) {
            int all_zero = 1;
            for (int i = 0; i < 8; i++) if (S.motors[i]) { all_zero = 0; break; }
            if (!all_zero) {
                fprintf(stderr, "[MC] 定深: 深度传感器无效, 停止电机\n");
            }
            memset(S.motors, 0, sizeof(S.motors));
            send_motor_rpm(S.motors);
            sync_to_shm();
            return;
        }
        if (S.target_depth <= 0.05) {
            int all_zero = 1;
            for (int i = 0; i < 8; i++) if (S.motors[i]) { all_zero = 0; break; }
            if (!all_zero) {
                fprintf(stderr, "[MC] 定深: 目标深度无效(%.2f), 停止电机\n", S.target_depth);
            }
            memset(S.motors, 0, sizeof(S.motors));
            send_motor_rpm(S.motors);
            sync_to_shm();
            return;
        }
        att_ok = (S.ins_att_valid && (wall_now_sec() - S.last_att_time) < ATT_TIMEOUT);
    } else {
        att_ok = (S.ins_att_valid && (wall_now_sec() - S.last_att_time) < ATT_TIMEOUT);
    }

    /* ── 定深模式运行 PID; 手动模式不运行 (定航向活跃时 yaw 单独算) ── */
    if (S.last_dive_flag > 0.1) {
        compute_depth_pid();
        compute_depth_ff();
        compute_roll_pid();
        compute_pitch_pid();
        if (!S.yaw_hold_active) compute_yaw_pid();
    }

    /* ════════════════════════════════════════════════════════════════
     *  构建 6-DOF 力/力矩向量 tau
     * ════════════════════════════════════════════════════════════════ */
    int gi = GEAR_INDEX(S.gear);  /* 档位增益索引 0..2 */

    /* Fx: 手动前进/后退; 翻转使 axis+1→电机后退(与监控一致) */
    double fx = -S.last_move;
    /* v9.0: 手动模式按档位增益放大前进输入 */
    if (S.last_dive_flag <= 0.1) {
        fx *= GEAR_FX_GAIN[gi];
    }

    /* Fy: 侧移 (当前无外部需求) */
    double fy = 0.0;

    /* Fz: 定深=PID输出, 手动=手柄输入 (+ = 下潜) */
    double fz;
    if (S.last_dive_flag > 0.1) {
        fz = S.depth_pid_out + S.fz_ff;
        if (!att_ok) fz = clampd(fz, -0.3, 0.3);
    } else {
        fz = S.last_up;
    }

    /* 当前深度控制阶段 (用于 mz/ID7 判断) */
    S.fixed_stage = 0;
    if (S.last_dive_flag > 0.1 && S.depth_valid) {
        S.fixed_stage = (fabs(S.target_depth - S.current_depth) > DEPTH_FIXED_THRESHOLD);
    }

    /* Mx/My: Roll/Pitch PID (当前临时禁用, mx=my=0) */
    double mx = 0.0; /* S.roll_pid_out */
    double my = 0.0; /* S.pitch_pid_out */

    /* Mz: 定航向优先 → 定深PID阶段 → 手动偏航
     * v7.7: YAW_DIRECTION 只修正 PID 输出, 不翻手动 steering
     * v8.6: 手动/自动增益分离 (手动0.60, 定深0.10) */
    double manual_yaw;
    if (S.last_dive_flag > 0.1) {
        manual_yaw = S.last_yaw * YAW_MANUAL_TRIM_AUTO;
    } else {
        manual_yaw = S.last_yaw * YAW_MANUAL_TRIM_MANUAL;
        /* v9.0: 手动模式按档位增益放大转向 */
        manual_yaw *= GEAR_YAW_GAIN[gi];
    }

    double mz_id7;
    if (S.yaw_hold_active) {
        S.yaw_target = S.yaw_hold_target;
        double yaw_err_hold = 0.0;
        if (S.ins_att_valid) yaw_err_hold = angle_diff(S.yaw_target, S.ins_yaw);
        if (S.ins_att_valid && fabs(yaw_err_hold) > YAW_HOLD_THRESHOLD) {
            /* 阶段1: 大转速回正 (mz_id7=±1.0 → ID7=1400RPM) */
            S.yaw_pid_out = (yaw_err_hold > 0) ? 1.0 : -1.0;
            S.yaw_err_i = 0.0;
            mz_id7 = S.yaw_pid_out;
        } else {
            compute_yaw_pid();
            mz_id7 = clampd(S.yaw_pid_out, -1.0, 1.0);
        }
        /* 定航向 PID 方向修正 */
        mz_id7 *= YAW_DIRECTION;
    } else if (S.last_dive_flag > 0.1) {
        if (S.fixed_stage) {
            mz_id7 = 0.0; /* 固定阶段完全不控制 ID7 */
        } else {
            compute_yaw_pid();
            mz_id7 = clampd(S.yaw_pid_out * YAW_DIRECTION + manual_yaw, -1.0, 1.0);
        }
    } else {
        mz_id7 = manual_yaw;
        /* 手动模式不应用 YAW_DIRECTION, 保持手柄原始方向 */
    }

    /* v7.3: 尾推 Yaw 辅助 (B+ 自动分配差速, 按比例缩放)
     * v8.6: 手动模式用增强比例0.6, 定深/定航向保持0.5 */
    double mz_tail;
    if (S.yaw_hold_active || S.last_dive_flag > 0.1) {
        mz_tail = mz_id7 * TAIL_YAW_RATIO_AUTO;
    } else {
        mz_tail = mz_id7 * TAIL_YAW_RATIO_MANUAL;
    }

    /* ── 推力分配 (Fz 不通过分配器, 直接控制 ID5/6) ── */
    double u_all[7], u_fxonly[7];
    thrust_allocate(fx, fy, 0.0, mx, my, mz_tail, u_all);

    /* ════════════════════════════════════════════════════════════════
     *  两阶段深度控制 + 手动模式方向不对称垂直增益
     * ════════════════════════════════════════════════════════════════ */
    double fz_tail, fz_vert;
    if (S.last_dive_flag > 0.1 && S.depth_valid) {
        double depth_error = S.target_depth - S.current_depth; /* + = 需下潜 */
        if (fabs(depth_error) > DEPTH_FIXED_THRESHOLD) {
            /* 阶段1: 固定推力 */
            if (depth_error > 0) {
                fz_tail =  DIVE_TAIL_NORM;   /* +0.334 → 1250 RPM */
                fz_vert =  DIVE_VERT_NORM;   /* +0.845 → 1480 RPM */
            } else {
                fz_tail = -SURF_TAIL_NORM;   /* -0.178 → -1180 RPM */
                fz_vert = -SURF_VERT_NORM;   /* -0.667 → -1400 RPM */
            }
        } else {
            /* 阶段2: PID 精细控制 */
            fz = S.depth_pid_out + S.fz_ff;
            fz_tail = fz * FZ_GAIN_TAIL;
            fz_vert = fz * FZ_GAIN_VERT;
            if (fz > 0.01) {
                if (fz_tail < TAIL_DIVE_MIN) fz_tail = TAIL_DIVE_MIN;
                if (fz_vert < VERT_DIVE_MIN) fz_vert = VERT_DIVE_MIN;
            }
        }
    } else {
        /* 手动模式或无深度数据: 方向不对称垂直增益 (v8.6)
         * 下潜 → 增强至与定深阶段1一致 (尾推1250/垂推1480)
         * 上浮 → 保持 v8.5 增益不变 (1180/1400, 上浮更容易)
         * v9.0: 手动模式按档位增益放大 */
        if (fz >= 0) {
            fz_tail = fz * MANUAL_DIVE_FZ_TAIL * GEAR_DIVE_GAIN[gi];
            fz_vert = fz * MANUAL_DIVE_FZ_VERT * GEAR_DIVE_GAIN[gi];
        } else {
            fz_tail = fz * FZ_GAIN_TAIL * GEAR_SURF_GAIN[gi];
            fz_vert = fz * FZ_GAIN_VERT * GEAR_SURF_GAIN[gi];
        }
    }

    /* 下潜/上浮时尾推不参与Yaw-pitch耦合, 保持4电机一致 */
    double a0, a1, a2, a3;
    if (fabs(fz_tail) > 0.001) {
        thrust_allocate(fx, fy, 0.0, 0.0, 0.0, mz_tail, u_fxonly);
        a0 = u_fxonly[0]; a1 = u_fxonly[1]; a2 = u_fxonly[2]; a3 = u_fxonly[3];
    } else {
        a0 = u_all[0]; a1 = u_all[1]; a2 = u_all[2]; a3 = u_all[3];
    }

    /* ── 转换为 RPM ── */
    int g[8] = {0,0,0,0,0,0,0,0};
    g[0] = norm_to_rpm(a0 - fz_tail, TAIL_RPM_MIN, TAIL_RPM_MAX);
    g[1] = norm_to_rpm(a1 + fz_tail, TAIL_RPM_MIN, TAIL_RPM_MAX);
    g[2] = norm_to_rpm(a2 + fz_tail, TAIL_RPM_MIN, TAIL_RPM_MAX);
    g[3] = norm_to_rpm(a3 - fz_tail, TAIL_RPM_MIN, TAIL_RPM_MAX);

    g[5] = norm_to_rpm(fz_vert, VERT_RPM_MIN, VERT_RPM_MAX);
    g[6] = norm_to_rpm(fz_vert, VERT_RPM_MIN, VERT_RPM_MAX);

    /* 尾推绝对值不超过垂推 (防 pitch 摆动) */
    if (fabs(fz_tail) > 0.01) {
        int vert_abs = abs(g[5]);
        int v6 = abs(g[6]);
        if (v6 > vert_abs) vert_abs = v6;
        for (int i = 0; i < 4; i++) {
            if (abs(g[i]) > vert_abs) {
                g[i] = (g[i] > 0) ? vert_abs : -vert_abs;
            }
        }
    }

    /* 横推 ID7: 纯 Yaw 主控 (ID7=100%mz, 尾推=50%辅助) */
    g[7] = norm_to_rpm(mz_id7, YAW_RPM_MIN, YAW_RPM_MAX);
    /* 固定深度阶段 ID7=0, 但定航向活跃时保持控制 */
    if (S.fixed_stage && !S.yaw_hold_active) {
        g[7] = 0;
    }

    /* ── Pitch 安全: 超阈值线性降推, 防翻覆 ── */
    if (S.ins_att_valid) {
        double pitch_abs = fabs(S.ins_pitch);
        if (pitch_abs > PITCH_SAFE) {
            double pitch_scale = 1.0 - (pitch_abs - PITCH_SAFE) / (PITCH_KILL - PITCH_SAFE);
            if (pitch_scale < 0.0) pitch_scale = 0.0;
            for (int i = 0; i < 8; i++) {
                g[i] = (int)((double)g[i] * pitch_scale);
            }
        }
    }

    memcpy(S.motors, g, sizeof(g));
    send_motor_rpm(g);
    sync_to_shm();

    /* ── 每秒日志 ── */
    if (S.hb_log_count % 10 == 0) {
        const char *mode_tag = "MANUAL";
        double err = 0.0;
        if (S.last_dive_flag > 0.1 && S.depth_valid) {
            err = S.target_depth - S.current_depth;
            if (fabs(err) > DEPTH_FIXED_THRESHOLD) mode_tag = (err > 0) ? "FIXED" : "FIXED_UP";
            else mode_tag = "PID";
        }
        fprintf(stderr,
            "v9.0 %s | gear=%d 深=%.3f/tar=%.2f err=%+.3f pit=%.1f rol=%.1f yaw=%.1f | "
            "fz=%+.3f mx=%+.3f my=%+.3f mz=%+.3f | "
            "T=%+d %+d %+d %+d V=%+d %+d Y=%+d\n",
            mode_tag, S.gear,
            S.depth_valid ? S.current_depth : -1.0, S.target_depth, err,
            S.ins_pitch, S.ins_roll, S.ins_yaw,
            fz, mx, my, mz_id7,
            S.motors[0], S.motors[1], S.motors[2], S.motors[3],
            S.motors[5], S.motors[6], S.motors[7]);
    }
}


/* ========================================================================
 *  超时检查 (与 Python timeout_check 一致)
 * ======================================================================== */
static void timeout_check(void) {
    if (wall_now_sec() - S.last_cmd_time > TIMEOUT_SEC) {
        int any_nonzero = 0;
        for (int i = 0; i < 8; i++) if (S.motors[i]) { any_nonzero = 1; break; }
        if (any_nonzero) {
            fprintf(stderr, "[MC] 命令超时, 自动停止所有电机\n");
        }
        S.last_move = 0.0; S.last_up = 0.0; S.last_yaw = 0.0;
        S.last_dive_flag = 0.0;
        memset(S.motors, 0, sizeof(S.motors));
        S.depth_err_i = 0.0; S.depth_pid_out = 0.0;
        S.roll_err_i = 0.0;  S.roll_pid_out = 0.0;
        S.pitch_err_i = 0.0; S.pitch_pid_out = 0.0;
        S.yaw_err_i = 0.0;   S.yaw_pid_out = 0.0;
        S.yaw_captured = 0;
        S.yaw_hold_active = 0;
        send_motor_rpm(S.motors);
    }
}


/* ========================================================================
 *  信号处理
 * ======================================================================== */
static void on_signal(int sig) {
    (void)sig;
    g_running = 0;
    int g[8] = {0,0,0,0,0,0,0,0};
    send_motor_rpm(g);
    can_close_all();
    _exit(0);
}


/* ========================================================================
 *  main
 * ======================================================================== */
int main(int argc, char **argv) {
    (void)argc; (void)argv;

    memset(&S, 0, sizeof(S));
    S.gear = 1;
    S.target_depth = 0.5;
    S.last_cmd_time = wall_now_sec();
    S.last_yaw_pid_time = wall_now_sec();

    /* 信号 */
    signal(SIGTERM, on_signal);
    signal(SIGINT,  on_signal);
    signal(SIGHUP,   on_signal);

    /* 共享内存 */
    if (!shm_init()) {
        fprintf(stderr, "[FATAL] 共享内存初始化失败, 退出\n");
        return 1;
    }

    /* CAN */
    if (!can_init()) {
        fprintf(stderr, "[FATAL] CAN 初始化失败, 退出\n");
        return 1;
    }

    fprintf(stderr, "[MC] motor_controller v9.0 (C) 启动, 10Hz 心跳\n");
    fprintf(stderr, "[MC] 档位: 新1档=原3档全速, 2档递增, 3档=硬件上限\n");
    fprintf(stderr, "[MC] 满杆RPM: 尾推 1235/1400/1550, 下潜 1480/1520/1550, "
                    "上浮 1400/1470/1550, ID7 1280/1340/1400\n");

    /* 10Hz 主循环 (100ms) */
    struct timespec ts_period = {0, 100 * 1000 * 1000}; /* 100ms */
    while (g_running) {
        heartbeat_tick();
        timeout_check();
        nanosleep(&ts_period, NULL);
    }

    /* 清理 */
    int g[8] = {0,0,0,0,0,0,0,0};
    send_motor_rpm(g);
    can_close_all();
    if (g_shm) munmap(g_shm, SHM_SIZE);
    return 0;
}
