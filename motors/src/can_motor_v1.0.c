/*
 * can_motor_v1.0.c
 * 策海科技水下推进器 7 电机 CAN 控制（多轴并发心跳版）
 *
 * 协议：策海 CAN通信协议（油门控制版本）V1.1
 *
 * 电机布局（物理安装）：
 *   ID 0,2    — 尾推 A 组（同向安装）
 *   ID 1,3    — 尾推 B 组（与 0,2 安装朝向相反）
 *   ID 5,6    — 垂推（均朝下安装，5正转=向下推水，6反转=向下推水，合力向上）
 *   ID 7      — 侧推（左右转向）
 *
 * 方向约定（用户输入视角）：
 *   move  正数=前进  负数=后退   → ID0,2 直接用，ID1,3 自动取反
 *   up    正数=上浮  负数=下潜   → ID5 直接用，ID6 自动取反（朝向相反）
 *   yaw   正数=右转  负数=左转   → ID7 直接用
 *
 * 重要：电机 3 秒无信号自动停机。所有运动命令持续心跳(500ms)，Ctrl+C 停止。
 *
 * 编译: gcc -O2 -o can_motor can_motor_v1.0.c
 *
 * 用法:
 *   ./can_motor init                         上电初始化（发速度0使能）
 *   ./can_motor stop                         全部停止
 *   ./can_motor move <rpm>                   前进(+)/后退(-)，持续
 *   ./can_motor up <rpm>                     上浮(+)/下潜(-)，持续
 *   ./can_motor yaw <rpm>                    右转(+)/左转(-)，持续
 *   ./can_motor motor <id> <rpm>             单台直接控制（不做方向映射）
 *   ./can_motor run [move=<v>] [up=<v>] [yaw=<v>]   多轴同时控制（持续）
 *   ./can_motor run [0=<v>] [1=<v>] ... [7=<v>]     按 ID 精确控制（不做映射）
 *   ./can_motor status [sec]                 读取反馈（默认2秒）
 *   ./can_motor setfreq <code>               反馈频率 0=关 1=10Hz 2=50Hz 3=100Hz
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <signal.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

/* ─────────────────────────────────────────────
   全局配置
   ───────────────────────────────────────────── */
#define CAN_INTERFACE       "can0"
#define CAN_BITRATE         500000

#define CTRL_FRAME_0        0x200   /* 电机 0~3 */
#define CTRL_FRAME_1        0x201   /* 电机 4~7 */
#define FB_BASE             0x300
#define SETUP_FRAME         0x400

#define MOTOR_COUNT         7
static const uint8_t MOTOR_IDS[MOTOR_COUNT] = {0, 1, 2, 3, 5, 6, 7};

#define HEARTBEAT_INTERVAL_MS   500
#define STATUS_DEFAULT_SEC      2

/* ─────────────────────────────────────────────
   全局电机转速状态
   g_motor_rpm[id] = 用户设定的目标转速（逻辑值）
   build_ctrl_* 负责按实际安装方向做翻转
   ───────────────────────────────────────────── */
static int g_motor_rpm[8] = {0};
static volatile sig_atomic_t g_running = 0;

/* ─────────────────────────────────────────────
   信号处理
   ───────────────────────────────────────────── */
static void sig_handler(int sig) { (void)sig; g_running = 0; }

static void setup_signal(void)
{
    struct sigaction sa = {0};
    sa.sa_handler = sig_handler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
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

static int recv_frame(int sock, struct can_frame *f, int timeout_ms)
{
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(sock, &fds);
    struct timeval tv = { timeout_ms / 1000, (timeout_ms % 1000) * 1000 };
    if (select(sock + 1, &fds, NULL, NULL, &tv) <= 0) return -1;
    return (read(sock, f, sizeof(*f)) > 0) ? 0 : -1;
}

/* ─────────────────────────────────────────────
   CAN 接口
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
    if (is_can_up()) { printf("[CAN] %s already UP.\n", CAN_INTERFACE); return 0; }
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "ip link set %s type can bitrate %d", CAN_INTERFACE, CAN_BITRATE);
    if (system(cmd) != 0) { fprintf(stderr, "[CAN] set bitrate failed\n"); return -1; }
    snprintf(cmd, sizeof(cmd), "ip link set %s up", CAN_INTERFACE);
    if (system(cmd) != 0) { fprintf(stderr, "[CAN] bring up failed\n"); return -1; }
    printf("[CAN] %s @ %d bps UP.\n", CAN_INTERFACE, CAN_BITRATE);
    return 0;
}

/* ─────────────────────────────────────────────
   rpm → cmd 编码
   bit10~0: 转速绝对值；bit11: 0=正转 1=反转
   ───────────────────────────────────────────── */
static uint16_t rpm_to_cmd(int rpm)
{
    if (rpm == 0) return 0;
    uint16_t abs_v = (rpm < 0) ? (uint16_t)(-rpm) : (uint16_t)rpm;
    uint16_t cmd = abs_v & 0x07FF;
    if (rpm < 0) cmd |= 0x0800;
    return cmd;
}

static void write_cmd_le(uint8_t *buf, uint16_t cmd)
{
    buf[0] = (cmd >> 0) & 0xFF;
    buf[1] = (cmd >> 8) & 0xFF;
}

/* ─────────────────────────────────────────────
   构建控制帧

   帧 0x200（电机 0~3）方向映射：
     ID 0,2  安装同向  → 直接用 g_motor_rpm[id]
     ID 1,3  安装反向  → 取反后再编码

   帧 0x201（电机 4~7）方向映射：
     ID 4  未使用，发0
     ID 5  安装朝下，正转=向下推水=向上合力  → 直接用 g_motor_rpm[5]
     ID 6  安装朝下，与 ID5 朝向相同，
           要产生和 ID5 相同方向的合力需反转   → 取反 g_motor_rpm[6]
     ID 7  侧推，直接用
   ───────────────────────────────────────────── */
static void build_ctrl_200(uint8_t out[8])
{
    write_cmd_le(out + 0, rpm_to_cmd( g_motor_rpm[0]));  /* ID0 直接 */
    write_cmd_le(out + 2, rpm_to_cmd(-g_motor_rpm[1]));  /* ID1 反向 */
    write_cmd_le(out + 4, rpm_to_cmd( g_motor_rpm[2]));  /* ID2 直接 */
    write_cmd_le(out + 6, rpm_to_cmd(-g_motor_rpm[3]));  /* ID3 反向 */
}

static void build_ctrl_201(uint8_t out[8])
{
    write_cmd_le(out + 0, 0);                             /* ID4 未用 */
    write_cmd_le(out + 2, rpm_to_cmd( g_motor_rpm[5]));  /* ID5 直接 */
    write_cmd_le(out + 4, rpm_to_cmd(-g_motor_rpm[6]));  /* ID6 反向（同朝向） */
    write_cmd_le(out + 6, rpm_to_cmd( g_motor_rpm[7]));  /* ID7 直接 */
}

static void send_all_motors(int sock)
{
    uint8_t f200[8], f201[8];
    build_ctrl_200(f200);
    build_ctrl_201(f201);
    send_frame(sock, CTRL_FRAME_0, f200, 8);
    send_frame(sock, CTRL_FRAME_1, f201, 8);
}

static void send_all_stop(int sock)
{
    memset(g_motor_rpm, 0, sizeof(g_motor_rpm));
    uint8_t zero[8] = {0};
    send_frame(sock, CTRL_FRAME_0, zero, 8);
    send_frame(sock, CTRL_FRAME_1, zero, 8);
}

/* ─────────────────────────────────────────────
   心跳循环
   持续发送 g_motor_rpm[] 对应的帧，Ctrl+C 后发停止帧退出
   ───────────────────────────────────────────── */
static void heartbeat_loop(int sock)
{
    g_running = 1;
    setup_signal();
    printf("[RUN] Heartbeat %dms, Ctrl+C to stop.\n", HEARTBEAT_INTERVAL_MS);
    printf("[RUN] Motors: 0=%+d  1=%+d  2=%+d  3=%+d  5=%+d  6=%+d  7=%+d rpm\n",
           g_motor_rpm[0], g_motor_rpm[1], g_motor_rpm[2], g_motor_rpm[3],
           g_motor_rpm[5], g_motor_rpm[6], g_motor_rpm[7]);

    while (g_running) {
        send_all_motors(sock);
        usleep((useconds_t)HEARTBEAT_INTERVAL_MS * 1000);
    }

    send_all_stop(sock);
    printf("\n[STOP] Heartbeat off, all motors stopped.\n");
}

/* ─────────────────────────────────────────────
   反馈解析
   ───────────────────────────────────────────── */
static const char *fault_str(uint8_t f)
{
    switch (f) {
        case 0: return "OK";
        case 1: return "电流反馈故障";
        case 2: return "功率器件故障";
        case 3: return "启动失败";
        case 4: return "启动保护";
        case 5: return "过流";
        case 6: return "过热";
        case 7: return "欠压";
        case 8: return "过压";
        default: return "未知";
    }
}

static const char *motor_role(uint8_t id)
{
    switch (id) {
        case 0: return "尾推A-0";
        case 1: return "尾推B-1";
        case 2: return "尾推A-2";
        case 3: return "尾推B-3";
        case 5: return "垂推-5";
        case 6: return "垂推-6";
        case 7: return "侧推-7";
        default: return "未知";
    }
}

static void print_motor_fb(uint8_t mid, const struct can_frame *f)
{
    if (f->can_dlc < 7) return;
    int16_t speed  = (int16_t)(f->data[0] | (f->data[1] << 8));
    int16_t torque = (int16_t)(f->data[2] | (f->data[3] << 8));
    uint8_t volt   = f->data[4];
    int8_t  temp   = (int8_t)f->data[5];
    uint8_t status = f->data[6];
    uint8_t fault  = (f->can_dlc >= 8) ? f->data[7] : 0;
    int mode = (status >> 4) & 0x03;
    const char *modes[] = {"速度闭环", "电压开环", "功率闭环", "未知"};
    printf("  [ID%d %-8s]  %+6d rpm  %5.2fA  %3uV  %+3d℃  %s  %s\n",
           mid, motor_role(mid), speed, torque * 0.01f, volt, temp,
           modes[mode < 3 ? mode : 3], fault_str(fault));
}

static void read_status(int sock, int dur_sec)
{
    printf("[STATUS] Listening %d sec...\n", dur_sec);
    struct timespec t0, tn;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    int got[16] = {0};
    int count = 0;
    long timeout_ms = (long)dur_sec * 1000;

    while (count < MOTOR_COUNT) {
        clock_gettime(CLOCK_MONOTONIC, &tn);
        long ms = (tn.tv_sec - t0.tv_sec) * 1000 + (tn.tv_nsec - t0.tv_nsec) / 1000000;
        if (ms > timeout_ms) break;
        struct can_frame f;
        if (recv_frame(sock, &f, 100) < 0) continue;
        if (f.can_id < FB_BASE || f.can_id > FB_BASE + 15) continue;
        uint8_t mid = f.can_id - FB_BASE;
        int valid = 0;
        for (int i = 0; i < MOTOR_COUNT; i++) if (MOTOR_IDS[i] == mid) { valid = 1; break; }
        if (!valid) continue;
        if (!got[mid]) { got[mid] = 1; count++; }
        print_motor_fb(mid, &f);
    }
    for (int i = 0; i < MOTOR_COUNT; i++)
        if (!got[MOTOR_IDS[i]])
            printf("  [ID%d] No feedback\n", MOTOR_IDS[i]);
}

/* ─────────────────────────────────────────────
   参数解析
   ───────────────────────────────────────────── */
static int parse_int(const char *s)
{
    char *ep; errno = 0;
    long v = strtol(s, &ep, 10);
    if (errno || *ep) { fprintf(stderr, "Invalid number: '%s'\n", s); exit(1); }
    return (int)v;
}

static void usage(const char *prog)
{
    fprintf(stderr,
        "Usage:\n"
        "  %s init                         上电初始化（发速度0使能）\n"
        "  %s stop                         全部停止\n"
        "  %s move <rpm>                   前进(+)/后退(-)\n"
        "  %s up <rpm>                     上浮(+)/下潜(-)\n"
        "  %s yaw <rpm>                    右转(+)/左转(-)\n"
        "  %s motor <id> <rpm>             单台直接控制(不做方向映射)\n"
        "  %s run [move=V] [up=V] [yaw=V]  多轴同时控制（持续心跳）\n"
        "  %s run [0=V] [1=V] [2=V] ...    按 ID 精确控制（不做方向映射）\n"
        "  %s status [sec]                 读取反馈（默认2秒）\n"
        "  %s setfreq <0~6>                反馈频率(0=关 1=10Hz 2=50Hz 3=100Hz 4=200Hz)\n"
        "\n"
        "电机方向映射（move/up/yaw 命令）:\n"
        "  move +N → ID0,2 正转, ID1,3 反转 (全部前进)\n"
        "  up   +N → ID5 正转, ID6 反转 (均朝下安装，同推力方向=上浮)\n"
        "  yaw  +N → ID7 正转 (右转)\n"
        "\n"
        "run 命令示例:\n"
        "  %s run move=500 up=200            前进同时上浮\n"
        "  %s run move=400 up=-150 yaw=200   前进+下潜+右转\n"
        "  %s run 0=600 1=600 5=300 6=300    按 ID 直接设转速（不做方向映射）\n"
        "\n"
        "所有运动命令持续发送心跳帧，Ctrl+C 退出并自动停机。\n",
        prog,prog,prog,prog,prog,prog,prog,prog,prog,prog,prog,prog,prog);
}

/* ─────────────────────────────────────────────
   run 命令解析器
   支持两种格式:
     move=V  up=V  yaw=V  → 走高级映射
     0=V  1=V ... 7=V     → 直接设 g_motor_rpm[id]（不做翻转）
   可混用，后写的覆盖先写的
   ───────────────────────────────────────────── */
static int parse_run_args(int argc, char *argv[])
{
    /* argv[0] 已经是 "run" 后的第一个参数 */
    for (int i = 0; i < argc; i++) {
        char *eq = strchr(argv[i], '=');
        if (!eq) {
            fprintf(stderr, "[run] Bad argument (expected key=value): '%s'\n", argv[i]);
            return -1;
        }
        *eq = '\0';
        const char *key = argv[i];
        int val = parse_int(eq + 1);

        if (strcmp(key, "move") == 0) {
            /* 高级：设所有 4 个尾推（映射在 build_ctrl 里做） */
            g_motor_rpm[0] = val;
            g_motor_rpm[1] = val;
            g_motor_rpm[2] = val;
            g_motor_rpm[3] = val;
        } else if (strcmp(key, "up") == 0) {
            /* 高级：设垂推（ID5 直接，ID6 在 build_ctrl 取反） */
            g_motor_rpm[5] = val;
            g_motor_rpm[6] = val;
        } else if (strcmp(key, "yaw") == 0) {
            g_motor_rpm[7] = val;
        } else {
            /* 按 ID 直接设置 */
            char *ep;
            long id = strtol(key, &ep, 10);
            if (*ep || id < 0 || id > 7) {
                fprintf(stderr, "[run] Unknown key '%s', expect move/up/yaw or 0~7\n", key);
                return -1;
            }
            g_motor_rpm[(int)id] = val;
        }
        *eq = '='; /* 还原字符串，不影响调用者 */
    }
    return 0;
}

/* ─────────────────────────────────────────────
   MAIN
   ───────────────────────────────────────────── */
int main(int argc, char *argv[])
{
    if (argc < 2) { usage(argv[0]); return 1; }

    if (setup_can() != 0) return 1;

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) { perror("socket"); return 1; }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, CAN_INTERFACE, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); close(s); return 1; }

    struct sockaddr_can addr = {0};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind"); close(s); return 1;
    }

    /* ── init ── */
    if (strcmp(argv[1], "init") == 0) {
        printf("[INIT] Enabling all motors (speed=0)...\n");
        memset(g_motor_rpm, 0, sizeof(g_motor_rpm));
        uint8_t zero[8] = {0};
        send_frame(s, CTRL_FRAME_0, zero, 8);
        usleep(50000);
        send_frame(s, CTRL_FRAME_1, zero, 8);
        usleep(50000);
        send_frame(s, CTRL_FRAME_0, zero, 8);
        usleep(50000);
        send_frame(s, CTRL_FRAME_1, zero, 8);
        printf("[INIT] Done. Ready for control.\n");
        printf("[INIT] NOTE: Motors stop after 3s without command!\n");
        close(s); return 0;
    }

    /* ── stop ── */
    if (strcmp(argv[1], "stop") == 0) {
        send_all_stop(s);
        printf("[STOP] All motors stopped.\n");
        close(s); return 0;
    }

    /* ── status [sec] ── */
    if (strcmp(argv[1], "status") == 0) {
        int dur = STATUS_DEFAULT_SEC;
        if (argc >= 3) { dur = parse_int(argv[2]); if (dur < 1) dur = 1; }
        read_status(s, dur);
        close(s); return 0;
    }

    /* ── setfreq <code> ── */
    if (strcmp(argv[1], "setfreq") == 0) {
        if (argc < 3) { fprintf(stderr, "Usage: %s setfreq <0~6>\n", argv[0]); close(s); return 1; }
        int freq = parse_int(argv[2]);
        if (freq < 0 || freq > 6) { fprintf(stderr, "code must be 0~6\n"); close(s); return 1; }
        uint8_t d[8] = {0};
        d[0] = 0x02; d[1] = (uint8_t)freq;
        send_frame(s, SETUP_FRAME, d, 8);
        printf("[SETFREQ] code=%d sent.\n", freq);
        close(s); return 0;
    }

    /* ── move <rpm> ── */
    if (strcmp(argv[1], "move") == 0) {
        if (argc < 3) { fprintf(stderr, "Usage: %s move <rpm>\n", argv[0]); close(s); return 1; }
        int v = parse_int(argv[2]);
        memset(g_motor_rpm, 0, sizeof(g_motor_rpm));
        g_motor_rpm[0] = g_motor_rpm[1] = g_motor_rpm[2] = g_motor_rpm[3] = v;
        heartbeat_loop(s);
        close(s); return 0;
    }

    /* ── up <rpm> ── */
    if (strcmp(argv[1], "up") == 0) {
        if (argc < 3) { fprintf(stderr, "Usage: %s up <rpm>\n", argv[0]); close(s); return 1; }
        int v = parse_int(argv[2]);
        memset(g_motor_rpm, 0, sizeof(g_motor_rpm));
        g_motor_rpm[5] = g_motor_rpm[6] = v;
        heartbeat_loop(s);
        close(s); return 0;
    }

    /* ── yaw <rpm> ── */
    if (strcmp(argv[1], "yaw") == 0) {
        if (argc < 3) { fprintf(stderr, "Usage: %s yaw <rpm>\n", argv[0]); close(s); return 1; }
        int v = parse_int(argv[2]);
        memset(g_motor_rpm, 0, sizeof(g_motor_rpm));
        g_motor_rpm[7] = v;
        heartbeat_loop(s);
        close(s); return 0;
    }

    /* ── motor <id> <rpm> ── 直接控制单台，不做方向映射 */
    if (strcmp(argv[1], "motor") == 0) {
        if (argc < 4) { fprintf(stderr, "Usage: %s motor <id> <rpm>\n", argv[0]); close(s); return 1; }
        int id  = parse_int(argv[2]);
        int rpm = parse_int(argv[3]);
        if (id < 0 || id > 7) { fprintf(stderr, "ID must be 0~7\n"); close(s); return 1; }
        memset(g_motor_rpm, 0, sizeof(g_motor_rpm));
        g_motor_rpm[id] = rpm;
        heartbeat_loop(s);
        close(s); return 0;
    }

    /* ── run [key=val ...] ── 多轴并发，持续心跳 */
    if (strcmp(argv[1], "run") == 0) {
        if (argc < 3) { fprintf(stderr, "Usage: %s run move=V [up=V] [yaw=V] ...\n", argv[0]); close(s); return 1; }
        memset(g_motor_rpm, 0, sizeof(g_motor_rpm));
        if (parse_run_args(argc - 2, argv + 2) < 0) { close(s); return 1; }
        heartbeat_loop(s);
        close(s); return 0;
    }

    fprintf(stderr, "Unknown command: %s\n", argv[1]);
    usage(argv[0]);
    close(s);
    return 1;
}
