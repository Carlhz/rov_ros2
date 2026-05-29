/*
 * can_demo_daemon.c
 * Kinco iSMK 双电机 PDO 守护进程
 *
 * 功能：
 *   从 stdin 读取简单文本指令，实时控制电机。
 *   持续心跳（200ms 一次 PDO），不会因 3s 超时停机。
 *   Ctrl+C / stdin EOF 时自动停止电机并退出。
 *
 * 配合使用：
 *   远端运行：sudo ./can_demo_daemon
 *   主机通过 SSH pipe 或 Python 将指令发进来
 *
 * 指令格式（每行一条，\n 结尾）：
 *   move <L> <R>         左右电机 rpm（可负）
 *   forward <speed>      前进
 *   backward <speed>     后退
 *   turn left <L> <R>    左转
 *   turn right <L> <R>   右转
 *   pivot left <speed>   左原地旋转
 *   pivot right <speed>  右原地旋转
 *   stop                 停止（但守护进程继续运行）
 *   quit / exit          停止并退出
 *
 * 编译: gcc -O2 -o can_demo_daemon can_demo_daemon.c
 * 运行: sudo ./can_demo_daemon
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <pthread.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

/* ─────────────────────────────────────────────
   配置
   ───────────────────────────────────────────── */
#define CAN_INTERFACE       "can0"
#define CAN_BITRATE         500000
#define NODE_LEFT           1
#define NODE_RIGHT          2

/* rpm → DEC 单位换算（同 v1.7） */
#define RPM_TO_DEC(rpm)     ((int32_t)((long)(rpm) * 512L * 65536L / 1875L))

#define SDO_TIMEOUT_MS      500
#define HEARTBEAT_MS        200   /* 心跳间隔，远小于 Kinco 3s 超时 */
#define MAX_RPM             3000  /* 限速（可改） */

/* ─────────────────────────────────────────────
   全局状态
   ───────────────────────────────────────────── */
static volatile sig_atomic_t g_running = 1;

/* 当前目标转速，由指令线程写，心跳线程读 */
static volatile int32_t g_left_rpm  = 0;
static volatile int32_t g_right_rpm = 0;
static pthread_mutex_t g_rpm_lock = PTHREAD_MUTEX_INITIALIZER;

static int g_sock = -1;

/* ─────────────────────────────────────────────
   信号处理
   ───────────────────────────────────────────── */
static void sig_handler(int sig) { (void)sig; g_running = 0; }

/* ─────────────────────────────────────────────
   CAN 收发（同 v1.7）
   ───────────────────────────────────────────── */
static int send_frame(int sock, uint32_t id, const uint8_t *data, uint8_t dlc)
{
    struct can_frame f = {0};
    f.can_id  = id;
    f.can_dlc = dlc;
    memcpy(f.data, data, dlc);
    return write(sock, &f, sizeof(f));
}

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
            if (cs == 0x60 || cs == 0x40) { if (out) memcpy(out, f.data, 8); return 0; }
            if (f.data[0] == 0x80) return -2;
        }
    }
}

static void sdo_write_u8(uint8_t *f, uint16_t idx, uint8_t sub, uint8_t v)
{ f[0]=0x2F; f[1]=idx&0xFF; f[2]=(idx>>8)&0xFF; f[3]=sub; f[4]=v; f[5]=f[6]=f[7]=0; }

static void sdo_write_u16(uint8_t *f, uint16_t idx, uint8_t sub, uint16_t v)
{ f[0]=0x2B; f[1]=idx&0xFF; f[2]=(idx>>8)&0xFF; f[3]=sub; f[4]=v&0xFF; f[5]=(v>>8)&0xFF; f[6]=f[7]=0; }

static void sdo_write_u32(uint8_t *f, uint16_t idx, uint8_t sub, uint32_t v)
{ f[0]=0x23; f[1]=idx&0xFF; f[2]=(idx>>8)&0xFF; f[3]=sub;
  f[4]=(v>>0)&0xFF; f[5]=(v>>8)&0xFF; f[6]=(v>>16)&0xFF; f[7]=(v>>24)&0xFF; }

static int sdo_write_ack(int sock, uint8_t node, uint8_t *sdo_buf)
{
    send_frame(sock, 0x600 + node, sdo_buf, 8);
    return wait_sdo_resp(sock, 0x580 + node, NULL);
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
    if (is_can_up()) { fprintf(stderr, "[CAN] %s already UP.\n", CAN_INTERFACE); return 0; }
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "ip link set %s type can bitrate %d", CAN_INTERFACE, CAN_BITRATE);
    if (system(cmd) != 0) return -1;
    snprintf(cmd, sizeof(cmd), "ip link set %s up", CAN_INTERFACE);
    if (system(cmd) != 0) return -1;
    fprintf(stderr, "[CAN] %s UP @ %d bps\n", CAN_INTERFACE, CAN_BITRATE);
    return 0;
}

/* ─────────────────────────────────────────────
   NMT / PDO 初始化（与 v1.7 相同逻辑）
   ───────────────────────────────────────────── */
static void nmt_send(int sock, uint8_t cs, uint8_t node)
{ uint8_t d[2] = {cs, node}; send_frame(sock, 0x000, d, 2); usleep(50000); }

static int configure_pdo(int sock, uint8_t node)
{
    uint8_t sdo[8];
    int ret = 0;
    /* RPDO1 */
    sdo_write_u32(sdo, 0x1400, 0x01, 0x80000200 + node); ret |= sdo_write_ack(sock, node, sdo);
    sdo_write_u8 (sdo, 0x1600, 0x00, 0x00);               ret |= sdo_write_ack(sock, node, sdo);
    sdo_write_u32(sdo, 0x1600, 0x01, 0x60400010);         ret |= sdo_write_ack(sock, node, sdo);
    sdo_write_u32(sdo, 0x1600, 0x02, 0x60FF0020);         ret |= sdo_write_ack(sock, node, sdo);
    sdo_write_u8 (sdo, 0x1600, 0x00, 0x02);               ret |= sdo_write_ack(sock, node, sdo);
    sdo_write_u8 (sdo, 0x1400, 0x02, 0xFE);               ret |= sdo_write_ack(sock, node, sdo);
    sdo_write_u32(sdo, 0x1400, 0x01, 0x00000200 + node);  ret |= sdo_write_ack(sock, node, sdo);
    /* TPDO1 */
    sdo_write_u32(sdo, 0x1800, 0x01, 0x80000180 + node);  ret |= sdo_write_ack(sock, node, sdo);
    sdo_write_u8 (sdo, 0x1A00, 0x00, 0x00);               ret |= sdo_write_ack(sock, node, sdo);
    sdo_write_u32(sdo, 0x1A00, 0x01, 0x60410010);         ret |= sdo_write_ack(sock, node, sdo);
    sdo_write_u32(sdo, 0x1A00, 0x02, 0x606C0020);         ret |= sdo_write_ack(sock, node, sdo);
    sdo_write_u8 (sdo, 0x1A00, 0x00, 0x02);               ret |= sdo_write_ack(sock, node, sdo);
    sdo_write_u8 (sdo, 0x1800, 0x02, 0xFE);               ret |= sdo_write_ack(sock, node, sdo);
    sdo_write_u16(sdo, 0x1800, 0x03, 0x0000);             ret |= sdo_write_ack(sock, node, sdo);
    sdo_write_u16(sdo, 0x1800, 0x05, 10);                 ret |= sdo_write_ack(sock, node, sdo);
    sdo_write_u32(sdo, 0x1800, 0x01, 0x00000180 + node);  ret |= sdo_write_ack(sock, node, sdo);
    return ret;
}

static void enable_motor_sdo(int sock, uint8_t node)
{
    uint8_t sdo[8];
    sdo_write_u16(sdo, 0x6040, 0x00, 0x0006); send_frame(sock, 0x600+node, sdo, 8);
    wait_sdo_resp(sock, 0x580+node, NULL); usleep(50000);
    sdo_write_u16(sdo, 0x6040, 0x00, 0x0007); send_frame(sock, 0x600+node, sdo, 8);
    wait_sdo_resp(sock, 0x580+node, NULL); usleep(50000);
    sdo_write_u8 (sdo, 0x6060, 0x00, 0x03);   send_frame(sock, 0x600+node, sdo, 8);
    wait_sdo_resp(sock, 0x580+node, NULL); usleep(50000);
    sdo_write_u16(sdo, 0x6040, 0x00, 0x000F); send_frame(sock, 0x600+node, sdo, 8);
    wait_sdo_resp(sock, 0x580+node, NULL); usleep(50000);
}

static int full_init(int sock)
{
    fprintf(stderr, "[INIT] Reset Comm + Pre-op...\n");
    uint8_t d2[2] = {0x82, 0x00}; send_frame(sock, 0x000, d2, 2); sleep(2);
    nmt_send(sock, 0x80, 0x00); usleep(200000);

    fprintf(stderr, "[INIT] Configure PDO...\n");
    if (configure_pdo(sock, NODE_LEFT)  < 0) { fprintf(stderr, "[INIT] PDO LEFT failed\n");  return -1; }
    if (configure_pdo(sock, NODE_RIGHT) < 0) { fprintf(stderr, "[INIT] PDO RIGHT failed\n"); return -1; }

    fprintf(stderr, "[INIT] Enable motors...\n");
    uint8_t sdo[8];
    sdo_write_u8(sdo, 0x6060, 0x00, 0x03);
    sdo_write_ack(sock, NODE_LEFT,  sdo);
    sdo_write_ack(sock, NODE_RIGHT, sdo);
    usleep(50000);

    enable_motor_sdo(sock, NODE_LEFT);
    enable_motor_sdo(sock, NODE_RIGHT);

    nmt_send(sock, 0x01, 0x00); usleep(100000);

    /* 发速度 0 */
    uint8_t zero[6] = {0x0F, 0x00, 0x00, 0x00, 0x00, 0x00};
    send_frame(sock, 0x200 + NODE_LEFT,  zero, 6);
    send_frame(sock, 0x200 + NODE_RIGHT, zero, 6);

    fprintf(stderr, "[INIT] Both motors ready.\n");
    return 0;
}

/* ─────────────────────────────────────────────
   PDO 速度发送
   ───────────────────────────────────────────── */
static void pdo_set_velocity(int sock, uint8_t node, int32_t rpm)
{
    /* 限速 */
    if (rpm >  MAX_RPM) rpm =  MAX_RPM;
    if (rpm < -MAX_RPM) rpm = -MAX_RPM;

    int32_t dec = RPM_TO_DEC(rpm);
    uint8_t d[6];
    d[0] = 0x0F; d[1] = 0x00;   /* 控制字 Enable Operation */
    d[2] = (dec >>  0) & 0xFF;
    d[3] = (dec >>  8) & 0xFF;
    d[4] = (dec >> 16) & 0xFF;
    d[5] = (dec >> 24) & 0xFF;
    send_frame(sock, 0x200 + node, d, 6);
}

/* ─────────────────────────────────────────────
   心跳线程：每 HEARTBEAT_MS 发一次 PDO
   ───────────────────────────────────────────── */
static void *heartbeat_thread(void *arg)
{
    int sock = *(int *)arg;
    while (g_running) {
        int32_t l, r;
        pthread_mutex_lock(&g_rpm_lock);
        l = g_left_rpm;
        r = g_right_rpm;
        pthread_mutex_unlock(&g_rpm_lock);

        pdo_set_velocity(sock, NODE_LEFT,  l);
        pdo_set_velocity(sock, NODE_RIGHT, r);

        usleep((useconds_t)HEARTBEAT_MS * 1000);
    }
    return NULL;
}

/* ─────────────────────────────────────────────
   指令解析
   ───────────────────────────────────────────── */
static void clamp_rpm(int32_t *v)
{
    if (*v >  MAX_RPM) *v =  MAX_RPM;
    if (*v < -MAX_RPM) *v = -MAX_RPM;
}

/* 返回 1=继续  0=退出 */
static int process_line(const char *line)
{
    /* 跳过前导空格 */
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0' || *line == '#') return 1;

    int32_t L = 0, R = 0;
    int updated = 0;
    long a, b;

    /* move <L> <R> */
    if (sscanf(line, "move %ld %ld", &a, &b) == 2) {
        L = (int32_t)a; R = (int32_t)b;
        clamp_rpm(&L); clamp_rpm(&R);
        updated = 1;
        fprintf(stderr, "[CMD] move L=%+d R=%+d rpm\n", L, R);
    }
    /* forward <speed> */
    else if (sscanf(line, "forward %ld", &a) == 1) {
        a = a < 0 ? -a : a;   /* 保证正数 */
        L = (int32_t)a; R = -(int32_t)a;
        updated = 1;
        fprintf(stderr, "[CMD] forward %+ld rpm\n", a);
    }
    /* backward <speed> */
    else if (sscanf(line, "backward %ld", &a) == 1) {
        a = a < 0 ? -a : a;
        L = -(int32_t)a; R = (int32_t)a;
        updated = 1;
        fprintf(stderr, "[CMD] backward %+ld rpm\n", a);
    }
    /* turn left <L> <R> */
    else if (sscanf(line, "turn left %ld %ld", &a, &b) == 2) {
        L = (int32_t)a; R = (int32_t)b;
        updated = 1;
        fprintf(stderr, "[CMD] turn left L=%+d R=%+d\n", L, R);
    }
    /* turn right <L> <R> */
    else if (sscanf(line, "turn right %ld %ld", &a, &b) == 2) {
        L = (int32_t)a; R = (int32_t)b;
        updated = 1;
        fprintf(stderr, "[CMD] turn right L=%+d R=%+d\n", L, R);
    }
    /* pivot left <speed> */
    else if (sscanf(line, "pivot left %ld", &a) == 1) {
        L = 0; R = (int32_t)a;
        updated = 1;
        fprintf(stderr, "[CMD] pivot left R=%+d\n", R);
    }
    /* pivot right <speed> */
    else if (sscanf(line, "pivot right %ld", &a) == 1) {
        L = (int32_t)a; R = 0;
        updated = 1;
        fprintf(stderr, "[CMD] pivot right L=%+d\n", L);
    }
    /* stop */
    else if (strncmp(line, "stop", 4) == 0) {
        L = R = 0; updated = 1;
        fprintf(stderr, "[CMD] stop\n");
    }
    /* quit / exit */
    else if (strncmp(line, "quit", 4) == 0 || strncmp(line, "exit", 4) == 0) {
        fprintf(stderr, "[CMD] quit\n");
        return 0;
    }
    else {
        fprintf(stderr, "[CMD] Unknown: %s\n", line);
        return 1;
    }

    if (updated) {
        pthread_mutex_lock(&g_rpm_lock);
        g_left_rpm  = L;
        g_right_rpm = R;
        pthread_mutex_unlock(&g_rpm_lock);
    }
    return 1;
}

/* ─────────────────────────────────────────────
   MAIN
   ───────────────────────────────────────────── */
int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    /* 信号处理 */
    struct sigaction sa = {0};
    sa.sa_handler = sig_handler;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP,  &sa, NULL);

    /* CAN 接口 */
    if (setup_can() != 0) return 1;

    /* 打开 socket */
    g_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (g_sock < 0) { perror("socket"); return 1; }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, CAN_INTERFACE, IFNAMSIZ - 1);
    if (ioctl(g_sock, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); close(g_sock); return 1; }

    struct sockaddr_can addr = {0};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(g_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind"); close(g_sock); return 1;
    }

    /* CAN ID 过滤（只收 SDO resp + TPDO1，与 v1.7 相同） */
    {
        struct can_filter rf[4];
        rf[0].can_id = 0x581; rf[0].can_mask = 0x7FF;
        rf[1].can_id = 0x582; rf[1].can_mask = 0x7FF;
        rf[2].can_id = 0x181; rf[2].can_mask = 0x7FF;
        rf[3].can_id = 0x182; rf[3].can_mask = 0x7FF;
        setsockopt(g_sock, SOL_CAN_RAW, CAN_RAW_FILTER, rf, sizeof(rf));
    }

    /* 初始化 */
    if (full_init(g_sock) < 0) { close(g_sock); return 1; }

    /* 设置 stdin 为非阻塞，避免 getline 阻塞期间心跳停 */
    /* 用心跳线程来处理，stdin 保持阻塞即可 */

    /* 启动心跳线程 */
    pthread_t hb_tid;
    if (pthread_create(&hb_tid, NULL, heartbeat_thread, &g_sock) != 0) {
        perror("pthread_create"); close(g_sock); return 1;
    }

    fprintf(stderr, "[DAEMON] Ready. Waiting for commands on stdin...\n");
    fprintf(stderr, "[DAEMON] Commands: move <L> <R> | forward N | backward N | "
                    "turn left L R | turn right L R | pivot left N | pivot right N | stop | quit\n");

    /* 主循环：读 stdin 指令 */
    char *line = NULL;
    size_t len = 0;
    ssize_t nread;
    while (g_running && (nread = getline(&line, &len, stdin)) != -1) {
        /* 去掉末尾换行 */
        if (nread > 0 && line[nread-1] == '\n') line[nread-1] = '\0';
        if (nread > 1 && line[nread-2] == '\r') line[nread-2] = '\0';

        if (!process_line(line)) {
            g_running = 0;
            break;
        }
    }
    free(line);

    /* 退出：停止心跳，发停止帧 */
    g_running = 0;
    pthread_join(hb_tid, NULL);

    /* 发停止 PDO */
    uint8_t stop_d[6] = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00};
    send_frame(g_sock, 0x200 + NODE_LEFT,  stop_d, 6);
    send_frame(g_sock, 0x200 + NODE_RIGHT, stop_d, 6);

    fprintf(stderr, "[DAEMON] Motors stopped. Bye.\n");
    close(g_sock);
    return 0;
}
