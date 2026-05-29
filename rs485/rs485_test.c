/*
 * rs485_test.c  v2  - RK3588 RS485 双串口互发测试（含 RS485 ioctl）
 *
 * 硬件：RS485 UART3 = /dev/ttyS3 (feb60000)
 *       RS485 UART5 = /dev/ttyS5 (feb80000)
 * 接线：ttyS3 A1 ←→ ttyS5 A2  |  B1 ←→ B2  |  共地 GNDI1
 *
 * 编译（x86 虚拟机交叉编译）：
 *   aarch64-linux-gnu-gcc -O2 -o rs485_test rs485_test.c
 *
 * 运行：
 *   ./rs485_test /dev/ttyS3 /dev/ttyS5 115200
 *   ./rs485_test /dev/ttyS5 /dev/ttyS3 115200
 *
 * 测试模式（可选）：
 *   ./rs485_test /dev/ttyS3 /dev/ttyS5 115200 noioctl   # 不启用RS485模式（调试用）
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

/* ── 测试参数 ─────────────────────────────────────── */
#define TEST_FRAMES       20
#define SEND_INTERVAL_MS  600     /* RS485半双工需要更长间隔等待换向 */
#define RX_TIMEOUT_SEC    (TEST_FRAMES * SEND_INTERVAL_MS / 1000 + 6)

/* ── 帧格式（8字节）──────────────────────────────────
 * [0] 0xAB 魔数  [1] SEQ  [2~6] 'RS485'  [7] XOR校验
 * ─────────────────────────────────────────────────── */
#define FRAME_LEN   8
#define FRAME_MAGIC 0xAB

static void frame_build(uint8_t *buf, uint8_t seq)
{
    buf[0] = FRAME_MAGIC; buf[1] = seq;
    buf[2]='R'; buf[3]='S'; buf[4]='4'; buf[5]='8'; buf[6]='5';
    uint8_t x = 0;
    for (int i = 0; i < 7; i++) x ^= buf[i];
    buf[7] = x;
}

static int frame_check(const uint8_t *buf)
{
    if (buf[0] != FRAME_MAGIC) return -1;
    uint8_t x = 0;
    for (int i = 0; i < 7; i++) x ^= buf[i];
    return (x == buf[7]) ? 0 : -2;
}

/* ── 串口初始化 ───────────────────────────────────── */
static int serial_open(const char *dev, int baud, int rs485_mode)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "[ERROR] 打开 %s 失败: %s\n", dev, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(fd, &tty);

    speed_t spd;
    switch (baud) {
        case 4800:   spd = B4800;   break;
        case 9600:   spd = B9600;   break;
        case 19200:  spd = B19200;  break;
        case 38400:  spd = B38400;  break;
        case 57600:  spd = B57600;  break;
        case 115200: spd = B115200; break;
        case 230400: spd = B230400; break;
        default:     spd = B115200;
    }
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    tty.c_cflag  = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_iflag  = 0;
    tty.c_oflag  = 0;
    tty.c_lflag  = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &tty);

    /* ── RS485 模式（关键）────────────────────────────
     * 启用后内核在发送时自动拉高 RTS（发送使能），
     * 发送完毕后自动拉低（切回接收），无需手动 GPIO 控制
     * ─────────────────────────────────────────────── */
    if (rs485_mode) {
        struct serial_rs485 rs485cfg;
        memset(&rs485cfg, 0, sizeof(rs485cfg));

        /* 先读取当前配置 */
        if (ioctl(fd, TIOCGRS485, &rs485cfg) < 0) {
            fprintf(stderr, "[WARN] TIOCGRS485 不支持 (%s)，继续使用普通模式\n",
                    strerror(errno));
        } else {
            rs485cfg.flags |= SER_RS485_ENABLED;          /* 使能RS485 */
            rs485cfg.flags |= SER_RS485_RTS_ON_SEND;      /* 发送时 RTS=1 */
            rs485cfg.flags &= ~SER_RS485_RTS_AFTER_SEND;  /* 发后  RTS=0 */
            rs485cfg.delay_rts_before_send = 0;            /* 换向延迟 ms */
            rs485cfg.delay_rts_after_send  = 0;

            if (ioctl(fd, TIOCSRS485, &rs485cfg) < 0) {
                fprintf(stderr, "[WARN] TIOCSRS485 设置失败 (%s)，继续使用普通模式\n",
                        strerror(errno));
            } else {
                printf("[INFO] %s RS485 模式已启用\n", dev);
            }
        }
    }

    return fd;
}

/* ── 子进程：接收端 ─────────────────────────────────── */
static void rx_process(const char *dev, int baud, int rs485_mode)
{
    printf("[RX] 启动接收端 %s @ %d baud\n", dev, baud);
    fflush(stdout);

    int fd = serial_open(dev, baud, rs485_mode);
    if (fd < 0) exit(0);

    int recv_ok = 0, recv_bad = 0;
    uint8_t rbuf[FRAME_LEN * 8];
    int rpos = 0;
    time_t deadline = time(NULL) + RX_TIMEOUT_SEC;

    while (time(NULL) < deadline) {
        uint8_t tmp[128];
        int n = read(fd, tmp, sizeof(tmp));
        if (n > 0) {
            if (rpos + n > (int)sizeof(rbuf)) {
                memmove(rbuf, rbuf + FRAME_LEN, rpos - FRAME_LEN);
                rpos -= FRAME_LEN;
            }
            memcpy(rbuf + rpos, tmp, n);
            rpos += n;

            while (rpos >= FRAME_LEN) {
                if (rbuf[0] != FRAME_MAGIC) {
                    memmove(rbuf, rbuf + 1, --rpos);
                    continue;
                }
                int r = frame_check(rbuf);
                if (r == 0) {
                    recv_ok++;
                    printf("[RX] ✓ SEQ=%3d  内容=%c%c%c%c%c  OK  (共收 %d)\n",
                           rbuf[1], rbuf[2],rbuf[3],rbuf[4],rbuf[5],rbuf[6], recv_ok);
                } else {
                    recv_bad++;
                    printf("[RX] ✗ 帧校验失败 err=%d\n", r);
                }
                fflush(stdout);
                memmove(rbuf, rbuf + FRAME_LEN, rpos - FRAME_LEN);
                rpos -= FRAME_LEN;
            }
        } else {
            usleep(5000);
        }
    }

    close(fd);
    printf("\n[RX] 结束  成功=%d  校验失败=%d\n", recv_ok, recv_bad);
    fflush(stdout);
    exit(recv_ok > 255 ? 255 : recv_ok);
}

/* ── main ───────────────────────────────────────────── */
int main(int argc, char *argv[])
{
    if (argc < 3) {
        fprintf(stderr,
            "用法: %s <TX串口> <RX串口> [波特率] [noioctl]\n"
            "示例:\n"
            "  %s /dev/ttyS3 /dev/ttyS5 115200\n"
            "  %s /dev/ttyS5 /dev/ttyS3 115200\n"
            "  %s /dev/ttyS3 /dev/ttyS5 115200 noioctl  # 不启用RS485模式\n",
            argv[0], argv[0], argv[0], argv[0]);
        return 1;
    }

    const char *port_tx = argv[1];
    const char *port_rx = argv[2];
    int baud = (argc >= 4) ? atoi(argv[3]) : 115200;
    int rs485_mode = 1;  /* 默认启用 RS485 ioctl */
    if (argc >= 5 && strcmp(argv[4], "noioctl") == 0)
        rs485_mode = 0;

    printf("========================================\n");
    printf("  RK3588 RS485 串口互发测试  v2\n");
    printf("  TX: %s  RX: %s\n", port_tx, port_rx);
    printf("  波特率: %d  RS485-ioctl: %s\n", baud, rs485_mode ? "开" : "关");
    printf("========================================\n");
    printf("[接线] %s A ←→ %s A  |  B ←→ B  |  共地\n\n", port_tx, port_rx);
    fflush(stdout);

    pid_t pid = fork();
    if (pid < 0) { perror("fork"); return 1; }
    if (pid == 0) {
        rx_process(port_rx, baud, rs485_mode);
    }

    usleep(400000);  /* 等子进程就绪 */

    int fd = serial_open(port_tx, baud, rs485_mode);
    if (fd < 0) { kill(pid, SIGTERM); wait(NULL); return 1; }

    printf("[TX] 发送端就绪，开始发帧...\n\n");
    fflush(stdout);

    int sent = 0;
    for (int i = 0; i < TEST_FRAMES; i++) {
        uint8_t frame[FRAME_LEN];
        frame_build(frame, (uint8_t)i);

        int n = write(fd, frame, FRAME_LEN);
        tcdrain(fd);   /* 等物理发送完毕，RS485换向前必须等 */
        usleep(5000);  /* 额外 5ms 让接收端换向稳定 */

        if (n == FRAME_LEN) {
            sent++;
            printf("[TX] → SEQ=%3d 已发  (%d/%d)\n", i, sent, TEST_FRAMES);
        } else {
            printf("[TX] ✗ SEQ=%d 失败 n=%d %s\n", i, n, strerror(errno));
        }
        fflush(stdout);
        usleep((SEND_INTERVAL_MS - 5) * 1000);
    }

    close(fd);
    printf("\n[TX] 完毕，等待接收端汇总...\n");
    fflush(stdout);

    int status;
    waitpid(pid, &status, 0);
    int recv_ok = WIFEXITED(status) ? WEXITSTATUS(status) : -1;

    printf("\n========================================\n");
    printf("  测试结果\n");
    printf("  发送: %d  接收: %d\n", sent, recv_ok);
    if (sent > 0 && recv_ok >= 0) {
        float rate = 100.0f * recv_ok / sent;
        printf("  成功率: %.1f%%\n", rate);
        if      (rate >= 95.0f) printf("  结论: ✓ RS485 通信正常！\n");
        else if (rate >  0.0f)  printf("  结论: △ 有丢帧，检查接线/波特率\n");
        else {
            printf("  结论: ✗ 未收到数据\n");
            if (rs485_mode)
                printf("  建议: 尝试 noioctl 模式排查方向控制问题\n");
            else
                printf("  建议: 检查物理接线和串口设备名\n");
        }
    }
    printf("========================================\n");
    return (recv_ok > 0) ? 0 : 1;
}
