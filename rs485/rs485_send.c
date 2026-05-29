/*
 * rs485_send.c - RS485 单串口持续发送测试
 *
 * 用法:
 *   aarch64-linux-gnu-gcc -O2 -o rs485_send rs485_send.c
 *
 *   ./rs485_send /dev/ttyS3 115200     # ttyS3 发送
 *   ./rs485_send /dev/ttyS5 9600        # ttyS5 低波特率发送
 *   ./rs485_send /dev/ttyS3 115200 raw  # 不启 RS485 ioctl
 *
 * 监听（在开发板另一个终端）:
 *   cat /dev/ttyS5
 *
 * 停止: Ctrl+C
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

static volatile int g_running = 1;
static void sig_handler(int sig) { (void)sig; g_running = 0; }

int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr,
            "用法: %s <串口> [波特率] [raw]\n"
            "示例: %s /dev/ttyS3 115200\n"
            "      %s /dev/ttyS5 9600 raw\n"
            "\n监听: cat /dev/ttyS5\n",
            argv[0], argv[0], argv[0]);
        return 1;
    }

    const char *dev = argv[1];
    int baud = (argc >= 3) ? atoi(argv[2]) : 115200;
    int rs485_mode = 1;
    for (int i = 2; i < argc; i++)
        if (strcmp(argv[i], "raw") == 0) rs485_mode = 0;

    /* 打开串口 */
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "[ERROR] 打开 %s 失败: %s\n", dev, strerror(errno));
        return 1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(fd, &tty);

    speed_t spd = B115200;
    switch (baud) {
        case 4800:   spd = B4800;   break;
        case 9600:   spd = B9600;   break;
        case 19200:  spd = B19200;  break;
        case 38400:  spd = B38400;  break;
        case 57600:  spd = B57600;  break;
        case 115200: spd = B115200; break;
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

    /* RS485 ioctl */
    if (rs485_mode) {
        struct serial_rs485 r = {0};
        if (ioctl(fd, TIOCGRS485, &r) >= 0) {
            r.flags |= SER_RS485_ENABLED;
            r.flags |= SER_RS485_RTS_ON_SEND;
            r.flags &= ~SER_RS485_RTS_AFTER_SEND;
            r.delay_rts_before_send = 0;
            r.delay_rts_after_send  = 0;
            if (ioctl(fd, TIOCSRS485, &r) >= 0)
                printf("[INFO] RS485 ioctl 已启用\n");
            else
                printf("[WARN] RS485 ioctl 设置失败: %s\n", strerror(errno));
        }
    }

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    printf("[TX] 发送端启动: %s @ %d baud  (Ctrl+C 停止)\n", dev, baud);

    char msg[128];
    int seq = 0;
    while (g_running) {
        int n = snprintf(msg, sizeof(msg),
            "[%03d] RS485_TEST - Hello from %s - 1234567890\n", seq++, dev);
        write(fd, msg, n);
        tcdrain(fd);
        printf("[TX] %s", msg);
        fflush(stdout);
        usleep(500000);  /* 500ms */
    }

    close(fd);
    printf("\n[TX] 已停止，共发送 %d 帧\n", seq);
    return 0;
}
