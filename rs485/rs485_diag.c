/*
 * rs485_diag.c - RS485 逐项诊断工具
 *
 * 逐个打开 ttyS3/ttyS5，检查：
 *   1. 能否打开
 *   2. RS485 ioctl 是否支持
 *   3. 能否发数据（用示波器/万用表可观察 TX 引脚）
 *   4. 自环测试（TX 接 RX）
 *
 * 编译: aarch64-linux-gnu-gcc -O2 -o rs485_diag rs485_diag.c
 *
 * 用法:
 *   ./rs485_diag                     # 诊断 ttyS3 和 ttyS5
 *   ./rs485_diag /dev/ttyS3          # 只诊断指定串口
 *   ./rs485_diag /dev/ttyS3 /dev/ttyS5  # 双串口测试
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#define BAUD 115200

static int serial_init(const char *dev)
{
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        printf("  [FAIL] open %s: %s\n", dev, strerror(errno));
        return -1;
    }
    printf("  [OK]   open %s  fd=%d\n", dev, fd);

    struct termios tty = {0};
    tcgetattr(fd, &tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag  = CS8 | CLOCAL | CREAD;
    tty.c_iflag  = 0;
    tty.c_oflag  = 0;
    tty.c_lflag  = 0;
    tcsetattr(fd, TCSANOW, &tty);

    return fd;
}

static int test_rs485_ioctl(int fd, const char *dev)
{
    struct serial_rs485 r = {0};
    if (ioctl(fd, TIOCGRS485, &r) < 0) {
        printf("  [INFO] TIOCGRS485 not supported on %s (%s)\n", dev, strerror(errno));
        return 0;
    }
    printf("  [OK]   TIOCGRS485 supported, flags=0x%x\n", r.flags);

    r.flags |= SER_RS485_ENABLED;
    r.flags |= SER_RS485_RTS_ON_SEND;
    r.flags &= ~SER_RS485_RTS_AFTER_SEND;
    r.delay_rts_before_send = 5;
    r.delay_rts_after_send  = 5;

    if (ioctl(fd, TIOCSRS485, &r) < 0) {
        printf("  [FAIL] TIOCSRS485: %s\n", strerror(errno));
        return 0;
    }
    printf("  [OK]   TIOCSRS485 set (RS485 mode ON)\n");

    /* 读回确认 */
    memset(&r, 0, sizeof(r));
    ioctl(fd, TIOCGRS485, &r);
    printf("  [INFO] readback flags=0x%x delay_b=%d delay_a=%d\n",
           r.flags, r.delay_rts_before_send, r.delay_rts_after_send);
    return 1;
}

static void test_send(int fd, const char *dev, int count)
{
    printf("  [INFO] sending %d bytes to %s...\n", count, dev);
    char msg[64];
    int n = snprintf(msg, sizeof(msg), "DIAG_TEST_%03d\n", count);
    int w = write(fd, msg, n);
    tcdrain(fd);
    printf("  [%s]  write returned %d (expected %d)\n",
           (w == n) ? "OK" : "FAIL", w, n);
}

/* 测试：串口 A 发，串口 B 收（需要外部接线 A-B 短接） */
static void test_cross(const char *dev_a, const char *dev_b)
{
    printf("\n--- cross test: %s -> %s ---\n", dev_a, dev_b);

    int fd_b = serial_init(dev_b);
    if (fd_b < 0) return;

    /* 先清空接收缓冲 */
    tcflush(fd_b, TCIFLUSH);

    int fd_a = serial_init(dev_a);
    if (fd_a < 0) { close(fd_b); return; }

    /* 启用 RS485 */
    test_rs485_ioctl(fd_a, dev_a);
    test_rs485_ioctl(fd_b, dev_b);

    /* 发送 */
    const char *msg = "CROSS_TEST_OK\n";
    printf("  [INFO] sending to %s...\n", dev_a);
    write(fd_a, msg, strlen(msg));
    tcdrain(fd_a);

    /* 等待接收 */
    usleep(200000); /* 200ms */
    char buf[256] = {0};
    int n = read(fd_b, buf, sizeof(buf) - 1);
    if (n > 0) {
        printf("  [OK]   %s received %d bytes: [%s]\n", dev_b, n, buf);
    } else {
        printf("  [FAIL] %s received 0 bytes\n", dev_b);

        /* 不用 RS485 ioctl 再试一次 */
        printf("  [INFO] retrying without RS485 ioctl...\n");
        struct serial_rs485 r = {0};
        ioctl(fd_a, TIOCGRS485, &r);
        r.flags &= ~SER_RS485_ENABLED;
        ioctl(fd_a, TIOCSRS485, &r);
        ioctl(fd_b, TIOCGRS485, &r);
        r.flags &= ~SER_RS485_ENABLED;
        ioctl(fd_b, TIOCSRS485, &r);

        tcflush(fd_b, TCIFLUSH);
        write(fd_a, msg, strlen(msg));
        tcdrain(fd_a);
        usleep(200000);
        memset(buf, 0, sizeof(buf));
        n = read(fd_b, buf, sizeof(buf) - 1);
        if (n > 0) {
            printf("  [OK]   no-ioctl: %s received %d bytes: [%s]\n", dev_b, n, buf);
        } else {
            printf("  [FAIL] no-ioctl: still 0 bytes received\n");
            printf("  [HINT] Check physical wiring: %s A<->%s A, B<->B, GND\n", dev_a, dev_b);
        }
    }

    close(fd_a);
    close(fd_b);
}

/* 自环测试（需要 TX 接 RX，对 RS485 可能不适用） */
static void test_loopback(const char *dev)
{
    printf("\n--- loopback test: %s (TX->RX) ---\n", dev);
    printf("  [HINT] need TX and RX physically connected\n");

    int fd = serial_init(dev);
    if (fd < 0) return;

    test_rs485_ioctl(fd, dev);
    tcflush(fd, TCIOFLUSH);

    const char *msg = "LOOPBACK_OK\n";
    write(fd, msg, strlen(msg));
    tcdrain(fd);
    usleep(100000);

    char buf[256] = {0};
    int n = read(fd, buf, sizeof(buf) - 1);
    if (n > 0)
        printf("  [OK]   loopback received: [%s]\n", buf);
    else
        printf("  [INFO] loopback received 0 (normal for RS485 half-duplex)\n");

    close(fd);
}

int main(int argc, char *argv[])
{
    const char *ports[] = {"/dev/ttyS3", "/dev/ttyS5", NULL};
    if (argc >= 2) {
        ports[0] = argv[1];
        ports[1] = (argc >= 3) ? argv[2] : NULL;
    }

    printf("========================================\n");
    printf("  RS485 Diagnostic Tool\n");
    printf("========================================\n");

    for (int i = 0; ports[i]; i++) {
        printf("\n--- diag: %s ---\n", ports[i]);
        int fd = serial_init(ports[i]);
        if (fd < 0) continue;
        test_rs485_ioctl(fd, ports[i]);
        test_send(fd, ports[i], 100);
        close(fd);
    }

    /* 如果有两个串口，做交叉测试 */
    if (ports[0] && ports[1]) {
        test_cross(ports[0], ports[1]);
        test_cross(ports[1], ports[0]);
    } else if (argc < 2) {
        /* 默认双串口测试 */
        test_cross("/dev/ttyS3", "/dev/ttyS5");
        test_cross("/dev/ttyS5", "/dev/ttyS3");
    }

    /* 自环测试（可选） */
    if (argc >= 2 && argc < 3) {
        test_loopback(argv[1]);
    }

    printf("\n========================================\n");
    printf("  Diag complete\n");
    printf("========================================\n");
    return 0;
}
