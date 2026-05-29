/*
 * rs485_tx_auto_de.c - RS485 测试 (TX 引脚自动控制 DE/RE 版本)
 *
 * 原理图设计: UART5_TX 通过 RS1G14XC5 缓冲器控制 CA-IS3082 的 DE/RE
 * 当 TX 发送数据时，信号经过 U16 自动切换 DE/RE 方向
 *
 * 使用方法:
 *   ./rs485_tx_auto_de /dev/ttyS5 115200
 *
 * 接线测试:
 *   终端1: ./rs485_tx_auto_de /dev/ttyS5 115200
 *   终端2: cat /dev/ttyS3  (或另一个RS485端口)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <time.h>

#define TEST_INTERVAL_MS 500

int open_serial(const char *device, int baud)
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    // 设置波特率
    speed_t speed;
    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            fprintf(stderr, "Unsupported baud rate: %d\n", baud);
            close(fd);
            return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1
    tty.c_cflag &= ~PARENB;  // 无校验
    tty.c_cflag &= ~CSTOPB;  // 1位停止位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8位数据

    // 禁用硬件流控
    tty.c_cflag &= ~CRTSCTS;

    // 启用接收，设置本地模式
    tty.c_cflag |= CREAD | CLOCAL;

    // 原始模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    // 读取设置
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    // 清空缓冲区
    tcflush(fd, TCIOFLUSH);

    return fd;
}

void send_break(int fd)
{
    // 发送 BREAK 信号，强制 TX 拉低一段时间
    // 这可能会触发 DE/RE 切换到发送模式
    tcsendbreak(fd, 0);
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <device> [baud]\n", argv[0]);
        fprintf(stderr, "Example: %s /dev/ttyS5 115200\n", argv[0]);
        return 1;
    }

    const char *device = argv[1];
    int baud = (argc >= 3) ? atoi(argv[2]) : 115200;

    printf("RS485 TX-Auto-DE Test\n");
    printf("=====================\n");
    printf("Device: %s\n", device);
    printf("Baud: %d\n", baud);
    printf("\n");

    int fd = open_serial(device, baud);
    if (fd < 0) {
        return 1;
    }

    printf("Serial opened successfully.\n");
    printf("\n");

    // 关键：先发一个 BREAK 或低电平信号，确保 DE/RE 被激活
    printf("Sending initial BREAK to activate DE/RE...\n");
    send_break(fd);
    usleep(10000);  // 10ms

    int count = 0;
    char msg[256];

    printf("Starting transmission loop...\n");
    printf("(Press Ctrl+C to stop)\n\n");

    while (1) {
        snprintf(msg, sizeof(msg), "[TX%04d] RS485_TEST_%.6d_%.6d_%.6d_END\n",
                 count, count*3, count*7, count*11);

        // 先发一个 0x00 字节，确保 TX 引脚有电平变化
        char zero = 0x00;
        write(fd, &zero, 1);
        tcdrain(fd);  // 等待发送完成
        usleep(1000); // 1ms 延时

        // 发送实际数据
        int len = strlen(msg);
        int n = write(fd, msg, len);
        tcdrain(fd);  // 等待发送完成

        if (n == len) {
            printf("Sent[%04d]: %s", count, msg);
        } else {
            printf("Send failed: %d/%d bytes\n", n, len);
        }

        count++;
        usleep(TEST_INTERVAL_MS * 1000);
    }

    close(fd);
    return 0;
}
