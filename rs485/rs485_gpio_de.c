/*
 * rs485_gpio_de.c - RS485 测试 (GPIO 手动控制 DE 版本)
 *
 * 原理图分析:
 *   - UART5_TX (AH26) 通过 U16(RS1G14XC5) 控制 U14(CA-IS3082) 的 DE/RE
 *   - 但 RS485 发送失败，说明 DE 可能没有被正确拉高
 *
 * 本程序使用 GPIO 直接控制 DE 引脚，绕过自动切换逻辑
 * 需要先确认 GPIO 编号（根据原理图和 RK3588 引脚定义）
 *
 * 使用方法:
 *   ./rs485_gpio_de /dev/ttyS5 115200 <gpio_num>
 *
 * 例如:
 *   ./rs485_gpio_de /dev/ttyS5 115200 136  (GPIO4_B0, 假设 DE 连这里)
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

// GPIO 操作函数
int gpio_export(int gpio)
{
    char path[64];
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) return -1;
    dprintf(fd, "%d", gpio);
    close(fd);
    usleep(100000);  // 100ms 等待 sysfs 创建
    return 0;
}

int gpio_unexport(int gpio)
{
    int fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd < 0) return -1;
    dprintf(fd, "%d", gpio);
    close(fd);
    return 0;
}

int gpio_set_direction(int gpio, const char *dir)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    write(fd, dir, strlen(dir));
    close(fd);
    return 0;
}

int gpio_set_value(int gpio, int value)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    dprintf(fd, "%d", value);
    close(fd);
    return 0;
}

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

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

int main(int argc, char *argv[])
{
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <device> <gpio_num> [baud]\n", argv[0]);
        fprintf(stderr, "Example: %s /dev/ttyS5 136 115200\n", argv[0]);
        fprintf(stderr, "\nGPIO number examples for RK3588:\n");
        fprintf(stderr, "  GPIO1_A0 = 32, GPIO1_A7 = 39\n");
        fprintf(stderr, "  GPIO3_B0 = 88, GPIO3_D0 = 120\n");
        fprintf(stderr, "  GPIO4_A0 = 128, GPIO4_B0 = 136\n");
        fprintf(stderr, "  GPIO4_C0 = 144, GPIO4_D0 = 152\n");
        return 1;
    }

    const char *device = argv[1];
    int gpio_num = atoi(argv[2]);
    int baud = (argc >= 4) ? atoi(argv[3]) : 115200;

    printf("RS485 GPIO-DE Test\n");
    printf("==================\n");
    printf("Device: %s\n", device);
    printf("GPIO DE: %d\n", gpio_num);
    printf("Baud: %d\n", baud);
    printf("\n");

    // 初始化 GPIO
    printf("Exporting GPIO %d...\n", gpio_num);
    gpio_unexport(gpio_num);  // 先取消导出，避免重复
    usleep(50000);
    if (gpio_export(gpio_num) < 0) {
        fprintf(stderr, "Failed to export GPIO %d\n", gpio_num);
        return 1;
    }

    printf("Setting GPIO %d as output...\n", gpio_num);
    if (gpio_set_direction(gpio_num, "out") < 0) {
        fprintf(stderr, "Failed to set GPIO %d direction\n", gpio_num);
        gpio_unexport(gpio_num);
        return 1;
    }

    // 初始状态：拉低 DE（接收模式）
    gpio_set_value(gpio_num, 0);
    printf("GPIO %d initialized to LOW (receive mode)\n", gpio_num);

    int fd = open_serial(device, baud);
    if (fd < 0) {
        gpio_unexport(gpio_num);
        return 1;
    }

    printf("Serial opened successfully.\n");
    printf("\n");

    int count = 0;
    char msg[256];

    printf("Starting transmission loop...\n");
    printf("GPIO %d will be HIGH during transmission, LOW after.\n", gpio_num);
    printf("(Press Ctrl+C to stop)\n\n");

    while (1) {
        snprintf(msg, sizeof(msg), "[TX%04d] RS485_TEST_%.6d_%.6d_%.6d_END\n",
                 count, count*3, count*7, count*11);

        int len = strlen(msg);

        // 拉高 DE，切换到发送模式
        gpio_set_value(gpio_num, 1);
        usleep(1000);  // 1ms 延时，确保 DE 稳定

        // 发送数据
        int n = write(fd, msg, len);
        tcdrain(fd);  // 等待发送完成

        // 发送完成后拉低 DE，切回接收模式
        usleep(2000);  // 2ms 延时，确保最后字节发送完成
        gpio_set_value(gpio_num, 0);

        if (n == len) {
            printf("Sent[%04d]: %s", count, msg);
        } else {
            printf("Send failed: %d/%d bytes\n", n, len);
        }

        count++;
        usleep(TEST_INTERVAL_MS * 1000);
    }

    close(fd);
    gpio_set_value(gpio_num, 0);
    gpio_unexport(gpio_num);
    return 0;
}
