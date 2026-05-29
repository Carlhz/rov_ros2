#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cerrno>
#include <chrono>
#include <thread>

// 配置参数（根据你的设备修改）
const char* SERIAL_PORT = "/dev/ttyUSB0";  // 常见端口：/dev/ttyUSB0, /dev/ttyACM0
const int BAUDRATE = B9600;               // SF系列默认波特率
const uint8_t DEVICE_ID = 1;              // 你的测深仪机号

// 设置串口属性
bool setupSerial(int fd) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr() failed\n";
        return false;
    }

    // 设置波特率
    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);

    // 8N1: 8位数据，无校验，1停止位
    tty.c_cflag &= ~PARENB;  // 无校验
    tty.c_cflag &= ~CSTOPB;  // 1停止位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8位数据

    // 硬件流控关闭
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    // 接收设置
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    // 超时设置：VTIME=10 → 1秒超时
    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 17;  // 等待17字节

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr() failed\n";
        return false;
    }

    return true;
}

// 发送单次测距指令
bool sendMeasureCommand(int fd, uint8_t dev_id) {
    uint8_t cmd[6];
    cmd[0] = 0xAA;
    cmd[1] = 0xA0;
    cmd[2] = dev_id;
    cmd[3] = 0x00;
    cmd[4] = 0x00;
    // 计算校验
    cmd[5] = cmd[0] ^ cmd[1] ^ cmd[2] ^ cmd[3] ^ cmd[4];

    ssize_t written = write(fd, cmd, 6);
    if (written != 6) {
        std::cerr << "发送失败\n";
        return false;
    }

    std::cout << "→ 发送: ";
    for (int i = 0; i < 6; ++i) {
        printf("%02X ", cmd[i]);
    }
    std::cout << "\n";
    return true;
}

// 读取并解析测深数据
bool readDepthResponse(int fd) {
    uint8_t buffer[17];
    ssize_t n = read(fd, buffer, sizeof(buffer));

    if (n != 17) {
        std::cerr << "收到 " << n << " 字节，期望 17\n";
        return false;
    }

    std::cout << "← 收到: ";
    for (int i = 0; i < 17; ++i) {
        printf("%02X ", buffer[i]);
    }
    std::cout << "\n";

    // 校验帧头
    if (buffer[0] != 0xAB || buffer[1] != 0xA0) {
        std::cerr << "帧头错误\n";
        return false;
    }

    // 提取“最强目标距离”（字节9-10）
    uint16_t distance_unit = (static_cast<uint16_t>(buffer[8]) << 8) | buffer[9];
    int depth_cm = static_cast<int>(distance_unit);  // 1 unit = 1 cm

    if (depth_cm < 20) {
        std::cout << "最强目标距离 " << depth_cm << " cm 可能不可靠（<20cm）\n";
    } else {
        std::cout << "最强目标距离: " << depth_cm << " cm\n";
    }
    // 提取“最近目标距离”（字节5-6）
    uint16_t distance_first_unit = (static_cast<uint16_t>(buffer[4]) << 8) | buffer[5];
    int depth_first_cm = static_cast<int>(distance_first_unit);  // 1 unit = 1 cm

    if (depth_first_cm < 20) {
        std::cout << "最近目标距离 " << depth_first_cm << " cm 可能不可靠（<20cm）\n";
    } else {
        std::cout << "最近目标距离: " << depth_first_cm << " cm\n";
    }
    std::cout << "当前距离目标: " << depth_cm << " cm\n\n";
    return true;
}

int main() {
    std::cout << "正在打开串口 " << SERIAL_PORT << " ...\n";

    int fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "无法打开串口 " << SERIAL_PORT << ": " << strerror(errno) << "\n";
        std::cerr << "\n请检查：\n";
        std::cerr << "1. 设备是否插入？运行 'ls /dev/ttyUSB*' 查看\n";
        std::cerr << "2. 是否有权限？可尝试 'sudo chmod 666 " << SERIAL_PORT << "'\n";
        return 1;
    }

    if (!setupSerial(fd)) {
        close(fd);
        return 1;
    }

    uint8_t junk[64];
        int bytes;
        do {
        bytes = read(fd, junk, sizeof(junk));
            if (bytes > 0) {
                std::cout << "→ 清除 " << bytes << " 字节上电残留数据: ";
                for (int i = 0; i < bytes; ++i) printf("%02X ", junk[i]);
                std::cout << "\n";
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        } while (bytes > 0);
        std::cout << "开始测量（按 Ctrl+C 停止）\n\n";

    while (true) {
        if (!sendMeasureCommand(fd, DEVICE_ID)) {
            break;
        }
        if (!readDepthResponse(fd)) {
            // 可选：继续重试
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    close(fd);
    return 0;
}
