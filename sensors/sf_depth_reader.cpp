#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cerrno>
#include <chrono>
#include <thread>

const char* SERIAL_PORT = "/dev/ttyUSB0";
const int BAUDRATE = B9600;
const uint8_t DEVICE_ID = 1;

bool setupSerial(int fd) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr() failed\n";
        return false;
    }

    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    // 关键：设为非阻塞模式（由应用层控制超时）
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr() failed\n";
        return false;
    }

    // 清空可能存在的旧数据
    tcflush(fd, TCIOFLUSH);
    return true;
}

bool sendMeasureCommand(int fd, uint8_t dev_id) {
    uint8_t cmd[6] = {0xAA, 0xA0, dev_id, 0x00, 0x00};
    cmd[5] = cmd[0] ^ cmd[1] ^ cmd[2] ^ cmd[3] ^ cmd[4];
    if (write(fd, cmd, 6) != 6) {
        std::cerr << "发送失败\n";
        return false;
    }

    std::cout << "→ 发送: ";
    for (int i = 0; i < 6; ++i) printf("%02X ", cmd[i]);
    std::cout << "\n";
    return true;
}

// 从串口流中同步并读取一个完整 SF 包
bool readValidPacket(int fd, uint8_t* buffer) {
    uint8_t byte;
    int state = 0; // 0=找AB, 1=找A0, 2=收剩余15字节
    int idx = 0;
    auto start = std::chrono::steady_clock::now();

    while (true) {
        if (read(fd, &byte, 1) == 1) {
            switch (state) {
                case 0:
                    if (byte == 0xAB) state = 1;
                    break;
                case 1:
                    if (byte == 0xA0) {
                        buffer[0] = 0xAB;
                        buffer[1] = 0xA0;
                        idx = 2;
                        state = 2;
                    } else {
                        state = (byte == 0xAB) ? 1 : 0;
                    }
                    break;
                case 2:
                    buffer[idx++] = byte;
                    if (idx == 17) return true;
                    break;
            }
        } else {
            // 超时判断
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
            if (elapsed.count() > 2000) { // 2秒超时
                std::cerr << "读取超时\n";
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
}

int main() {
    std::cout << "正在打开串口 " << SERIAL_PORT << " ...\n";

    int fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "无法打开串口 " << SERIAL_PORT << ": " << strerror(errno) << "\n";
        return 1;
    }

    if (!setupSerial(fd)) {
        close(fd);
        return 1;
    }

    std::cout << "等待设备稳定...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2)); // 给设备上电时间

    std::cout << "开始测量（按 Ctrl+C 停止）\n\n";

    while (true) {
        sendMeasureCommand(fd, DEVICE_ID);

        uint8_t buffer[17];
        if (readValidPacket(fd, buffer)) {
            std::cout << "← 收到: ";
            for (int i = 0; i < 17; ++i) printf("%02X ", buffer[i]);
            std::cout << "\n";

            // 解析最强目标距离（字节8-9）
            uint16_t depth_cm = (static_cast<uint16_t>(buffer[8]) << 8) | buffer[9];
            std::cout << "水深: " << static_cast<int>(depth_cm) << " cm\n\n";
        } else {
            std::cout << "未收到有效数据包\n\n";
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    close(fd);
    return 0;
}
