#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <thread>
#include <atomic>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstring>
#include <cctype>
#include <iomanip>


std::atomic<int> g_serial_fd(-1);

// 发送命令到 DVL（自动加 \r）
void send_dvl_command_raw(const std::string& full_16byte_cmd) {
    if (g_serial_fd >= 0 && full_16byte_cmd.length() == 16) {
        write(g_serial_fd, full_16byte_cmd.c_str(), 16);
        std::cout << "[发送] " << full_16byte_cmd.substr(0, 13) << "...\n"; // 隐藏 r0000
    }
}

void send_dvl_command(const std::string& cmd) {
    if (g_serial_fd >= 0) {
        std::string msg = cmd + "\r";
        write(g_serial_fd, msg.c_str(), msg.size());
        std::cout << "\n 已发送命令: \"" << cmd << "\"\n";
        usleep(50000);
    }
}

std::string format_h1000_command(const std::string& short_cmd) {
    // 示例输入: "PC 1400.00"
    size_t pos = short_cmd.find(' ');
    if (pos == std::string::npos) return ""; // 无空格，无效

    std::string cmd_head = short_cmd.substr(0, pos);
    std::string param = short_cmd.substr(pos + 1);

    // 检查命令头长度（必须2字符）
    if (cmd_head.length() != 2) return "";

    // 构造: [CMD][ ][param][\r][r0000]
    std::string formatted = cmd_head + " " + param + "\r" + "r0000";

    // 截断或补空格，确保总长16
    if (formatted.length() > 16) {
        formatted = formatted.substr(0, 16);
    } else if (formatted.length() < 16) {
        formatted.resize(16, ' '); // 右侧补空格（但通常不会）
    }

    return formatted;
}

// 全局变量：用于接收线程通知
std::atomic<bool> g_wait_for_ack(false);
std::atomic<bool> g_ack_received(false);
std::string g_expected_ack_tag;

void send_config_command(const std::string& short_cmd) {
    std::string full_cmd = format_h1000_command(short_cmd);
    if (full_cmd.empty()) {
        std::cout << "命令格式错误: \"" << short_cmd << "\"\n";
        return;
    }

    // 提取命令头用于确认（如 "PC" → 期待 ":PC"）
    std::string cmd_head = short_cmd.substr(0, 2);
    g_expected_ack_tag = ":" + cmd_head;
    g_ack_received = false;
    g_wait_for_ack = true;

    std::cout << "正在应用配置: " << short_cmd << " ...\n";

    // 执行 CZ → CMD → CS
    send_dvl_command_raw("CZ         r0000"); usleep(100000);
    send_dvl_command_raw(full_cmd);           usleep(100000);
    send_dvl_command_raw("CS         r0000"); usleep(100000);

    // 等待 DVL 回复确认（最多2秒）
    int timeout_ms = 2000;
    while (timeout_ms > 0 && !g_ack_received) {
        usleep(10000); // 10ms
        timeout_ms -= 10;
    }

    if (g_ack_received) {
        std::cout << "配置成功: " << short_cmd << "\n";
    } else {
        std::cout << "超时未收到确认，可能未生效\n";
    }

    g_wait_for_ack = false;
}

// 判断一个字符串是否是有效的数值（整数或浮点），允许前导/后缀空格
// 要求整个 token 都是数值（不允许 "123A" 这种）
double safe_stod(const std::string& s, bool& ok) {
    ok = false;
    if (s.empty()) return 0.0;

    // 去除前后空格
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return 0.0; // 全是空格
    size_t end = s.find_last_not_of(" \t\r\n");
    std::string clean = s.substr(start, end - start + 1);

    if (clean.empty()) return 0.0;

    // 手动验证：是否为合法数值格式？
    size_t i = 0;
    // 允许前导正负号
    if (clean[i] == '+' || clean[i] == '-') i++;

    bool has_digit = false;
    bool has_dot = false;

    while (i < clean.size()) {
        char c = clean[i];
        if (std::isdigit(c)) {
            has_digit = true;
            i++;
        } else if (c == '.' && !has_dot) {
            has_dot = true;
            i++;
        } else {
            // 遇到非法字符 → 整个不是纯数值
            return 0.0;
        }
    }

    // 至少有一个数字才算有效
    if (!has_digit) return 0.0;

    // 现在可以安全转换
    try {
        size_t pos;
        double val = std::stod(clean, &pos);
        // 因为已手动验证，pos 应等于 clean.size()
        if (pos == clean.size()) {
            ok = true;
            return val;
        }
    } catch (...) {
        // 转换失败（如溢出）
    }

    return 0.0;
}


//添加 trim 函数
std::string trim(const std::string& str) {
    if (str.empty()) return str;
    size_t start = str.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = str.find_last_not_of(" \t\r\n");
    return str.substr(start, end - start + 1);
}
// 解析并输出中文
void parse_and_translate(const std::string& line) {
	// 检测是否是配置确认包（如 :PC 1400.00:）
	if (g_wait_for_ack && line.length() > 3 && line[0] == ':' && line.back() == ':') {
		if (line.substr(0, g_expected_ack_tag.length()) == g_expected_ack_tag) {
			g_ack_received = true;
			// 不打印此行（避免干扰）
			return;
		}
	}

    if (line.empty() || line[0] != ':') return;

    size_t comma = line.find(',');
    if (comma == std::string::npos) return;

    std::string tag = line.substr(0, comma);
    std::string data_part = line.substr(comma + 1);

    std::vector<double> nums;
    std::vector<std::string> tokens;

    std::stringstream ss(data_part);
    std::string token;
    while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
        bool ok;
        double val = safe_stod(token, ok);
        if (ok) {
            nums.push_back(val);
        } else {
            nums.push_back(0.0);
        }
    }

    if (tag == ":SA") {
        if (nums.size() >= 3) {
            std::cout << "\n   姿态:\n"
                      << "     纵摇: " << nums[0] << "°\n"
                      << "     横摇: " << nums[1] << "°\n"
                      << "     艏向: " << nums[2] << "°\n";
        }
    }
    else if (tag == ":TS") {
        if (tokens.size() >= 6) {
            long long ts_int = static_cast<long long>(nums[0]);
            char ts_buf[20];
            snprintf(ts_buf, sizeof(ts_buf), "%014lld", ts_int);

            std::string status_code = trim(tokens[5]);

            std::cout << "\n   时间与环境:\n"
                      << "     时间戳: " << ts_buf << " (YYMMDDHHMMSSss)\n"
                      << "     盐度: " << nums[1] << " ppt\n"
                      << "     水温: " << nums[2] << " °C\n"
                      << "     入水深度: " << nums[3] << " m\n"
                      << "     声速: " << static_cast<int>(nums[4]) << " m/s\n"
                      << "     系统状态/协议版本: \"" << status_code << "\"\n";
        }
    }
    else if (tag == ":BI" || tag == ":BS" || tag == ":BE") {
        if (nums.size() >= 4 && tokens.size() >= 4) {
            std::string status_str = trim(tokens.back()); // 获取最后一个字段
            char status = (status_str == "A") ? 'A' : 'V'; // 确保是 A/V

            std::cout << "\n   底跟踪-" << (tag == ":BI" ? "设备坐标系" :
                                        tag == ":BS" ? "船体坐标系" : "大地坐标系") << "下速度数据:\n";

            if (tag == ":BI") {
                std::cout << "     X轴速度-前向为正: " << nums[0] << " mm/s\n"
                          << "     Y轴速度-右向为正: " << nums[1] << " mm/s\n"
                          << "     Z轴速度-向下为正: " << nums[2] << " mm/s\n"
                          << "     速度误差: " << nums[3] << " mm/s\n";
            }
            else if (tag == ":BS") {
                std::cout << "     X轴速度-船头为正: " << nums[0] << " mm/s\n"
                          << "     Y轴速度-右舷为正: " << nums[1] << " mm/s\n"
                          << "     Z轴速度-向下为正: " << nums[2] << " mm/s\n";
            }
            else if (tag == ":BE") {
                std::cout << "     东向速度: " << nums[0] << " mm/s\n"
                          << "     北向速度: " << nums[1] << " mm/s\n"
                          << "     向上速度: " << nums[2] << " mm/s\n";
            }

            std::cout << "     状态: " << status << " (A=有效, V=无效)\n";
        }
    }
    else if (tag == ":BD") {
        if (nums.size() >= 5) {
            std::cout << "\n   底跟踪-大地坐标系下距离数据:\n"
                      << "     东向距离: " << nums[0] << " m\n"
                      << "     北向距离: " << nums[1] << " m\n"
                      << "     向上距离: " << nums[2] << " m\n"
                      << "     设备离底距离: " << nums[3] << " m\n"
                      << "     距离上一呯速度有效估计的时间: " << nums[4] << " s\n";
        }
    }
    else {
        std::cout << " 未知标签: " << tag << "\n";
    }

    std::cout << "----------------------------------------\n";
}

// 键盘监听线程
void keyboard_listener() {
    struct termios old_term, new_term;
    tcgetattr(STDIN_FILENO, &old_term);
    new_term = old_term;
    new_term.c_lflag &= ~(ICANON | ECHO); // 非规范模式，无回显
    tcsetattr(STDIN_FILENO, TCSANOW, &new_term);

    char c;
    while (true) {
        if (read(STDIN_FILENO, &c, 1) > 0) {
            if (c == 'q' || c == 'Q') {
                std::cout << "\n正在退出...\n";
                break;
            }
            else if (c == '`' || c == '~') { // 用反引号进入命令模式
                // 恢复终端设置，以便 getline 能用
                tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
                std::cout << "[DVL命令  |示例           |范围 ]:\n";
                std::cout << "呯率      |PR 10          |1-20\n";
				std::cout << "平均次数  |PM 10          |1-20\n";
				std::cout << "量程      |BX 68.00       |0.5-80m\n";
				std::cout << "盲区      |WF 0.51        |0.8-70m\n";
				std::cout << "数据格式  |DF 1           |0-PD6/1-PLT/3-EPD6\n";
				std::cout << "测速范围  |PV 10.00       |10m/s\n";
				std::cout << "声速模式  |PF 1           |“1”自定义，“0”自动计算\n";
				std::cout << "声速值    |PC 1500.00     |1400-1600\n";
				std::cout << "盐度值    |PS 35.00       |10-50 PPT\n";
				std::cout << "时间同步  |DF ST+YYMMDDHHmmSShh \n";
				
                std::string cmd;
                std::getline(std::cin, cmd); // 用户输入完整命令

                if (!cmd.empty()) {
                    send_dvl_command(cmd);
                }

                // 重新进入非规范模式
                tcsetattr(STDIN_FILENO, TCSANOW, &new_term);
            }
            else if (c == 's' || c == 'S') {
                send_dvl_command("CS");
            }
            else if (c == 'z' || c == 'Z') {
                send_dvl_command("CZ");
            }
            // 其他快捷键...
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
    if (g_serial_fd >= 0) close(g_serial_fd);
    exit(0);
}


int main() {
    const char* PORT = "/dev/ttyACM0";
    const int BAUD = B115200;

    int fd = open(PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "无法打开串口: " << PORT << "\n";
        return 1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "获取串口属性失败\n";
        close(fd);
        return 1;
    }

    cfmakeraw(&tty);
    cfsetspeed(&tty, BAUD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "设置串口失败\n";
        close(fd);
        return 1;
    }

    g_serial_fd = fd;

    //  初始化：切换到 PD6 + 开始测量
    std::cout << "正在初始化 H1000...\n";
    sleep(1); // 等待设备稳定
    send_dvl_command("DF 0"); // 切换为 PD6 格式（根据手册）
    usleep(100000);           // 等 100ms
    send_dvl_command("CS");   // 开始测量

    std::cout << "\n H1000 控制器启动成功！\n";
    std::cout << "   按 's' → 发送 CS（开始测量）\n";
    std::cout << "   按 'z' → 发送 CZ（停止测量）\n";
    std::cout << "   按 'q' → 退出程序\n";
    std::cout << "========================================\n";

    // 启动键盘监听
    std::thread key_thread(keyboard_listener);

    // 主循环：读取串口
    std::string line;
    char ch;
    while (read(fd, &ch, 1) > 0) {
        if (ch == '\n') {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (!line.empty()) {
                std::cout << "数据: " << line << "\n";
                parse_and_translate(line);
            }
            line.clear();
        } else {
            line += ch;
        }
    }

    key_thread.join();
    close(fd);
    return 0;
}



