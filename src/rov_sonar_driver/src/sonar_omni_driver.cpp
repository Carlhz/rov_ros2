/**
 * @file sonar_omni_driver.cpp
 * @brief ROS2 全向扫描声纳驱动节点 (Scanfish-II Omni)
 *
 * 基于原 ROS1 sensor_sonar_scanfish2 移植到 ROS2 C++。
 * 通过 UDP 与声纳通信，发布 PointCloud2 点云数据。
 *
 * 协议: FE/FD 帧格式，28字节命令帧，可变长响应帧
 * 运行平台: RK3588 (ARM64 Linux, ROS2 Humble)
 * 声纳 IP: 192.168.0.5 (可配置)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rov_sonar_interface/srv/sonar_config.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

// ========== 常量定义 ==========
static constexpr size_t CMD_SIZE       = 28;    // 命令帧长度
static constexpr size_t RECV_BUF_SIZE  = 1500;  // 接收缓冲区 (<MTU)
static constexpr size_t PKG_BUF_SIZE   = 1050;  // 完整数据包缓存
static constexpr size_t LEFTOVER_SIZE  = 3000;  // 分包遗留缓存

// ========== 帧标记 ==========
static constexpr uint8_t FRAME_HEAD = 0xFE;
static constexpr uint8_t FRAME_TAIL = 0xFD;

// ========== 工作状态 ==========
static constexpr uint8_t STATUS_RUN  = 0x23;  // 主模式运行+扇扫+发送+发射
static constexpr uint8_t STATUS_STOP = 0x2B;  // 停止

using PointCloud2 = sensor_msgs::msg::PointCloud2;

class SonarOmniDriver : public rclcpp::Node
{
public:
    SonarOmniDriver()
    : Node("sonar_omni_driver")
    {
        // ---- 声明参数 ----
        this->declare_parameter("server_ip",          "192.168.0.5");
        this->declare_parameter("server_port",        23);
        this->declare_parameter("frame_id",           "sonar_omni_link");
        this->declare_parameter("range",              4);
        this->declare_parameter("start_gain",         20);
        this->declare_parameter("logf",               40);
        this->declare_parameter("absorption",         10);
        this->declare_parameter("sound_speed",        1485);
        this->declare_parameter("train_angle",        0);
        this->declare_parameter("sector_width",       3600);
        this->declare_parameter("data_len",           1000);
        this->declare_parameter("pulse_type",         0);
        this->declare_parameter("gate",               200);
        this->declare_parameter("min_range",          150);
        this->declare_parameter("delay_us",           0);
        this->declare_parameter("frequency",          0);
        this->declare_parameter("rigidity_threshold", 5);
        this->declare_parameter("cmd_interval_ms",    5000);
        this->declare_parameter("read_rate_hz",       200.0);

        load_params();

        // ---- 发布者 ----
        pub_original_ = this->create_publisher<PointCloud2>("sonar/omni/original", 10);
        pub_rigidity_ = this->create_publisher<PointCloud2>("sonar/omni/rigidity", 10);
        pub_boundary_ = this->create_publisher<PointCloud2>("sonar/omni/boundary", 10);

        // ---- 服务 ----
        using SonarConfig = rov_sonar_interface::srv::SonarConfig;
        config_srv_ = this->create_service<SonarConfig>(
            "sonar/omni/config",
            std::bind(&SonarOmniDriver::config_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ---- 连接声纳 ----
        udp_connect();
        update_command();

        // ---- 定时器 ----
        // 高速读取定时器 (处理声纳回波数据)
        int read_period_ms = static_cast<int>(1000.0 / read_rate_hz_);
        if (read_period_ms < 1) read_period_ms = 1;
        read_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(read_period_ms),
            std::bind(&SonarOmniDriver::read_callback, this));

        // 命令发送定时器 (维持声纳扫描)
        cmd_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(cmd_interval_ms_),
            std::bind(&SonarOmniDriver::cmd_callback, this));

        RCLCPP_INFO(this->get_logger(),
            "全向声纳驱动已启动 -> %s:%d, sector=%d (PPI模式)",
            server_ip_.c_str(), server_port_, sector_width_);
    }

    ~SonarOmniDriver() override
    {
        RCLCPP_INFO(this->get_logger(), "发送停止指令...");
        work_status_ = STATUS_STOP;
        update_command();
        write(socket_fd_, cmd_buf_, CMD_SIZE);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        close(socket_fd_);
        RCLCPP_INFO(this->get_logger(), "声纳驱动已安全退出");
    }

private:
    // ==================== 参数加载 ====================
    void load_params()
    {
        server_ip_          = this->get_parameter("server_ip").as_string();
        server_port_        = static_cast<uint16_t>(this->get_parameter("server_port").as_int());
        frame_id_           = this->get_parameter("frame_id").as_string();
        range_              = static_cast<uint8_t>(this->get_parameter("range").as_int());
        start_gain_         = static_cast<uint8_t>(this->get_parameter("start_gain").as_int());
        logf_               = static_cast<uint8_t>(this->get_parameter("logf").as_int());
        absorption_         = static_cast<uint8_t>(this->get_parameter("absorption").as_int());
        sound_speed_        = this->get_parameter("sound_speed").as_int();
        train_angle_        = this->get_parameter("train_angle").as_int();
        sector_width_       = this->get_parameter("sector_width").as_int();
        data_len_           = this->get_parameter("data_len").as_int();
        pulse_type_         = static_cast<uint8_t>(this->get_parameter("pulse_type").as_int());
        gate_               = this->get_parameter("gate").as_int();
        min_range_          = this->get_parameter("min_range").as_int();
        delay_us_           = this->get_parameter("delay_us").as_int();
        frequency_          = static_cast<uint8_t>(this->get_parameter("frequency").as_int());
        rigidity_threshold_ = this->get_parameter("rigidity_threshold").as_int();
        cmd_interval_ms_    = this->get_parameter("cmd_interval_ms").as_int();
        read_rate_hz_       = this->get_parameter("read_rate_hz").as_double();

        work_status_ = STATUS_RUN;
        buf_left_len_ = 0;
    }

    // ==================== 命令构建 ====================
    void update_command()
    {
        memset(cmd_buf_, 0, CMD_SIZE);
        cmd_buf_[0]  = FRAME_HEAD;
        cmd_buf_[1]  = 0x00;                          // Broadcast
        cmd_buf_[2]  = work_status_;
        cmd_buf_[3]  = range_;
        cmd_buf_[4]  = start_gain_;
        cmd_buf_[5]  = logf_;
        cmd_buf_[6]  = absorption_;
        cmd_buf_[7]  = 0x01;                          // StepSize (仅支持1)
        cmd_buf_[8]  = static_cast<uint8_t>((sound_speed_ >> 8) & 0xFF);
        cmd_buf_[9]  = static_cast<uint8_t>(sound_speed_ & 0xFF);
        cmd_buf_[10] = static_cast<uint8_t>((train_angle_ >> 8) & 0xFF);
        cmd_buf_[11] = static_cast<uint8_t>(train_angle_ & 0xFF);
        cmd_buf_[12] = static_cast<uint8_t>((sector_width_ >> 8) & 0xFF);
        cmd_buf_[13] = static_cast<uint8_t>(sector_width_ & 0xFF);
        cmd_buf_[14] = static_cast<uint8_t>((data_len_ >> 8) & 0xFF);
        cmd_buf_[15] = static_cast<uint8_t>(data_len_ & 0xFF);
        cmd_buf_[16] = pulse_type_;
        cmd_buf_[17] = 0x00;                          // Res1
        cmd_buf_[18] = static_cast<uint8_t>((gate_ >> 8) & 0xFF);
        cmd_buf_[19] = static_cast<uint8_t>(gate_ & 0xFF);
        cmd_buf_[20] = static_cast<uint8_t>((min_range_ >> 8) & 0xFF);
        cmd_buf_[21] = static_cast<uint8_t>(min_range_ & 0xFF);
        cmd_buf_[22] = static_cast<uint8_t>((delay_us_ >> 8) & 0xFF);
        cmd_buf_[23] = static_cast<uint8_t>(delay_us_ & 0xFF);
        cmd_buf_[24] = 0x00;                          // Res2
        cmd_buf_[25] = 0x00;                          // Res3
        cmd_buf_[26] = frequency_;
        cmd_buf_[27] = FRAME_TAIL;
    }

    // ==================== 配置服务 ====================
    void config_callback(
        const std::shared_ptr<rov_sonar_interface::srv::SonarConfig::Request>  req,
        std::shared_ptr<rov_sonar_interface::srv::SonarConfig::Response>       res)
    {
        work_status_ = req->on_off ? STATUS_RUN : STATUS_STOP;

        if (req->range > 0)              range_              = static_cast<uint8_t>(req->range);
        if (req->start_gain > 0)         start_gain_         = static_cast<uint8_t>(req->start_gain);
        if (req->logf > 0)               logf_               = static_cast<uint8_t>(req->logf);
        if (req->absorption > 0)         absorption_         = static_cast<uint8_t>(req->absorption);
        if (req->sound_speed > 0)       sound_speed_         = req->sound_speed;
        if (req->train_angle >= 0)       train_angle_        = req->train_angle;
        if (req->sector_width >= 0)      sector_width_       = req->sector_width;
        if (req->data_len > 0)           data_len_           = req->data_len;
        if (req->pulse_type >= 0)        pulse_type_         = static_cast<uint8_t>(req->pulse_type);
        if (req->gate > 0)               gate_               = req->gate;
        if (req->min_range >= 0)         min_range_          = req->min_range;
        if (req->delay_us >= 0)          delay_us_           = req->delay_us;
        if (req->frequency >= 0)         frequency_          = static_cast<uint8_t>(req->frequency);
        if (req->rigidity_threshold >= 0) rigidity_threshold_ = req->rigidity_threshold;

        update_command();
        res->success = true;
        res->message = req->on_off ? "Sonar scanning started" : "Sonar stopped";

        RCLCPP_INFO(this->get_logger(),
            "配置更新: on=%d, range=%d, gain=%d, gate=%d, sector=%d",
            req->on_off, range_, start_gain_, gate_, sector_width_);
    }

    // ==================== UDP 连接 ====================
    void udp_connect()
    {
        socket_fd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "创建 UDP socket 失败: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }

        memset(&addr_, 0, sizeof(addr_));
        addr_.sin_family = AF_INET;
        addr_.sin_port   = htons(server_port_);
        addr_.sin_addr.s_addr = inet_addr(server_ip_.c_str());

        // 持续尝试连接
        while (rclcpp::ok()) {
            int ret = connect(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr_), sizeof(addr_));
            if (ret == 0) {
                RCLCPP_INFO(this->get_logger(), "UDP 已连接 -> %s:%d", server_ip_.c_str(), server_port_);
                break;
            }
            RCLCPP_WARN(this->get_logger(),
                "UDP 连接 %s:%d 失败，5秒后重试...", server_ip_.c_str(), server_port_);
            rclcpp::sleep_for(std::chrono::seconds(5));
        }
    }

    void reconnect()
    {
        RCLCPP_WARN(this->get_logger(), "尝试重新连接声纳...");
        close(socket_fd_);
        udp_connect();
    }

    // ==================== 定时器回调 ====================
    void cmd_callback()
    {
        if (socket_fd_ < 0) return;

        // 如果声纳是开启状态，周期性发送命令维持扫描
        if (work_status_ != STATUS_STOP) {
            ssize_t sent = write(socket_fd_, cmd_buf_, CMD_SIZE);
            if (sent < 0) {
                RCLCPP_WARN(this->get_logger(), "发送命令失败: %s", strerror(errno));
            }
        }
    }

    void read_callback()
    {
        if (socket_fd_ < 0) return;

        // 持续读取直到缓冲区空
        int total_read = 0;
        while (true) {
            ssize_t n = read(socket_fd_, recv_buf_, sizeof(recv_buf_));
            if (n > 0) {
                read_len_ = static_cast<int>(n);
                total_read += read_len_;
                parse_package_from_buffer();
            } else {
                break;  // 没有更多数据
            }
        }

        // 检查 socket 错误
        if (total_read == 0 && errno != EWOULDBLOCK && errno != EAGAIN) {
            if (errno != 0) {
                RCLCPP_WARN(this->get_logger(), "Socket 读取错误: %s", strerror(errno));
                reconnect();
            }
        }
    }

    // ==================== 数据包解析 ====================
    /**
     * @brief 从接收缓冲区中解析 FE/FD 帧，支持分包重组
     */
    void parse_package_from_buffer()
    {
        int bad_data = 0;

        for (int i = 0; i < read_len_; i++)
        {
            // ---- 处理历史遗留数据 (分包场景) ----
            if (i == 0 && buf_left_len_ > 0) {
                if (buf_left_len_ + read_len_ >= 8) {
                    uint8_t end_byte = (buf_left_len_ > 7)
                        ? leftover_buf_[7] : recv_buf_[7 - buf_left_len_];
                    if (end_byte != FRAME_TAIL) {
                        bad_data += buf_left_len_;
                        buf_left_len_ = 0;
                        i--;
                        continue;
                    }

                    uint8_t data_lo = (buf_left_len_ > 3)
                        ? leftover_buf_[3] : recv_buf_[3 - buf_left_len_];
                    uint8_t data_hi = (buf_left_len_ > 4)
                        ? leftover_buf_[4] : recv_buf_[4 - buf_left_len_];
                    int data_len = ((data_hi & 0x7F) << 7) | (data_lo & 0x7F);

                    if (data_len < 0 || data_len > 1000) {
                        bad_data += buf_left_len_;
                        buf_left_len_ = 0;
                        i--;
                        continue;
                    }

                    int total_needed = 8 + data_len + 2;
                    if (buf_left_len_ + read_len_ < total_needed) {
                        bad_data += buf_left_len_;
                        buf_left_len_ = 0;
                        i--;
                        continue;
                    }

                    // 拼接完整包
                    memcpy(pkg_buf_, leftover_buf_, buf_left_len_);
                    memcpy(pkg_buf_ + buf_left_len_, recv_buf_,
                           total_needed - buf_left_len_);
                    pkg_len_ = total_needed;
                    publish_sonar_data();
                    buf_left_len_ = 0;
                    i += total_needed - buf_left_len_ - 1;
                    continue;
                } else {
                    bad_data += buf_left_len_ + read_len_;
                    buf_left_len_ = 0;
                    break;
                }
            }

            // ---- 寻找帧头 ----
            if (recv_buf_[i] != FRAME_HEAD) {
                bad_data++;
                continue;
            }

            // 剩余不足8字节(最小应答帧)，缓存
            if (read_len_ - i < 8) {
                memcpy(leftover_buf_, recv_buf_ + i, read_len_ - i);
                buf_left_len_ = read_len_ - i;
                break;
            }

            // 检查帧尾
            if (recv_buf_[i + 7] != FRAME_TAIL) {
                bad_data += 8;
                i += 7;
                continue;
            }

            // 计算数据长度
            uint8_t data_lo = recv_buf_[i + 3];
            uint8_t data_hi = recv_buf_[i + 4];
            int data_len = ((data_hi & 0x7F) << 7) | (data_lo & 0x7F);

            if (data_len < 0 || data_len > 1000) {
                bad_data += 8;
                i += 7;
                continue;
            }

            int total_needed = 8 + data_len + 2;

            // 包不完整，缓存
            if (read_len_ - i < total_needed) {
                memcpy(leftover_buf_, recv_buf_ + i, read_len_ - i);
                buf_left_len_ = read_len_ - i;
                break;
            }

            // 完整包，处理
            memcpy(pkg_buf_, recv_buf_ + i, total_needed);
            pkg_len_ = total_needed;
            publish_sonar_data();
            i += total_needed - 1;
        }

        if (bad_data > 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "解析异常: 跳过 %d 字节无效数据", bad_data);
        }
    }

    // ==================== 数据发布 ====================
    void publish_sonar_data()
    {
        // 纯应答包(无数据)，跳过
        if (pkg_len_ <= 8) return;

        auto now = this->now();

        // ---- 解析角度 ----
        uint8_t  angle_lo = pkg_buf_[5];
        uint8_t  angle_hi = pkg_buf_[6];
        int      raw_angle = ((angle_hi & 0x7F) << 7) | (angle_lo & 0x7F);
        double   angle_rad = (raw_angle / 800.0 * 360.0) * M_PI / 180.0;

        // ---- 解析剖面边界 ----
        int n_points = pkg_len_ - 8 - 2;
        uint8_t profile_lo = pkg_buf_[pkg_len_ - 2];
        uint8_t profile_hi = pkg_buf_[pkg_len_ - 1];
        int     profile_raw = ((profile_hi & 0x7F) << 7) | (profile_lo & 0x7F);
        double  profile_range = profile_raw * 2.5e-6 * sound_speed_ / 2.0;

        // ---- 构建 Original PointCloud2 ----
        auto cloud_orig = build_cloud2(n_points);
        auto cloud_rig  = build_cloud2(n_points);
        auto cloud_bnd  = build_cloud2(1);

        size_t orig_count = 0, rig_count = 0;

        // 计算各采样点的笛卡尔坐标
        double cos_a = cos(angle_rad);
        double sin_a = sin(angle_rad);
        double range_per_sample = 2.5e-6 * sound_speed_ / 2.0;

        sensor_msgs::PointCloud2Iterator<float> ox(*cloud_orig, "x");
        sensor_msgs::PointCloud2Iterator<float> oy(*cloud_orig, "y");
        sensor_msgs::PointCloud2Iterator<float> oz(*cloud_orig, "z");
        sensor_msgs::PointCloud2Iterator<float> oi(*cloud_orig, "intensity");

        sensor_msgs::PointCloud2Iterator<float> rx(*cloud_rig, "x");
        sensor_msgs::PointCloud2Iterator<float> ry(*cloud_rig, "y");
        sensor_msgs::PointCloud2Iterator<float> rz(*cloud_rig, "z");
        sensor_msgs::PointCloud2Iterator<float> ri(*cloud_rig, "intensity");

        for (int i = 0; i < n_points; i++)
        {
            int intensity = pkg_buf_[8 + i];
            double range = (i + 1) * range_per_sample;

            if (intensity > 0) {
                // Original 点云 (所有有效回波点)
                *ox = static_cast<float>(range * cos_a);
                *oy = static_cast<float>(-range * sin_a);
                *oz = 0.0f;
                *oi = static_cast<float>(intensity);
                ++ox; ++oy; ++oz; ++oi;
                orig_count++;

                // Rigidity 点云 (差分刚性检测)
                int rigidity = 0;
                if (i > 3 && i > min_range_) {
                    rigidity = pkg_buf_[8 + i] - pkg_buf_[8 + i - 3];
                }
                if (rigidity > rigidity_threshold_) {
                    *rx = static_cast<float>(range * cos_a);
                    *ry = static_cast<float>(-range * sin_a);
                    *rz = 0.0f;
                    *ri = static_cast<float>(rigidity);
                    ++rx; ++ry; ++rz; ++ri;
                    rig_count++;
                }
            }
        }

        // 调整大小并发布
        resize_cloud2(*cloud_orig, orig_count);
        resize_cloud2(*cloud_rig,  rig_count);

        cloud_orig->header.stamp = now;
        cloud_rig->header.stamp  = now;
        pub_original_->publish(std::move(*cloud_orig));
        pub_rigidity_->publish(std::move(*cloud_rig));

        // ---- 构建 Boundary 点云 (单点) ----
        {
            sensor_msgs::PointCloud2Iterator<float> bx(*cloud_bnd, "x");
            sensor_msgs::PointCloud2Iterator<float> by(*cloud_bnd, "y");
            sensor_msgs::PointCloud2Iterator<float> bz(*cloud_bnd, "z");
            sensor_msgs::PointCloud2Iterator<float> bi(*cloud_bnd, "intensity");

            *bx = static_cast<float>(profile_range * cos_a);
            *by = static_cast<float>(-profile_range * sin_a);
            *bz = 0.0f;
            *bi = 255.0f;

            cloud_bnd->header.stamp = now;
            pub_boundary_->publish(std::move(*cloud_bnd));
        }
    }

    /**
     * @brief 构建带 x/y/z/intensity 字段的空 PointCloud2
     */
    std::unique_ptr<PointCloud2> build_cloud2(size_t num_points)
    {
        auto cloud = std::make_unique<PointCloud2>();
        cloud->header.frame_id = frame_id_;
        cloud->height = 1;
        cloud->is_bigendian = false;
        cloud->is_dense = true;
        cloud->point_step = 16;  // 4 × float32
        cloud->fields.resize(4);

        cloud->fields[0].name     = "x";
        cloud->fields[0].offset   = 0;
        cloud->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud->fields[0].count    = 1;

        cloud->fields[1].name     = "y";
        cloud->fields[1].offset   = 4;
        cloud->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud->fields[1].count    = 1;

        cloud->fields[2].name     = "z";
        cloud->fields[2].offset   = 8;
        cloud->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud->fields[2].count    = 1;

        cloud->fields[3].name     = "intensity";
        cloud->fields[3].offset   = 12;
        cloud->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud->fields[3].count    = 1;

        cloud->width    = static_cast<uint32_t>(num_points);
        cloud->row_step = cloud->point_step * cloud->width;
        cloud->data.resize(cloud->row_step);

        return cloud;
    }

    /**
     * @brief 调整 PointCloud2 实际点数 (重新分配data)
     */
    void resize_cloud2(PointCloud2& cloud, size_t actual_points)
    {
        cloud.width    = static_cast<uint32_t>(actual_points);
        cloud.row_step = cloud.point_step * cloud.width;
        cloud.data.resize(cloud.row_step);
    }

    // ==================== 成员变量 ====================

    // ROS2
    rclcpp::Publisher<PointCloud2>::SharedPtr  pub_original_;
    rclcpp::Publisher<PointCloud2>::SharedPtr  pub_rigidity_;
    rclcpp::Publisher<PointCloud2>::SharedPtr  pub_boundary_;
    rclcpp::Service<rov_sonar_interface::srv::SonarConfig>::SharedPtr config_srv_;
    rclcpp::TimerBase::SharedPtr read_timer_;
    rclcpp::TimerBase::SharedPtr cmd_timer_;

    // 参数
    std::string server_ip_;
    uint16_t    server_port_;
    std::string frame_id_;

    uint8_t  work_status_;
    uint8_t  range_;
    uint8_t  start_gain_;
    uint8_t  logf_;
    uint8_t  absorption_;
    int      sound_speed_;
    int      train_angle_;
    int      sector_width_;
    int      data_len_;
    uint8_t  pulse_type_;
    int      gate_;
    int      min_range_;
    int      delay_us_;
    uint8_t  frequency_;
    int      rigidity_threshold_;
    int      cmd_interval_ms_;
    double   read_rate_hz_;

    // 网络
    int                socket_fd_{-1};
    struct sockaddr_in addr_;

    // 缓冲区
    uint8_t cmd_buf_[CMD_SIZE]{};
    uint8_t recv_buf_[RECV_BUF_SIZE]{};
    uint8_t leftover_buf_[LEFTOVER_SIZE]{};
    uint8_t pkg_buf_[PKG_BUF_SIZE]{};

    int read_len_{0};
    int buf_left_len_{0};
    int pkg_len_{0};
};

// ==================== 入口 ====================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SonarOmniDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
