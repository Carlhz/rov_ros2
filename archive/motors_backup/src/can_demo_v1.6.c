#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define CAN_INTERFACE "can0"
#define CAN_BITRATE   500000
#define NODE_LEFT     1   // COB-ID: 0x601 / 0x581
#define NODE_RIGHT    2   // COB-ID: 0x602 / 0x582

// ========== SDO 构建函数 ==========

// 写 1 字节（BYTE）
void build_sdo_write_byte(uint8_t *frame, uint16_t index, uint8_t value) {
    frame[0] = 0x2F;
    frame[1] = index & 0xFF;
    frame[2] = (index >> 8) & 0xFF;
    frame[3] = 0x00;
    frame[4] = value;
    frame[5] = 0x00;
    frame[6] = 0x00;
    frame[7] = 0x00;
}

// 写 2 字节（WORD）
void build_sdo_write_word(uint8_t *frame, uint16_t index, uint16_t value) {
    frame[0] = 0x2B;
    frame[1] = index & 0xFF;
    frame[2] = (index >> 8) & 0xFF;
    frame[3] = 0x00;
    frame[4] = value & 0xFF;
    frame[5] = (value >> 8) & 0xFF;
    frame[6] = 0x00;
    frame[7] = 0x00;
}

// 写 4 字节有符号整数（INT32，用于 0x60FF）
void build_sdo_write_int32(uint8_t *frame, uint16_t index, int32_t value) {
    frame[0] = 0x23;
    frame[1] = index & 0xFF;
    frame[2] = (index >> 8) & 0xFF;
    frame[3] = 0x00;
    frame[4] = (uint8_t)(value >> 0);
    frame[5] = (uint8_t)(value >> 8);
    frame[6] = (uint8_t)(value >> 16);
    frame[7] = (uint8_t)(value >> 24);
}

// 读请求
void build_sdo_read(uint8_t *frame, uint16_t index, uint8_t subindex) {
    frame[0] = 0x40;
    frame[1] = index & 0xFF;
    frame[2] = (index >> 8) & 0xFF;
    frame[3] = subindex;
    memset(frame + 4, 0, 4);
}

// ========== 通信辅助 ==========

int send_can_frame(int sock, uint32_t id, const uint8_t *data, uint8_t dlc) {
    struct can_frame frame = {0};
    frame.can_id = id;
    frame.can_dlc = dlc;
    memcpy(frame.data, data, dlc);
    return write(sock, &frame, sizeof(struct can_frame));
}

int wait_sdo_response(int sock, uint32_t expected_rx_id, uint8_t *out_data) {
    struct can_frame frame;
    struct timespec start, now;
    clock_gettime(CLOCK_MONOTONIC, &start);

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        long ms = (now.tv_sec - start.tv_sec) * 1000 + (now.tv_nsec - start.tv_nsec) / 1000000;
        if (ms > 100) return -1;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(sock, &readfds);
        struct timeval timeout = {0, 1000};
        if (select(sock + 1, &readfds, NULL, NULL, &timeout) <= 0) continue;

        ssize_t n = read(sock, &frame, sizeof(frame));
        if (n > 0 && frame.can_id == expected_rx_id && frame.can_dlc == 8) {
            if ((frame.data[0] & 0xE0) == 0x40 || (frame.data[0] & 0xE0) == 0x60) {
                memcpy(out_data, frame.data, 8);
                return 0;
            }
        }
    }
    return -1;
}

// ========== CAN 接口管理 ==========

int is_can_interface_up() {
    char cmd[256];
    FILE *fp;
    char line[256];
    snprintf(cmd, sizeof(cmd), "ip -br link show %s 2>/dev/null", CAN_INTERFACE);
    fp = popen(cmd, "r");
    if (!fp) return 0;
    if (fgets(line, sizeof(line), fp) != NULL) {
        pclose(fp);
        return (strstr(line, "UP") != NULL);
    }
    pclose(fp);
    return 0;
}

int setup_can_if_needed() {
    if (is_can_interface_up()) {
        printf("CAN interface %s is already UP.\n", CAN_INTERFACE);
        return 0;
    }

    char cmd[256];
    snprintf(cmd, sizeof(cmd), "ip link set %s type can bitrate %d", CAN_INTERFACE, CAN_BITRATE);
    if (system(cmd) != 0) {
        fprintf(stderr, "Failed to set CAN bitrate!\n");
        return -1;
    }
    snprintf(cmd, sizeof(cmd), "ip link set %s up", CAN_INTERFACE);
    if (system(cmd) != 0) {
        fprintf(stderr, "Failed to bring up CAN interface!\n");
        return -1;
    }
    printf("CAN interface %s configured and brought UP.\n", CAN_INTERFACE);
    return 0;
}

// ========== SDO 读操作 ==========

int read_sdo_word(int sock, uint8_t node_id, uint16_t index, uint16_t *value) {
    uint8_t req[8], resp[8];
    uint32_t tx_id = 0x600 + node_id;
    uint32_t rx_id = 0x580 + node_id;
    
    build_sdo_read(req, index, 0x00);
    if (send_can_frame(sock, tx_id, req, 8) < 0) return -1;
    if (wait_sdo_response(sock, rx_id, resp) < 0) return -1;
    if ((resp[0] & 0xE0) != 0x40) return -1;
    *value = resp[4] | (resp[5] << 8);
    return 0;
}

int read_sdo_byte(int sock, uint8_t node_id, uint16_t index, uint8_t *value) {
    uint8_t req[8], resp[8];
    uint32_t tx_id = 0x600 + node_id;
    uint32_t rx_id = 0x580 + node_id;
    
    build_sdo_read(req, index, 0x00);
    if (send_can_frame(sock, tx_id, req, 8) < 0) return -1;
    if (wait_sdo_response(sock, rx_id, resp) < 0) return -1;
    if ((resp[0] & 0xE0) != 0x40) return -1;
    *value = resp[4];
    return 0;
}

// ========== 参数解析函数 ==========

long parse_signed_rpm(const char* str) {
    char *endptr;
    errno = 0;
    long val = strtol(str, &endptr, 10);
    if (errno != 0 || *endptr != '\0') {
        fprintf(stderr, "Invalid number: '%s'\n", str);
        exit(1);
    }
    return val;
}

long parse_positive_rpm(const char* str) {
    long val = parse_signed_rpm(str);
    if (val <= 0) {
        fprintf(stderr, "Speed must be positive: '%s'\n", str);
        exit(1);
    }
    return val;
}

// ========== 电机控制 ==========

int initialize_single_motor(int sock, uint8_t node_id) {
    uint8_t sdo_data[8];
    uint32_t tx_id = 0x600 + node_id;

    printf("Initializing motor NodeID=%d...\n", node_id);

    // Shutdown
    build_sdo_write_word(sdo_data, 0x6040, 0x0006);
    send_can_frame(sock, tx_id, sdo_data, 8);
    usleep(50000);

    // Switch On
    build_sdo_write_word(sdo_data, 0x6040, 0x0007);
    send_can_frame(sock, tx_id, sdo_data, 8);
    usleep(50000);

    // Set Mode = 3 (Profile Velocity)
    build_sdo_write_byte(sdo_data, 0x6060, 0x03);
    send_can_frame(sock, tx_id, sdo_data, 8);
    usleep(50000);

    // Enable Operation
    build_sdo_write_word(sdo_data, 0x6040, 0x000F);
    send_can_frame(sock, tx_id, sdo_data, 8);
    usleep(50000);

    printf("Motor NodeID=%d initialized and enabled.\n", node_id);
    return 0;
}

int initialize_both_motors(int sock) {
    initialize_single_motor(sock, NODE_LEFT);
    initialize_single_motor(sock, NODE_RIGHT);
    return 0;
}

int set_target_velocity(int sock, uint8_t node_id, int32_t rpm_x1000) {
    uint8_t sdo_data[8];
    uint32_t tx_id = 0x600 + node_id;
    build_sdo_write_int32(sdo_data, 0x60FF, rpm_x1000);
    send_can_frame(sock, tx_id, sdo_data, 8);
    return 0;
}

int disable_single_motor(int sock, uint8_t node_id) {
    uint8_t sdo_data[8];
    uint32_t tx_id = 0x600 + node_id;
    build_sdo_write_word(sdo_data, 0x6040, 0x0006); // Shutdown
    send_can_frame(sock, tx_id, sdo_data, 8);
    return 0;
}

int disable_both_motors(int sock) {
    disable_single_motor(sock, NODE_LEFT);
    disable_single_motor(sock, NODE_RIGHT);
    printf("Both motors stopped.\n");
    return 0;
}

int check_motor_status(int sock, uint8_t node_id, int *need_init) {
    uint16_t statusword = 0;
    uint8_t mode_display = 0;
    *need_init = 0;
    
    if (read_sdo_word(sock, node_id, 0x6041, &statusword) == 0 &&
        read_sdo_byte(sock, node_id, 0x6061, &mode_display) == 0) {
        if ((statusword & 0x2000) == 0 || mode_display != 3) {
            *need_init = 1;
        }
    } else {
        *need_init = 1;
    }
    return 0;
}

// ========== 控制函数 ==========

void move_custom(int sock, long left_rpm, long right_rpm) {
    set_target_velocity(sock, NODE_LEFT, left_rpm*512*65536/1875);
    set_target_velocity(sock, NODE_RIGHT, right_rpm*512*65536/1875);
    printf("Custom move: Left=%+ld rpm, Right=%+ld rpm\n", left_rpm, right_rpm);
}

void move_forward(int sock, long speed_rpm) {
    move_custom(sock, speed_rpm, -speed_rpm);
}

void move_backward(int sock, long speed_rpm) {
    move_custom(sock, -speed_rpm, speed_rpm);
}

void turn_left(int sock, long left_speed, long right_speed) {
    if (left_speed <= 0 || right_speed <= 0 || right_speed <= left_speed) {
        fprintf(stderr, "For left turn: both speeds > 0 and right > left\n");
        exit(1);
    }
    move_custom(sock, left_speed, right_speed);
}

void turn_right(int sock, long left_speed, long right_speed) {
    if (left_speed <= 0 || right_speed <= 0 || left_speed <= right_speed) {
        fprintf(stderr, "For right turn: both speeds > 0 and left > right\n");
        exit(1);
    }
    move_custom(sock, left_speed, right_speed);
}

void pivot_left(int sock, long speed_rpm) {
    move_custom(sock, 0, speed_rpm);
}

void pivot_right(int sock, long speed_rpm) {
    move_custom(sock, speed_rpm, 0);
}

// ========== MAIN ==========

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage:\n");
        fprintf(stderr, "  %s move <L> <R>         # Custom: L=left rpm, R=right rpm (can be negative)\n", argv[0]);
        fprintf(stderr, "  %s forward <speed>      # e.g., 500 (positive only)\n", argv[0]);
        fprintf(stderr, "  %s backward <speed>     # e.g., 500 (positive only)\n", argv[0]);
        fprintf(stderr, "  %s turn left <L> <R>    # e.g., 300 500 (both positive, R>L)\n", argv[0]);
        fprintf(stderr, "  %s turn right <L> <R>   # e.g., 500 300 (both positive, L>R)\n", argv[0]);
        fprintf(stderr, "  %s pivot left <speed>   # e.g., 300 (positive only)\n", argv[0]);
        fprintf(stderr, "  %s pivot right <speed>  # e.g., 300 (positive only)\n", argv[0]);
        fprintf(stderr, "  %s stop                 # stop both motors\n", argv[0]);
        exit(1);
    }

    if (setup_can_if_needed() != 0) {
        exit(1);
    }

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) { perror("socket"); exit(1); }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, CAN_INTERFACE, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); close(s); exit(1); }

    struct sockaddr_can addr = {0};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); close(s); exit(1); }

    if (strcmp(argv[1], "stop") == 0) {
        disable_both_motors(s);
        close(s);
        return 0;
    }

    // 检查并初始化电机（仅在需要时）
    int need_init_left = 0, need_init_right = 0;
    check_motor_status(s, NODE_LEFT, &need_init_left);
    check_motor_status(s, NODE_RIGHT, &need_init_right);
    if (need_init_left || need_init_right) {
        initialize_both_motors(s);
    }

    // 核心命令：move L R
    if (strcmp(argv[1], "move") == 0) {
        if (argc != 4) {
            fprintf(stderr, "Usage: %s move <left_rpm> <right_rpm>\n", argv[0]);
            close(s); exit(1);
        }
        long left_rpm = parse_signed_rpm(argv[2]);
        long right_rpm = parse_signed_rpm(argv[3]);
        move_custom(s, left_rpm, right_rpm);
    }
    // 前进（speed 必须为正）
    else if (strcmp(argv[1], "forward") == 0) {
        if (argc != 3) {
            fprintf(stderr, "Error: missing speed value\n");
            close(s); exit(1);
        }
        long speed = parse_positive_rpm(argv[2]);
        move_forward(s, speed);
    }
    // 后退（speed 必须为正）
    else if (strcmp(argv[1], "backward") == 0) {
        if (argc != 3) {
            fprintf(stderr, "Error: missing speed value\n");
            close(s); exit(1);
        }
        long speed = parse_positive_rpm(argv[2]);
        move_backward(s, speed);
    }
    // 转向
    else if (strcmp(argv[1], "turn") == 0) {
        if (argc != 5) {
            fprintf(stderr, "Usage: turn left|right <L> <R>\n");
            close(s); exit(1);
        }
        char *direction = argv[2];
        long L = parse_positive_rpm(argv[3]);
        long R = parse_positive_rpm(argv[4]);
        if (strcmp(direction, "left") == 0) {
            turn_left(s, L, R);
        } else if (strcmp(direction, "right") == 0) {
            turn_right(s, L, R);
        } else {
            fprintf(stderr, "Direction must be 'left' or 'right'\n");
            close(s); exit(1);
        }
    }
    // 原地转
    else if (strcmp(argv[1], "pivot") == 0) {
        if (argc != 4) {
            fprintf(stderr, "Usage: pivot left|right <speed>\n");
            close(s); exit(1);
        }
        char *direction = argv[2];
        long speed = parse_positive_rpm(argv[3]);
        if (strcmp(direction, "left") == 0) {
            pivot_left(s, speed);
        } else if (strcmp(direction, "right") == 0) {
            pivot_right(s, speed);
        } else {
            fprintf(stderr, "Direction must be 'left' or 'right'\n");
            close(s); exit(1);
        }
    }
    else {
        fprintf(stderr, "Unknown command: %s\n", argv[1]);
        close(s); exit(1);
    }

    close(s);
    return 0;
}
