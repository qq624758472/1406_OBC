#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// 帮助信息（明确每个单字符选项的功能）
void print_usage(const char *progname)
{
    fprintf(stderr, "Usage: %s [options]\n", progname);
    fprintf(stderr, "Single-character options (all supported):\n");
    fprintf(stderr, "  -i <ifname>   CAN interface name (default: can0)\n");
    fprintf(stderr, "  -f            Enable CAN FD mode (required for -b)\n");
    fprintf(stderr, "  -n <id>       CAN frame ID (hex, default: 0x17)\n");
    fprintf(stderr, "  -d <data>     Data bytes (hex, space/comma separated, e.g. '11 22' or 'aa,bb')\n");
    fprintf(stderr, "  -b            Enable BRS (Bit Rate Switch) for CAN FD (acceleration)\n");
    fprintf(stderr, "\nExamples:\n");
    fprintf(stderr, "  1. Send standard CAN frame: %s -i can0 -n 1a5 -d '01 02 03'\n", progname);
    fprintf(stderr, "  2. Send CAN FD frame with BRS: %s -f -b -i can0 -n 23f -d '11 22 33 44'\n", progname);
}

// 解析十六进制数据（复用原逻辑）
int parse_data(const char *data_str, unsigned char *data, int max_len)
{
    int count = 0;
    char *str = strdup(data_str);
    char *token = strtok(str, " ,");

    while (token && count < max_len)
    {
        unsigned int val;
        // 解析十六进制（支持纯数字如"1a"或带0x前缀如"0x1a"）
        if (sscanf(token, "%x", &val) != 1 && sscanf(token, "0x%x", &val) != 1)
        {
            fprintf(stderr, "Error: Invalid data byte '%s' (must be hex, e.g. '1a' or '0x1a')\n", token);
            free(str);
            return -1;
        }
        if (val > 0xFF)
        {
            fprintf(stderr, "Error: Data byte '%s' too large (max 0xFF)\n", token);
            free(str);
            return -1;
        }
        data[count++] = (unsigned char)val;
        token = strtok(NULL, " ,");
    }

    free(str);
    return count;
}

int main(int argc, char *argv[])
{
    // 1. 初始化变量（默认值清晰）
    int s;                                    // CAN套接字
    struct sockaddr_can addr;                 // CAN地址结构
    struct ifreq ifr;                         // 接口请求结构
    struct can_frame std_frame;               // 标准CAN帧
    struct canfd_frame fd_frame;              // CAN FD帧
    int is_canfd = 0;                         // 是否启用CAN FD（-f控制）
    int use_brs = 0;                          // 是否启用BRS加速（-b控制）
    const char *ifname = "can0";              // 默认接口（-i可修改）
    canid_t can_id = 0x2ee00;                    // 默认帧ID（-n可修改）
    unsigned char data[CANFD_MAX_DLEN] = {0}; // 数据缓冲区
    int data_len = 0;                         // 实际数据长度（-d解析后赋值）
    int opt;                                  // getopt返回的选项字符

    // 2. 解析命令行参数（仅用单字符选项，符合getopt规则）
    // 选项字符串"fbi:n:d:"含义：
    // f/b：无参数选项；i/n/d：带参数选项（后跟:）
    while ((opt = getopt(argc, argv, "fbi:n:d:")) != -1)
    {
        switch (opt)
        {
        case 'f': // 启用CAN FD模式
            is_canfd = 1;
            break;

        case 'b': // 启用BRS加速（仅CAN FD有效）
            use_brs = 1;
            break;

        case 'i': // 指定CAN接口（如can0、can1）
            ifname = optarg;
            break;

        case 'n': // 指定CAN帧ID（十六进制）
            if (sscanf(optarg, "%x", &can_id) != 1 && sscanf(optarg, "0x%x", &can_id) != 1)
            {
                fprintf(stderr, "Error: Invalid CAN ID '%s' (use hex, e.g. '1a5' or '0x1a5')\n", optarg);
                return EXIT_FAILURE;
            }
            break;

        case 'd': // 解析数据（根据模式限制最大长度）
            data_len = parse_data(optarg, data, is_canfd ? CANFD_MAX_DLEN : CAN_MAX_DLEN);
            if (data_len < 0)
                return EXIT_FAILURE;
            break;

        case '?': // 未知选项或选项缺少参数
            print_usage(argv[0]);
            return EXIT_FAILURE;

        default: // 理论不会触发，防止异常
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    // 3. 参数合法性校验（避免逻辑错误）
    // 3.1 BRS只能在CAN FD模式下使用
    if (use_brs && !is_canfd)
    {
        fprintf(stderr, "Error: Option '-b' (BRS) requires CAN FD mode (add '-f' option)\n");
        return EXIT_FAILURE;
    }

    // 3.2 必须指定数据（-d不可缺）
    if (data_len == 0)
    {
        fprintf(stderr, "Error: No data specified (use '-d' to provide hex data)\n");
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    // 3.3 数据长度不能超过对应模式的限制
    if ((!is_canfd && data_len > CAN_MAX_DLEN) || (is_canfd && data_len > CANFD_MAX_DLEN))
    {
        fprintf(stderr, "Error: Data too long (max %d bytes for %s mode)\n",
                is_canfd ? CANFD_MAX_DLEN : CAN_MAX_DLEN,
                is_canfd ? "CAN FD" : "standard CAN");
        return EXIT_FAILURE;
    }

    // 4. CAN套接字初始化（创建→绑定→启用FD）
    // 4.1 创建CAN原始套接字
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Error: socket() failed");
        fprintf(stderr, "Hint: Requires root privilege (run with 'sudo')\n");
        return EXIT_FAILURE;
    }

    // 4.2 绑定到指定CAN接口
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';              // 确保字符串终止，避免溢出
    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name); // 获取接口索引

    if (!ifr.ifr_ifindex)
    {
        perror("Error: if_nametoindex() failed");
        fprintf(stderr, "Hint: Check if interface '%s' exists (run 'ip link show')\n", ifname);
        close(s);
        return EXIT_FAILURE;
    }

    // 4.3 启用CAN FD模式（仅当-is_canfd=1时）
    if (is_canfd)
    {
        int enable_fd = 1;
        if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd, sizeof(enable_fd)) < 0)
        {
            perror("Error: setsockopt(CAN_RAW_FD_FRAMES) failed");
            fprintf(stderr, "Hint: Interface '%s' may not support CAN FD\n", ifname);
            close(s);
            return EXIT_FAILURE;
        }
        printf("Info: CAN FD mode enabled\n");
    }
    else
    {
        printf("Info: Using standard CAN mode\n");
    }

    // 4.4 绑定套接字到CAN接口
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Error: bind() failed");
        close(s);
        return EXIT_FAILURE;
    }

    // 5. 构造并发送帧
    int nbytes; // 实际发送的字节数
    if (is_canfd)
    {
        // 5.1 构造CAN FD帧（带BRS标志）
        // fd_frame.can_id = can_id | CAN_EFF_FLAG; // 扩展帧ID（29位）
        fd_frame.can_id = can_id ; // 扩展帧ID（29位）
        fd_frame.len = data_len;                 // 数据长度
        fd_frame.flags = 0;                      // 初始化标志
        if (use_brs)
        {
            fd_frame.flags |= CANFD_BRS; // 正确设置BRS标志（通过flags字段）
            printf("Info: BRS (Bit Rate Switch) enabled\n");
        }
        memcpy(fd_frame.data, data, data_len); // 复制数据到帧

        // 5.2 发送CAN FD帧
        nbytes = write(s, &fd_frame, sizeof(struct canfd_frame));
        if (nbytes != sizeof(struct canfd_frame))
        {
            perror("Error: write() failed for CAN FD frame");
            fprintf(stderr, "Info: Sent %d bytes (expected %zu)\n", nbytes, sizeof(struct canfd_frame));
            close(s);
            return EXIT_FAILURE;
        }

        // 5.3 打印发送结果（CAN FD）
        printf("Success: Sent CAN FD frame\n");
        printf("  Interface: %s\n", ifname);
        printf("  ID: 0x%08X (extended 29-bit)\n", can_id);
        printf("  DLC: %d bytes\n", data_len);
        printf("  BRS: %s\n", use_brs ? "ON" : "OFF");
        printf("  Data: [");
    }
    else
    {
        // 5.4 构造标准CAN帧
        std_frame.can_id = can_id;              // 标准帧ID（11位）
        std_frame.can_dlc = data_len;           // 数据长度
        memcpy(std_frame.data, data, data_len); // 复制数据到帧

        // 5.5 发送标准CAN帧
        nbytes = write(s, &std_frame, sizeof(struct can_frame));
        if (nbytes != sizeof(struct can_frame))
        {
            perror("Error: write() failed for standard CAN frame");
            fprintf(stderr, "Info: Sent %d bytes (expected %zu)\n", nbytes, sizeof(struct can_frame));
            close(s);
            return EXIT_FAILURE;
        }

        // 5.6 打印发送结果（标准CAN）
        printf("Success: Sent standard CAN frame\n");
        printf("  Interface: %s\n", ifname);
        printf("  ID: 0x%03X (standard 11-bit)\n", can_id);
        printf("  DLC: %d bytes\n", data_len);
        printf("  Data: [");
    }

    // 6. 打印数据内容（两种模式通用）
    for (int i = 0; i < data_len; i++)
    {
        printf("%02X", data[i]);
        if (i < data_len - 1)
            printf(" ");
    }
    printf("]\n");

    // 7. 关闭套接字（释放资源）
    close(s);
    return EXIT_SUCCESS;
}
