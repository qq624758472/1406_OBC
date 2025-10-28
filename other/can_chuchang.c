#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// 配置参数（可根据需求修改）
#define CAN_INTERFACE "can0" // 默认CAN接口
#define CANFD_FRAME_ID 0x17 // 固定CAN FD帧ID（标准11位）
#define MAX_CANFD_DATA 64    // CAN FD标准最大数据长度（硬件支持可扩至640）
#define INPUT_BUF_SIZE 512   // 用户输入缓冲区大小（足够容纳64字节数据字符串）

/**
 * @brief 解析用户输入的16进制字符串（格式：0x15 0x22 0x33）
 * @param input：用户输入的字符串
 * @param data：存储解析后的16进制数据缓冲区
 * @param max_data_len：data缓冲区的最大长度（避免溢出）
 * @return 成功：解析出的数据字节数；失败：-1
 */
int parse_hex_input(const char *input, unsigned char *data, int max_data_len)
{
    int data_cnt = 0;
    char *buf = strdup(input);      // 复制输入字符串，避免修改原数据
    char *token = strtok(buf, " "); // 按空格分割输入字段

    // 循环解析每个16进制数据
    while (token != NULL && data_cnt < max_data_len)
    {
        // 1. 校验输入格式（必须是"0xXX"格式，共4个字符）
        if (strlen(token) != 4 || token[0] != '0' || token[1] != 'x')
        {
            fprintf(stderr, "[错误] 格式无效：%s → 正确格式应为「0xXX」（如0x15）\n", token);
            free(buf);
            return -1;
        }

        // 2. 解析16进制数值（跳过"0x"前缀，取后两位字符）
        unsigned int hex_val;
        if (sscanf(token + 2, "%2x", &hex_val) != 1)
        {
            fprintf(stderr, "[错误] 16进制数无效：%s → 仅支持00~FF范围\n", token);
            free(buf);
            return -1;
        }

        // 3. 存入数据缓冲区
        data[data_cnt++] = (unsigned char)hex_val;
        token = strtok(NULL, " "); // 解析下一个数据
    }

    // 4. 处理数据超出最大长度的情况
    if (token != NULL && data_cnt >= max_data_len)
    {
        fprintf(stderr, "[警告] 输入数据过长（最大支持%d字节），已截断多余部分\n", max_data_len);
    }

    free(buf);
    return data_cnt;
}

int main()
{
    // 1. 定义CAN相关结构体和变量
    int sock_fd;                                   // CAN套接字文件描述符
    struct sockaddr_can can_addr;                  // CAN地址结构
    struct ifreq if_req;                           // 网络接口请求结构
    struct canfd_frame fd_frame;                   // CAN FD帧结构
    int enable_canfd = 1;                          // 启用CAN FD模式的标志
    char input_buf[INPUT_BUF_SIZE] = {0};          // 用户输入缓冲区
    unsigned char send_data[MAX_CANFD_DATA] = {0}; // 待发送的16进制数据

    // -------------------------------------------------------------------------
    // 2. 初始化CAN FD套接字（创建→绑定接口→启用FD模式）
    // -------------------------------------------------------------------------
    // 2.1 创建CAN原始套接字
    if ((sock_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("[错误] 创建CAN套接字失败（需root权限，执行sudo运行）");
        return EXIT_FAILURE;
    }

    // 2.2 绑定到指定CAN接口（can0）
    strncpy(if_req.ifr_name, CAN_INTERFACE, IFNAMSIZ - 1);
    if_req.ifr_name[IFNAMSIZ - 1] = '\0'; // 确保字符串终止，避免缓冲区溢出
    // 获取接口索引
    if (ioctl(sock_fd, SIOCGIFINDEX, &if_req) < 0)
    {
        perror("[错误] 获取CAN接口索引失败（确认can0已启用：ip link show can0）");
        close(sock_fd);
        return EXIT_FAILURE;
    }
    // 配置CAN地址
    can_addr.can_family = AF_CAN;
    can_addr.can_ifindex = if_req.ifr_ifindex;
    // 绑定套接字
    if (bind(sock_fd, (struct sockaddr *)&can_addr, sizeof(can_addr)) < 0)
    {
        perror("[错误] 绑定CAN接口失败");
        close(sock_fd);
        return EXIT_FAILURE;
    }

    // 2.3 启用CAN FD模式（关键：不启用则无法发送FD帧）
    if (setsockopt(sock_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                   &enable_canfd, sizeof(enable_canfd)) < 0)
    {
        perror("[错误] 启用CAN FD模式失败（可能内核不支持或接口未配置FD）");
        close(sock_fd);
        return EXIT_FAILURE;
    }

    // -------------------------------------------------------------------------
    // 3. 循环接收用户输入并发送CAN FD帧（加速帧：启用BRS）
    // -------------------------------------------------------------------------
    printf("================================ CAN FD加速帧发送程序 ================================\n");
    printf("配置信息：接口=%s | 帧ID=0x%X | 最大数据长度=%d字节 | 加速模式=启用（BRS=ON）\n",
           CAN_INTERFACE, CANFD_FRAME_ID, MAX_CANFD_DATA);
    printf("使用说明：输入16进制数据（格式示例：0x15 0x22 0x33 0x66），输入「q」或「Q」退出程序\n");
    printf("======================================================================================\n\n");

    while (1)
    {
        // 3.1 清空缓冲区，提示用户输入
        memset(input_buf, 0, sizeof(input_buf));
        memset(send_data, 0, sizeof(send_data));
        printf("请输入16进制数据：");

        // 读取用户输入（fgets会包含换行符，后续需处理）
        if (fgets(input_buf, sizeof(input_buf), stdin) == NULL)
        {
            fprintf(stderr, "\n[错误] 读取用户输入失败\n");
            continue;
        }

        // 3.2 处理换行符（fgets会读取末尾的'\n'，替换为字符串终止符）
        input_buf[strcspn(input_buf, "\n")] = '\0';

        // 3.3 退出逻辑（输入q或Q）
        if (strcmp(input_buf, "q") == 0 || strcmp(input_buf, "Q") == 0)
        {
            printf("\n程序正常退出\n");
            break;
        }

        // 3.4 解析用户输入的16进制数据
        int data_len = parse_hex_input(input_buf, send_data, MAX_CANFD_DATA);
        if (data_len <= 0)
        {
            fprintf(stderr, "[提示] 本次输入无效，跳过发送\n\n");
            continue;
        }

        // 3.5 构造CAN FD帧（加速帧：启用BRS标志）
        memset(&fd_frame, 0, sizeof(struct canfd_frame));

        fd_frame.len = data_len; // 实际数据长度
        // 4. 初始化CAN FD帧（含BRS位）
        //fd_frame.can_id = CANFD_FRAME_ID;           // 标准帧ID（如需扩展帧，加CAN_EFF_FLAG）
        fd_frame.flags = CANFD_BRS;                 // 启用比特率切换（BRS位）
        fd_frame.can_id = CANFD_FRAME_ID | CAN_EFF_FLAG ;

        memcpy(fd_frame.data, send_data, data_len); // 复制待发送数据

        // 3.6 发送CAN FD帧
        ssize_t send_bytes = write(sock_fd, &fd_frame, sizeof(struct canfd_frame));
        if (send_bytes != sizeof(struct canfd_frame))
        {
            perror("[错误] 发送CAN FD帧失败");
            fprintf(stderr, "[细节] 实际发送字节数：%zd / 预期：%zd\n\n",
                    send_bytes, sizeof(struct canfd_frame));
            continue;
        }

        // 3.7 发送成功，打印详情
        printf("[成功] 发送完成！\n");
        printf("       数据长度：%d 字节\n", data_len);
        printf("       数据内容：");
        for (int i = 0; i < data_len; i++)
        {
            printf("0x%02X ", fd_frame.data[i]);
        }
        printf("\n\n");
    }

    // 4. 关闭套接字，释放资源
    close(sock_fd);
    return EXIT_SUCCESS;
}
