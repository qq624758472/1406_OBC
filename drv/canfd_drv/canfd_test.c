#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <fcntl.h> // 用于设置非阻塞模式
#include <sys/select.h> // 用于设置接收超时

int main() {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct canfd_frame tx_frame;
    struct canfd_frame rx_frame; // 接收帧结构体
    int i, j, k;
    int rx_count = 0; // 累计接收数量
    struct timeval timeout; // 接收超时设置
    fd_set readfds;
    int can_num; // CAN接口编号
    char can_name[10]; // CAN接口名称
    
    // 定义要测试的 DLC 值列表
    int dlc_values[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    int dlc_count = sizeof(dlc_values) / sizeof(dlc_values[0]);
    int send_count = 1000; // 每个长度发送的次数

    // 创建 CAN RAW 套接字
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket");
        return 1;
    }

    // 让用户输入CAN接口编号（3-7）
    printf("请输入CAN接口编号 (3-7): ");
    if (scanf("%d", &can_num) != 1) {
        fprintf(stderr, "输入无效\n");
        close(s);
        return 1;
    }
    
    // 验证输入范围
    if (can_num < 3 || can_num > 7) {
        fprintf(stderr, "接口编号必须在3-7之间\n");
        close(s);
        return 1;
    }
    
    // 构建CAN接口名称
    sprintf(can_name, "can%d", can_num);
    
    // 获取CAN接口索引
    strcpy(ifr.ifr_name, can_name);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        close(s);
        return 1;
    }

    // 绑定到CAN接口
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(s);
        return 1;
    }

    // 启用 CAN FD 支持
    int enable_canfd = 1;
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
        perror("setsockopt CAN_RAW_FD_FRAMES");
        close(s);
        return 1;
    }

    printf("开始发送 CAN FD 帧 (接口: %s)...\n", can_name);
    
    // 对每个 DLC 值进行测试
    for (i = 0; i < dlc_count; i++) {
        int current_dlc = dlc_values[i];
        
        // 准备 CAN FD 帧
        memset(&tx_frame, 0, sizeof(tx_frame));
        tx_frame.can_id = 0x123; // 标准帧 ID
        tx_frame.len = current_dlc; // 设置数据长度
        
        // 填充数据
        for (j = 0; j < current_dlc; j++) {
            tx_frame.data[j] = j % 256; // 数据填充为0-255循环
        }
        
        printf("发送 DLC = %d 字节的数据，共%d次...\n", current_dlc, send_count);
        
        // 发送指定次数
        for (k = 0; k < send_count; k++) {
            if (write(s, &tx_frame, sizeof(tx_frame)) != sizeof(tx_frame)) {
                perror("write");
                close(s);
                return 1;
            }
            
            // 可选：添加小延时，避免发送过快
            usleep(100); // 100微秒
        }
        
        printf("DLC = %d 字节的数据发送完成\n", current_dlc);
    }
    
    printf("所有数据发送完成\n");
    
    // 不需要设置非阻塞模式，参考 canfd_wr.c 的实现方式
    
    printf("开始接收 CAN FD 帧，按Ctrl+C结束接收...\n");
    
    while (1) {
        ssize_t nbytes = read(s, &rx_frame, sizeof(rx_frame));
        
        if (nbytes < 0) {
            perror("read");
            break;
        }
        
        if (nbytes < sizeof(struct can_frame)) {
            fprintf(stderr, "不完整的 CAN 帧\n");
            continue;
        }
        
        rx_count++;
        printf("接收到第 %d 帧：ID=0x%03X, DLC=%d 字节\n", rx_count, rx_frame.can_id & CAN_EFF_MASK, rx_frame.len);
        /*printf("数据：");
        for (j = 0; j < rx_frame.len; j++) {
            printf("%02X ", rx_frame.data[j]);
        }*/
        printf("\n");
    }
    
    printf("接收结束，共接收了 %d 帧\n", rx_count);
    
    close(s);
    return 0;
}