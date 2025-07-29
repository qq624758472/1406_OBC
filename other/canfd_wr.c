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

int main() {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct canfd_frame tx_frame, rx_frame;

    // 创建 CAN RAW 套接字
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket");
        return 1;
    }

    // 获取 can0 接口索引
    strcpy(ifr.ifr_name, "can1");
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        close(s);
        return 1;
    }

    // 绑定到 can0
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

    // 准备并发送一帧 CAN FD 帧
    memset(&tx_frame, 0, sizeof(tx_frame));
    tx_frame.can_id = 0x123;      // 标准帧 ID
    tx_frame.len = 16;            // 16 字节数据
    for (int i = 0; i < tx_frame.len; i++) {
        tx_frame.data[i] = i;
    }

    printf("Sending CAN FD frame...\n");
    if (write(s, &tx_frame, sizeof(tx_frame)) != sizeof(tx_frame)) {
        perror("write");
        close(s);
        return 1;
    }

    // 轮询接收
    printf("Start receiving CAN FD frames...\n");
    while (1) {
        ssize_t nbytes = read(s, &rx_frame, sizeof(rx_frame));
        if (nbytes < 0) {
            perror("read");
            break;
        }

        if (nbytes < sizeof(struct can_frame)) {
            fprintf(stderr, "Incomplete CAN frame\n");
            continue;
        }

        printf("Received CAN FD frame:\n");
        printf("  ID: 0x%03X DLC: %d Data:", rx_frame.can_id & CAN_EFF_MASK, rx_frame.len);
        for (int i = 0; i < rx_frame.len; i++) {
            printf(" %02X", rx_frame.data[i]);
        }
        printf("\n");
    }

    close(s);
    return 0;
}
