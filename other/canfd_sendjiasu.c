#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>

#define CAN_INTERFACE "can0"
#define CANFD_FRAME_ID 0x123 // 帧ID，可根据需要修改
#define DATA_LENGTH 245      // 0x11到0xFF共245个字节

int main()
{
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct canfd_frame frame;

    // 创建CAN原始套接字
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("socket");
        return EXIT_FAILURE;
    }

    // 指定CAN接口为can0
    strcpy(ifr.ifr_name, CAN_INTERFACE);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("ioctl");
        close(s);
        return EXIT_FAILURE;
    }

    // 设置CAN地址结构
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 绑定套接字到CAN接口
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        close(s);
        return EXIT_FAILURE;
    }

    // 启用CAN FD模式
    int enable_canfd = 1;
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0)
    {
        perror("setsockopt CAN_RAW_FD_FRAMES");
        close(s);
        return EXIT_FAILURE;
    }

    // 初始化CAN FD帧
    frame.can_id = CANFD_FRAME_ID;
    frame.len = DATA_LENGTH; // CAN FD支持最长64字节标准帧和64-640字节扩展帧
    frame.flags = CANFD_BRS; // 启用比特率切换(加速帧)

    // 填充数据: 0x11到0xFF
    for (int i = 0; i < DATA_LENGTH; i++)
    {
        frame.data[i] = 0x11 + i;
    }

    // 发送CAN FD帧
    if (write(s, &frame, sizeof(struct canfd_frame)) != sizeof(struct canfd_frame))
    {
        perror("write");
        close(s);
        return EXIT_FAILURE;
    }

    printf("成功发送CAN FD加速帧: ID=0x%X, 长度=%d字节\n", frame.can_id, frame.len);

    // 关闭套接字
    close(s);
    return EXIT_SUCCESS;
}
