#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define CAN_INTERFACE "can0"
#define CANFD_FRAME_ID 0x17 // 帧ID（根据需求修改）
#define DATA_LEN 64          // 先尝试64字节（多数硬件支持），成功后可增至640

int main()
{
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct canfd_frame frame;
    int enable_canfd = 1;

    // 1. 创建CAN原始套接字
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("socket创建失败");
        return EXIT_FAILURE;
    }

    // 2. 绑定到can0接口
    strcpy(ifr.ifr_name, CAN_INTERFACE);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("获取接口索引失败（确认can0是否存在）");
        close(s);
        return EXIT_FAILURE;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("绑定接口失败");
        close(s);
        return EXIT_FAILURE;
    }

    // 3. 启用CAN FD模式（关键步骤）
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0)
    {
        perror("启用CAN FD失败（可能内核不支持或接口未配置FD）");
        close(s);
        return EXIT_FAILURE;
    }

    // 4. 初始化CAN FD帧（含BRS位）
    frame.can_id = CANFD_FRAME_ID; // 标准帧ID（如需扩展帧，加CAN_EFF_FLAG）
    frame.len = DATA_LEN;          // 数据长度（0-640，需硬件支持）
    //frame.flags = CANFD_BRS;       // 启用比特率切换（BRS位）
#if 0
    // 填充数据（0x11到0xFF，根据长度截断）
    for (int i = 0; i < frame.len; i++)
    {
        frame.data[i] = 0x11 + i;
        if (frame.data[i] > 0xFF)
            frame.data[i] = 0xFF; // 防止溢出
    }
#else
    memset(frame.data,0x55, frame.len);
#endif
    // 5. 发送帧
    ssize_t nbytes = write(s, &frame, sizeof(struct canfd_frame));
    if (nbytes != sizeof(struct canfd_frame))
    {
        perror("发送失败（检查接口配置、数据长度、BRS支持）");
        printf("实际发送字节数: %zd (预期: %zd)\n", nbytes, sizeof(struct canfd_frame));
        close(s);
        return EXIT_FAILURE;
    }

    printf("发送成功: ID=0x%X, 长度=%d字节, BRS=%s\n",
           frame.can_id, frame.len,
           (frame.flags & CANFD_BRS) ? "启用" : "禁用");

    close(s);
    return EXIT_SUCCESS;
}
