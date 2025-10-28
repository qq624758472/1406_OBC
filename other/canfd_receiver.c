#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int main(int argc, char *argv[])
{
    int s;
    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    struct canfd_frame fd_frame;
    int is_canfd = 0;

    // 检查参数，默认使用can0接口
    const char *ifname = "can0";
    if (argc > 1)
    {
        ifname = argv[1];
    }

    // 创建CAN原始套接字
    // 尝试创建CAN FD套接字，如果失败则回退到标准CAN
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("socket");
        return EXIT_FAILURE;
    }

    // 设置接口名
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

    if (!ifr.ifr_ifindex)
    {
        perror("if_nametoindex");
        return EXIT_FAILURE;
    }

    // 检查接口是否支持CAN FD
    int enable = 1;
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) == 0)
    {
        printf("CAN FD mode enabled\n");
        is_canfd = 1;
    }
    else
    {
        printf("Using standard CAN mode\n");
        is_canfd = 0;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 绑定套接字到CAN接口
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        return EXIT_FAILURE;
    }

    printf("Listening for CAN messages on %s...\n", ifname);
    printf("Press Ctrl+C to exit\n");

    // 循环接收CAN消息
    while (1)
    {
        if (is_canfd)
        {
            // 接收CAN FD帧
            nbytes = read(s, &fd_frame, sizeof(struct canfd_frame));
            if (nbytes < 0)
            {
                perror("read");
                return EXIT_FAILURE;
            }

            if (nbytes != sizeof(struct canfd_frame))
            {
                fprintf(stderr, "Read incomplete CAN FD frame\n");
                continue;
            }

            // 打印CAN FD消息信息
            printf("CAN FD Frame: ID=0x%03X, DLC=%d, Data=[",
                   fd_frame.can_id, fd_frame.len);

            for (int i = 0; i < fd_frame.len; i++)
            {
                printf("%02X", fd_frame.data[i]);
                if (i < fd_frame.len - 1)
                    printf(" ");
            }
            printf("]\n");
        }
        else
        {
            // 接收标准CAN帧
            nbytes = read(s, &frame, sizeof(struct can_frame));
            if (nbytes < 0)
            {
                perror("read");
                return EXIT_FAILURE;
            }

            if (nbytes != sizeof(struct can_frame))
            {
                fprintf(stderr, "Read incomplete CAN frame\n");
                continue;
            }

            // 打印标准CAN消息信息
            printf("Standard CAN Frame: ID=0x%03X, DLC=%d, Data=[",
                   frame.can_id, frame.can_dlc);

            for (int i = 0; i < frame.can_dlc; i++)
            {
                printf("%02X", frame.data[i]);
                if (i < frame.can_dlc - 1)
                    printf(" ");
            }
            printf("]\n");
        }
    }

    // 关闭套接字（实际不会执行到这里，因为上面是无限循环）
    close(s);
    return EXIT_SUCCESS;
}
