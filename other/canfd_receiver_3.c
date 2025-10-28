#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <pthread.h>
#include <signal.h>

// 全局标志，用于通知线程退出
volatile int running = 1;

// 线程数据结构，传递给每个接收线程
typedef struct
{
    const char *ifname;
} ThreadData;

// 信号处理函数，用于捕获Ctrl+C
void handle_signal(int sig)
{
    running = 0;
    printf("\nReceived exit signal. Stopping...\n");
}

// 接收CAN消息的线程函数
void *can_receiver_thread(void *arg)
{
    ThreadData *data = (ThreadData *)arg;
    const char *ifname = data->ifname;
    int s;
    int nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    struct canfd_frame fd_frame;
    int is_canfd = 0;

    // 创建CAN原始套接字
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("socket");
        pthread_exit(NULL);
    }

    // 设置接口名
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

    if (!ifr.ifr_ifindex)
    {
        perror("if_nametoindex");
        close(s);
        pthread_exit(NULL);
    }

    // 检查接口是否支持CAN FD
    int enable = 1;
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) == 0)
    {
        printf("[%s] CAN FD mode enabled\n", ifname);
        is_canfd = 1;
    }
    else
    {
        printf("[%s] Using standard CAN mode\n", ifname);
        is_canfd = 0;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 绑定套接字到CAN接口
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        close(s);
        pthread_exit(NULL);
    }

    printf("[%s] Listening for CAN messages...\n", ifname);

    // 循环接收CAN消息，直到收到退出信号
    while (running)
    {
        if (is_canfd)
        {
            // 接收CAN FD帧
            nbytes = read(s, &fd_frame, sizeof(struct canfd_frame));
            if (nbytes < 0)
            {
                if (running) // 只有在非退出状态下才报告错误
                    perror("read");
                break;
            }

            if (nbytes != sizeof(struct canfd_frame))
            {
                fprintf(stderr, "[%s] Read incomplete CAN FD frame\n", ifname);
                continue;
            }

            // 打印CAN FD消息信息，区分标准帧和扩展帧
            if (fd_frame.can_id & CAN_EFF_FLAG)
            {
                printf("[%s] CAN FD Frame: ID=0x%08X (extended), DLC=%d, Data=[",
                       ifname, fd_frame.can_id & CAN_EFF_MASK, fd_frame.len);
            }
            else
            {
                printf("[%s] CAN FD Frame: ID=0x%03X (standard), DLC=%d, Data=[",
                       ifname, fd_frame.can_id & CAN_SFF_MASK, fd_frame.len);
            }

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
                if (running) // 只有在非退出状态下才报告错误
                    perror("read");
                break;
            }

            if (nbytes != sizeof(struct can_frame))
            {
                fprintf(stderr, "[%s] Read incomplete CAN frame\n", ifname);
                continue;
            }

            // 打印标准CAN消息信息，区分标准帧和扩展帧
            if (frame.can_id & CAN_EFF_FLAG)
            {
                printf("[%s] Standard CAN Frame: ID=0x%08X (extended), DLC=%d, Data=[",
                       ifname, frame.can_id & CAN_EFF_MASK, frame.can_dlc);
            }
            else
            {
                printf("[%s] Standard CAN Frame: ID=0x%03X (standard), DLC=%d, Data=[",
                       ifname, frame.can_id & CAN_SFF_MASK, frame.can_dlc);
            }

            for (int i = 0; i < frame.can_dlc; i++)
            {
                printf("%02X", frame.data[i]);
                if (i < frame.can_dlc - 1)
                    printf(" ");
            }
            printf("]\n");
        }
    }

    // 关闭套接字
    close(s);
    printf("[%s] Receiver stopped\n", ifname);
    pthread_exit(NULL);
}

int main()
{
    // 注册信号处理函数，捕获Ctrl+C
    signal(SIGINT, handle_signal);

    // 定义要监听的CAN接口
    const char *ifnames[] = {"can0", "can1", "can2"};
    const int num_interfaces = sizeof(ifnames) / sizeof(ifnames[0]);

    pthread_t threads[num_interfaces];
    ThreadData thread_data[num_interfaces];

    // 创建线程，每个线程负责一个CAN接口
    for (int i = 0; i < num_interfaces; i++)
    {
        thread_data[i].ifname = ifnames[i];
        if (pthread_create(&threads[i], NULL, can_receiver_thread, &thread_data[i]) != 0)
        {
            perror("pthread_create");
            // 如果创建线程失败，终止已创建的线程
            for (int j = 0; j < i; j++)
            {
                pthread_cancel(threads[j]);
                pthread_join(threads[j], NULL);
            }
            return EXIT_FAILURE;
        }
    }

    printf("Listening on %d CAN interfaces: can0, can1, can2\n", num_interfaces);
    printf("Press Ctrl+C to exit\n");

    // 等待所有线程结束
    for (int i = 0; i < num_interfaces; i++)
    {
        pthread_join(threads[i], NULL);
    }

    printf("All receivers stopped. Exiting.\n");
    return EXIT_SUCCESS;
}
