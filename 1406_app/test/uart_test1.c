#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <pthread.h> // 引入线程库

#define SERIAL_PORT "/dev/ttyLP1"
#define BAUDRATE B115200
#define SEND_INTERVAL 100000 // 发送间隔(微秒)，可根据需要调整

// 全局文件描述符，供两个线程访问
int fd;

// 发送线程函数：循环发送数据
void *send_thread(void *arg)
{
    unsigned char tx_buf[] = {0xAA, 0xFF, 0xBB, 0xAA};
    while (1)
    {
        // 发送数据
        write(fd, tx_buf, sizeof(tx_buf));
        // 打印发送内容
        printf("Sent: ");
        for (size_t i = 0; i < sizeof(tx_buf); i++)
        {
            printf("%02X ", tx_buf[i]);
        }
        printf("\n");
        // 间隔一段时间再发送
        usleep(SEND_INTERVAL);
    }
    return NULL;
}

// 接收线程函数：循环接收并打印
void *recv_thread(void *arg)
{
    unsigned char rx_buf[256];
    while (1)
    {
        int n = read(fd, rx_buf, sizeof(rx_buf));
        if (n > 0)
        {
            printf("Received [%d]: ", n);
            for (int i = 0; i < n; i++)
            {
                printf("%02X ", rx_buf[i]);
            }
            printf("\n");
        }
        // 短暂延时降低CPU占用
        usleep(10000);
    }
    return NULL;
}

int main()
{
    pthread_t send_tid, recv_tid; // 线程ID

    // 打开串口
    fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        perror("open serial port");
        return 1;
    }

    // 取消非阻塞
    fcntl(fd, F_SETFL, 0);

    struct termios options;
    tcgetattr(fd, &options);

    // 设置波特率
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);

    // 基础串口配置：8N1，无流控
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= (CLOCAL | CREAD);

    // 原始模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    // 设置超时机制
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    tcsetattr(fd, TCSANOW, &options);

    // 创建发送线程
    if (pthread_create(&send_tid, NULL, send_thread, NULL) != 0)
    {
        perror("create send thread");
        close(fd);
        return 1;
    }

    // 创建接收线程
    if (pthread_create(&recv_tid, NULL, recv_thread, NULL) != 0)
    {
        perror("create receive thread");
        close(fd);
        return 1;
    }

    // 等待线程结束（实际不会执行到这里，因为线程是无限循环）
    pthread_join(send_tid, NULL);
    pthread_join(recv_tid, NULL);

    close(fd);
    return 0;
}