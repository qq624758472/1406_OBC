#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

#define DEFAULT_BAUDRATE B115200

speed_t get_baudrate_speed(int baud)
{
    switch (baud)
    {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    default:
        return DEFAULT_BAUDRATE;
    }
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        fprintf(stderr, "Usage: %s <channel_num (0-4)> [-b baudrate] [-p parity]\n", argv[0]);
        return 1;
    }

    int chan = atoi(argv[1]);
    if (chan < 0 || chan > 4)
    {
        fprintf(stderr, "Invalid channel number: %d (must be 0-4)\n", chan);
        return 1;
    }

    int baud = 115200;
    char parity[8] = "none";

    // 解析其他参数
    for (int i = 2; i < argc; i++)
    {
        if (strcmp(argv[i], "-b") == 0 && i + 1 < argc)
        {
            baud = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc)
        {
            strncpy(parity, argv[++i], sizeof(parity) - 1);
        }
    }

    char dev_path[64];
    snprintf(dev_path, sizeof(dev_path), "/dev/ttyGH%d", chan);

    int fd = open(dev_path, O_RDWR | O_NOCTTY | O_NDELAY);
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
    speed_t speed = get_baudrate_speed(baud);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 基础串口配置：8N1，无流控
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag |= (CLOCAL | CREAD);

    // 设置奇偶校验
    if (strcmp(parity, "even") == 0)
    {
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
    }
    else if (strcmp(parity, "odd") == 0)
    {
        options.c_cflag |= PARENB;
        options.c_cflag |= PARODD;
    }

    // 原始模式
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    // 设置超时机制（非阻塞轮询）
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    tcsetattr(fd, TCSANOW, &options);

    // ➤ 启动时发送4个十六进制字节
    unsigned char tx_buf[] = {0xAA, 0xFF, 0xBB, 0xAA};
    write(fd, tx_buf, sizeof(tx_buf));
    printf("Sent: ");
    for (size_t i = 0; i < sizeof(tx_buf); i++)
    {
        printf("%02X ", tx_buf[i]);
    }
    printf("\n");

    // ➤ 持续轮询接收
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
        // if (n > 0)
        // {
        //     write(fd, rx_buf, n);
        // }

        usleep(1000); // 1ms 延时，降低CPU占用
    }

    close(fd);
    return 0;
}
