#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

#define SERIAL_PORT "/dev/ttyLP1"
#define BAUDRATE B115200

int main() {
    int fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
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

    // 设置超时机制（非阻塞轮询）
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    tcsetattr(fd, TCSANOW, &options);

    // ➤ 启动时发送4个十六进制字节
    unsigned char tx_buf[] = {0xAA, 0xFF, 0xBB, 0xAA};
    write(fd, tx_buf, sizeof(tx_buf));
    printf("Sent: ");
    for (size_t i = 0; i < sizeof(tx_buf); i++) {
        printf("%02X ", tx_buf[i]);
    }
    printf("\n");

    // ➤ 持续轮询接收
    unsigned char rx_buf[256];
    while (1) {
        int n = read(fd, rx_buf, sizeof(rx_buf));
        if (n > 0) {
            printf("Received [%d]: ", n);
            for (int i = 0; i < n; i++) {
                printf("%02X ", rx_buf[i]);
            }
            printf("\n");
        }

        usleep(10000); // 10ms 延时，降低CPU占用
    }

    close(fd);
    return 0;
}
