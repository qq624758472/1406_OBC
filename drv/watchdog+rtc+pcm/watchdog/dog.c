#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/watchdog.h>

#define WATCHDOG_DEV "/dev/watchdog"

int main(void)
{
    int fd;
    int i;

    // 打开看门狗设备
    fd = open(WATCHDOG_DEV, O_WRONLY);
    if (fd == -1)
    {
        perror("无法打开看门狗设备");
        return EXIT_FAILURE;
    }

    // 启动看门狗（打开设备通常会自动启动看门狗）
    printf("启动看门狗...\n");

    // 喂狗10次
    // for (i = 0; i < 10; i++)
    while(1)
    {
        printf("第 %d 次喂狗\n", i + 1);

        // 喂狗操作（写入任意数据或使用IOCTL）
        if (ioctl(fd, WDIOC_KEEPALIVE, 0) == -1)
        {
            perror("喂狗失败");
            close(fd);
            return EXIT_FAILURE;
        }

        // 休眠1秒（实际应根据看门狗超时时间调整）
        sleep(1);
    }

    // 停止看门狗（发送'M'字符是标准方式）
    printf("停止看门狗...\n");
    if (write(fd, "V", 1) == -1)
    {
        perror("停止看门狗失败");
        close(fd);
        return EXIT_FAILURE;
    }

    // 关闭设备
    close(fd);
    printf("测试完成\n");

    return EXIT_SUCCESS;
}