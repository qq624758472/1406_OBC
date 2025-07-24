/*
 * @Description:
 * @Version: 2.0
 * @Autor: ruog__
 * @Date: 2024-08-15 14:44:07
 * @LastEditors: ruog__
 * @LastEditTime: 2025-02-24 13:58:45
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdint.h>

#define REGISTER_ADDRESS 0x41250000
#define REGISTER_SIZE 0xff // 适当设置寄存器的大小

int main()
{
    int fd;
    volatile uint32_t *register_ptr;

    // 打开/dev/mem设备文件
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0)
    {
        perror("open");
        exit(EXIT_FAILURE);
    }

    // 将物理地址映射到用户空间
    register_ptr = mmap(NULL, REGISTER_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, REGISTER_ADDRESS);
    if (register_ptr == MAP_FAILED)
    {
        perror("mmap");
        close(fd);
        exit(EXIT_FAILURE);
    }

    // 每秒给寄存器发脉冲信号
    while (1)
    {
        *register_ptr = 1; // 发脉冲信号，具体值根据需要设置
        usleep(10);    // 等待100毫秒，具体时间根据需要设置
        *register_ptr = 0; // 关闭脉冲信号
        // usleep(900000);    // 等待900毫秒
        sleep(1);
    }

    // 解除映射和关闭文件
    if (munmap((void *)register_ptr, REGISTER_SIZE) == -1)
    {
        perror("munmap");
    }
    close(fd);

    return 0;
}
