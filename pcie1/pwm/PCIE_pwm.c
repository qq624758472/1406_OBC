/*
 * @Description: 
 * @Version: 2.0
 * @Autor: ruog__
 * @Date: 2025-07-24 09:38:41
 * @LastEditors: ruog__
 * @LastEditTime: 2025-07-24 09:41:23
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#define MAP_SIZE 0x1000
#define BASE_ADDR_PWM(n) (0x72230000 + ((n - 1) * 0x10000)) // PWM1~PWM6

// CorePWM寄存器偏移
#define REG_PRESCALE 0x00
#define REG_PERIOD 0x04
#define REG_PWM_ENABLE 0x08
#define REG_PWM1_POSEDGE 0x10
#define REG_PWM1_NEGEDGE 0x14
#define REG_SYNC_UPDATE 0xE4

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("用法: %s <PWM通道编号: 1~6> <时钟分频>\n", argv[0]);
        return -1;
    }

    int pwm_id = atoi(argv[1]);
    if (pwm_id < 1 || pwm_id > 6)
    {
        fprintf(stderr, "错误: PWM通道编号必须在1到6之间\n");
        return -1;
    }

    uint32_t pwm_base_addr = BASE_ADDR_PWM(pwm_id);

    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0)
    {
        perror("open /dev/mem");
        return -1;
    }

    void *map_base = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, pwm_base_addr);
    if (map_base == MAP_FAILED)
    {
        perror("mmap");
        close(fd);
        return -1;
    }

    volatile uint8_t *pwm = (volatile uint8_t *)map_base;

    // 配置时钟和周期（20 MHz → 1 MHz）
    pwm[REG_PRESCALE] = 0x13; // 粒度 = 10ns
    pwm[REG_PERIOD] = 0x63;   // (99+1)*10ns = 1us，总周期1MHz  //0X63是99，但是逻辑会默认加1所以会分成100份。

    // 配置 PWM 占空比为 50%
    pwm[REG_PWM1_POSEDGE] = 0x01;
    pwm[REG_PWM1_NEGEDGE] = 0x01;

    // 启用 PWM 输出
    pwm[REG_PWM_ENABLE] = 0x01;//输出使能

    // 同步更新
    pwm[REG_SYNC_UPDATE] = 0x01;//工作标记

    printf("已配置 PWM%d：频率1MHz，占空比50%%，运行...\n", pwm_id);

    //sleep(10);

    munmap((void *)pwm, MAP_SIZE);
    close(fd);

    return 0;
}
