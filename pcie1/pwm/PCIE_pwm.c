#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <getopt.h>
#include <errno.h>

// 内存映射配置
#define MAP_SIZE 0x1000
#define BASE_ADDR_PWM(n) (0x72230000 + ((n - 1) * 0x10000)) // PWM1~PWM6通道基地址

// CorePWM寄存器偏移地址
#define REG_PRESCALE     0x00  // 时钟分频寄存器
#define REG_PERIOD       0x04  // 周期寄存器
#define REG_PWM_ENABLE   0x08  // 使能寄存器
#define REG_PWM1_POSEDGE 0x10  // 上升沿位置寄存器
#define REG_PWM1_NEGEDGE 0x14  // 下降沿位置寄存器
#define REG_SYNC_UPDATE  0xE4  // 同步更新寄存器

// 默认参数（未指定时使用）
#define DEFAULT_PRESCALE  0x13  // 时钟分频默认值
#define DEFAULT_PERIOD    0x63  // 周期默认值
#define DEFAULT_POSEDGE   0x01  // 上升沿默认值
#define DEFAULT_NEGEDGE   0x01  // 下降沿默认值

int main(int argc, char *argv[])
{
    // 初始化参数（默认值）
    int pwm_id = -1;                  // PWM通道编号（必须指定）
    uint32_t prescale  = DEFAULT_PRESCALE;
    uint32_t period    = DEFAULT_PERIOD;
    uint32_t posedge   = DEFAULT_POSEDGE;
    uint32_t negedge   = DEFAULT_NEGEDGE;

    // 解析命令行参数
    int opt;
    while ((opt = getopt(argc, argv, "c:t:f:p:n:")) != -1) {
        switch (opt) {
            case 'c':  // PWM通道编号（1~6）
                pwm_id = atoi(optarg);
                break;
            case 't':  // REG_PRESCALE（十六进制）
                prescale = strtol(optarg, NULL, 16);
                if (errno == EINVAL) {
                    fprintf(stderr, "错误：REG_PRESCALE值无效（应为十六进制，如0x13）\n");
                    return -1;
                }
                break;
            case 'f':  // REG_PERIOD（十六进制）
                period = strtol(optarg, NULL, 16);
                if (errno == EINVAL) {
                    fprintf(stderr, "错误：REG_PERIOD值无效（应为十六进制，如0x63）\n");
                    return -1;
                }
                break;
            case 'p':  // REG_PWM1_POSEDGE（十六进制）
                posedge = strtol(optarg, NULL, 16);
                if (errno == EINVAL) {
                    fprintf(stderr, "错误：REG_PWM1_POSEDGE值无效（应为十六进制，如0x01）\n");
                    return -1;
                }
                break;
            case 'n':  // REG_PWM1_NEGEDGE（十六进制）
                negedge = strtol(optarg, NULL, 16);
                if (errno == EINVAL) {
                    fprintf(stderr, "错误：REG_PWM1_NEGEDGE值无效（应为十六进制，如0x01）\n");
                    return -1;
                }
                break;
            default:   // 无效选项
                fprintf(stderr, "用法：%s -c <通道号> [-t 分频] [-f 周期] [-p 上升沿] [-n 下降沿]\n", argv[0]);
                fprintf(stderr, "示例：%s -c 1 -t 0x13 -f 0x63 -p 0x01 -n 0x01\n", argv[0]);
                return -1;
        }
    }

    // 校验通道编号
    if (pwm_id == -1) {
        fprintf(stderr, "错误：必须使用-c指定PWM通道（1~6）\n");
        return -1;
    }
    if (pwm_id < 1 || pwm_id > 6) {
        fprintf(stderr, "错误：通道号必须在1~6之间\n");
        return -1;
    }

    // 计算PWM基地址
    uint32_t pwm_base_addr = BASE_ADDR_PWM(pwm_id);

    // 打开物理内存设备
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("打开/dev/mem失败");
        return -1;
    }

    // 内存映射PWM寄存器
    void *map_base = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, pwm_base_addr);
    if (map_base == MAP_FAILED) {
        perror("内存映射失败");
        close(fd);
        return -1;
    }

    // 寄存器操作指针
    volatile uint8_t *pwm = (volatile uint8_t *)map_base;

    // 配置PWM寄存器
    pwm[REG_PRESCALE]     = prescale;    // 时钟分频
    pwm[REG_PERIOD]       = period;      // 周期设置
    pwm[REG_PWM1_POSEDGE] = posedge;     // 上升沿位置
    pwm[REG_PWM1_NEGEDGE] = negedge;     // 下降沿位置

    // 启用PWM并同步更新
    pwm[REG_PWM_ENABLE] = 0x01;  // 使能输出
    pwm[REG_SYNC_UPDATE] = 0x01; // 同步配置

    // 打印配置信息
    printf("PWM%d 配置完成：\n", pwm_id);
    printf("  分频寄存器(REG_PRESCALE): 0x%02X\n", prescale);
    printf("  周期寄存器(REG_PERIOD): 0x%02X\n", period);
    printf("  上升沿寄存器(POSEDGE): 0x%02X\n", posedge);
    printf("  下降沿寄存器(NEGEDGE): 0x%02X\n", negedge);

    // 释放资源
    munmap((void *)pwm, MAP_SIZE);
    close(fd);

    return 0;
}