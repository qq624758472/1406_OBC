#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#include <stdio.h>
#include <linux/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#include <errno.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <pthread.h>
#include <poll.h>
#include <signal.h>
#include <mtd/mtd-abi.h>
#include <linux/types.h>

// arm-linux-gnueabi-gcc rtc_lyb.c -static -o rtctest
// 强制一字节对齐
#pragma pack(1)

// 定义寄存器偏移量结构体，使用普通数据类型
typedef struct
{
    unsigned char slv_reg0;
    unsigned char slv_reg1;
    unsigned int slv_reg2;
    unsigned char slv_reg3;
    unsigned char set_pps_trigger_mode;
    unsigned char clr_pps1_rx_state;
    unsigned char clr_pps2_rx_state;
    unsigned char reserved1[0x1C - 0x0A];
    unsigned char compare_en;
    unsigned char compare_mode;
    unsigned char reserved2[3];
    unsigned int compare_times;
    unsigned int compare_value;
    unsigned char current_clk_source_output;
    unsigned char state_clk_source_output;
    unsigned int inner_s_cnt_1_o_output;
    unsigned int inner_s_cnt_2_o_output;
    unsigned int second_value_output;
    unsigned int microsecond_value_output;
    unsigned char pps1_rx_state_output;
    unsigned char pps2_rx_state_output;
    // 比较功能相关寄存器
    unsigned char slv_reg7;  // 0x1C 比较使能
    unsigned char slv_reg8;  // 0x20 比较模式
    unsigned int slv_reg9;   // 0x24 比较次数
    unsigned int slv_reg10;  // 0x28 比较值低32位
    unsigned int slv_reg11;  // 0x2C 比较值高32位
    // RTC计时器和中断寄存器
    unsigned int slv_reg35;  // 0x8C RTC计时器低32位
    unsigned int slv_reg36;  // 0x90 RTC计时器高32位
    unsigned long long rtc_timer_output;
    unsigned char rtc_int_state_output; // 0x94 中断状态寄存器
} Registers;

// 恢复默认对齐方式
#pragma pack()

// 寄存器基址
#define BASE_ADDRESS 0x62000000
// 页大小，通常为 4096 字节
#define PAGE_SIZE 4096

// 内存映射函数
volatile Registers *map_registers()
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1)
    {
        perror("无法打开 /dev/mem");
        exit(EXIT_FAILURE);
    }

    off_t page_base = (BASE_ADDRESS / PAGE_SIZE) * PAGE_SIZE;
    off_t offset_within_page = BASE_ADDRESS % PAGE_SIZE;

    volatile Registers *mapped_base = (volatile Registers *)mmap(0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, page_base);
    if (mapped_base == MAP_FAILED)
    {
        perror("内存映射失败");
        close(fd);
        exit(EXIT_FAILURE);
    }

    return (volatile Registers *)((char *)mapped_base + offset_within_page);
}

// 比较中断监测线程函数 
void *compare_interrupt_monitor(void *arg) 
{ 
    volatile Registers *regs = (volatile Registers *)arg; 
    while (1) 
    { 
        // 检查中断状态寄存器的bit2（比较中断） 
        if ((regs->rtc_int_state_output & (1 << 2)) != 0) 
        { 
            printf("检测到比较中断！RTC计时器值已匹配\n");
            // 读取当前RTC计时器值
            unsigned long long current_time = ((unsigned long long)regs->slv_reg36 << 32) | regs->slv_reg35;
            printf("当前RTC计时器值: 0x%016llx\n", current_time);
            // 清除中断标志
            regs->rtc_int_state_output = (1 << 2);  
        }
        usleep(100000);  // 100ms轮询间隔
    }
    return NULL;
}

// 新增PPS中断监测线程函数
void *pps_interrupt_monitor(void *arg) 
{ 
    volatile Registers *regs = (volatile Registers *)arg; 
    while (1) 
    { 
        // 检查中断状态寄存器的bit1（PPS中断）
        if ((regs->rtc_int_state_output & (1 << 1)) != 0) 
        { 
            printf("检测到PPS中断！RTC计时器已同步\n");
            // 读取当前RTC计时器值
            unsigned long long current_time = regs->rtc_timer_output;
            printf("当前RTC计时器值: 0x%016llx\n", current_time);
            // 清除中断标志（写1清除）
            regs->rtc_int_state_output = (1 << 1);  
        }
        usleep(100000);  // 100ms轮询间隔
    }
    return NULL;
}

// 测试寄存器读写函数
void test_register_read_write(volatile Registers *base_addr, size_t offset, size_t size)
{
    unsigned long long read_value = 0;
    unsigned long long test_value = 0x55; // 简单的测试值
    volatile unsigned char *reg_addr = (volatile unsigned char *)base_addr + offset;

    // 读取初始值
    switch (size)
    {
    case 1:
        read_value = *reg_addr;
        break;
    case 4:
        read_value = *(volatile unsigned int *)reg_addr;
        break;
    case 8:
        read_value = *(volatile unsigned long long *)reg_addr;
        break;
    default:
        fprintf(stderr, "不支持的寄存器大小\n");
        return;
    }
    printf("寄存器偏移 0x%lx 初始值: 0x%llx\n", (unsigned long)offset, read_value);

    // 写入测试值
    switch (size)
    {
    case 1:
        *reg_addr = (unsigned char)test_value;
        break;
    case 4:
        *(volatile unsigned int *)reg_addr = (unsigned int)test_value;
        break;
    case 8:
        *(volatile unsigned long long *)reg_addr = test_value;
        break;
    }
    printf("向寄存器偏移 0x%lx 写入值 0x%llx\n", (unsigned long)offset, test_value);

    // 再次读取以验证
    switch (size)
    {
    case 1:
        read_value = *reg_addr;
        break;
    case 4:
        read_value = *(volatile unsigned int *)reg_addr;
        break;
    case 8:
        read_value = *(volatile unsigned long long *)reg_addr;
        break;
    }
    printf("寄存器偏移 0x%lx 写入后值: 0x%llx\n", (unsigned long)offset, read_value);
}

// 外部时钟选择
void external_clock_selection(volatile Registers *base_addr)
{
    test_register_read_write(base_addr, offsetof(Registers, slv_reg0), 1);
}

// PPS输入源选择
void pps_source_selection(volatile Registers *base_addr)
{
    test_register_read_write(base_addr, offsetof(Registers, slv_reg3), 1);
}

// 比较使能
void comparison_enable(volatile Registers *base_addr)
{
    test_register_read_write(base_addr, offsetof(Registers, compare_en), 1);
}

// 比较功能设置
void comparison_function_setup(volatile Registers *base_addr)
{
    unsigned int compare_value, compare_times;
    unsigned char compare_mode;

    printf("请输入比较值: ");
    scanf("%u", &compare_value);
    printf("请输入比较次数: ");
    scanf("%u", &compare_times);
    printf("请输入比较模式 (0x55: 单次比较, 0xAA: 多次比较): ");
    scanf("%hhx", &compare_mode);

    volatile unsigned char *reg_addr;

    // 设置比较值
    reg_addr = (volatile unsigned char *)base_addr + offsetof(Registers, compare_value);
    *(volatile unsigned int *)reg_addr = compare_value;
    printf("已设置比较值为: 0x%x\n", compare_value);

    // 设置比较次数
    reg_addr = (volatile unsigned char *)base_addr + offsetof(Registers, compare_times);
    *(volatile unsigned int *)reg_addr = compare_times;
    printf("已设置比较次数为: %u\n", compare_times);

    // 设置比较模式
    reg_addr = (volatile unsigned char *)base_addr + offsetof(Registers, compare_mode);
    *reg_addr = compare_mode;
    printf("已设置比较模式为: 0x%x\n", compare_mode);

    // 开启比较使能
    reg_addr = (volatile unsigned char *)base_addr + offsetof(Registers, compare_en);
    *reg_addr = 0x55;
    printf("已开启比较使能\n");
}

// 设置时钟参数
void set_clock_parameters(volatile Registers *base_addr)
{
    unsigned int second_value;
    printf("请输入时钟参数秒值: ");
    scanf("%u", &second_value);

    volatile unsigned char *reg_addr;

    // 设置时钟参数秒值
    reg_addr = (volatile unsigned char *)base_addr + offsetof(Registers, slv_reg2);
    *(volatile unsigned int *)reg_addr = second_value;
    printf("已设置时钟参数秒值为: %u\n", second_value);

    // 时钟参数秒设置使能
    reg_addr = (volatile unsigned char *)base_addr + offsetof(Registers, slv_reg1);

    // 先写 0xaa
    *reg_addr = 0xaa;
    printf("已向时钟参数秒设置使能写入 0xaa\n");

    // 等待 1ms
    struct timespec ts = {0, 1000000};
    nanosleep(&ts, NULL);

    // 再写 0x55
    *reg_addr = 0x55;
    printf("已向时钟参数秒设置使能写入 0x55\n");
}


void *rtc_irq_read_thrd1(void *arg)
{
    printf("compare reg coming......");
}

void rtc_irq_signal_handler(int signum) {
    if (signum == SIGIO) {
        printf("收到SIGIO信号！\n");
    }
}

// 打印所有寄存器的地址
void print_all_register_addresses(volatile Registers *base_addr)
{
    printf("寄存器 slv_reg0 地址: 0x%p\n", (void *)&base_addr->slv_reg0);
    printf("寄存器 slv_reg1 地址: 0x%p\n", (void *)&base_addr->slv_reg1);
    printf("寄存器 slv_reg2 地址: 0x%p\n", (void *)&base_addr->slv_reg2);
    printf("寄存器 slv_reg3 地址: 0x%p\n", (void *)&base_addr->slv_reg3);
    printf("寄存器 set_pps_trigger_mode 地址: 0x%p\n", (void *)&base_addr->set_pps_trigger_mode);
    printf("寄存器 clr_pps1_rx_state 地址: 0x%p\n", (void *)&base_addr->clr_pps1_rx_state);
    printf("寄存器 clr_pps2_rx_state 地址: 0x%p\n", (void *)&base_addr->clr_pps2_rx_state);
    printf("寄存器 compare_en 地址: 0x%p\n", (void *)&base_addr->compare_en);
    printf("寄存器 compare_mode 地址: 0x%p\n", (void *)&base_addr->compare_mode);
    printf("寄存器 compare_times 地址: 0x%p\n", (void *)&base_addr->compare_times);
    printf("寄存器 compare_value 地址: 0x%p\n", (void *)&base_addr->compare_value);
    printf("寄存器 current_clk_source_output 地址: 0x%p\n", (void *)&base_addr->current_clk_source_output);
    printf("寄存器 state_clk_source_output 地址: 0x%p\n", (void *)&base_addr->state_clk_source_output);
    printf("寄存器 inner_s_cnt_1_o_output 地址: 0x%p\n", (void *)&base_addr->inner_s_cnt_1_o_output);
    printf("寄存器 inner_s_cnt_2_o_output 地址: 0x%p\n", (void *)&base_addr->inner_s_cnt_2_o_output);
    printf("寄存器 second_value_output 地址: 0x%p\n", (void *)&base_addr->second_value_output);
    printf("寄存器 microsecond_value_output 地址: 0x%p\n", (void *)&base_addr->microsecond_value_output);
    printf("寄存器 pps1_rx_state_output 地址: 0x%p\n", (void *)&base_addr->pps1_rx_state_output);
    printf("寄存器 pps2_rx_state_output 地址: 0x%p\n", (void *)&base_addr->pps2_rx_state_output);
    printf("寄存器 rtc_timer_output 地址: 0x%p\n", (void *)&base_addr->rtc_timer_output);
    printf("寄存器 rtc_int_state_output 地址: 0x%p\n", (void *)&base_addr->rtc_int_state_output);
}

int main()
{
    volatile Registers *base_addr = map_registers();
    // 创建比较中断监测线程
    pthread_t compare_irq_thread;
    if (pthread_create(&compare_irq_thread, NULL, compare_interrupt_monitor, (void *)base_addr) != 0) {
        perror("创建比较中断监测线程失败");
        exit(EXIT_FAILURE);
    }
   
    // 创建PPS中断监测线程
    pthread_t pps_irq_thread;  
    if (pthread_create(&pps_irq_thread, NULL, pps_interrupt_monitor, (void *)base_addr) != 0) {
        perror("创建PPS中断监测线程失败");
        exit(EXIT_FAILURE);
    }

    int choice;
    int rtc_irq_fd = open("/dev/rtc1_irq", O_RDWR);
    if (rtc_irq_fd < 0)
    {
        printf("/dev/rtc_irq_gkhy can't open!\n");
    }

    signal(SIGIO, rtc_irq_read_thrd1);
    // signal(SIGIO, compare_interrupt_monitor);
    // signal(SIGIO, rtc_irq_signal_handler); // 信号处理函数,匹配signal函数参数类型
    // 设置进程为RTC中断设备的拥有者
    fcntl(rtc_irq_fd, F_SETOWN, getpid());
    // 获取当前文件状态标志
    int Oflags = fcntl(rtc_irq_fd, F_GETFL);
    // 设置文件状态标志为异步模式
    fcntl(rtc_irq_fd, F_SETFL, Oflags | FASYNC);


    while (1)
    {
        printf("\n主菜单:\n");
        printf("1: 外部时钟选择\n");
        printf("2: PPS输入源选择\n");
        printf("3: 比较使能\n");
        printf("4: 比较功能设置，输入比较值，比较次数，比较模式，然后开启使能\n");
        printf("5: 设置时钟参数\n");
        printf("6: 打印所有寄存器的地址\n");
        printf("7: 退出\n");
        printf("请输入你的选择: ");
        scanf("%d", &choice);

        switch (choice)
        {
        case 1:
            external_clock_selection(base_addr);
            break;
        case 2:
            pps_source_selection(base_addr);
            break;
        case 3:
            comparison_enable(base_addr);
            break;
        case 4:
            comparison_function_setup(base_addr);
            break;
        case 5:
            set_clock_parameters(base_addr);
            break;
        case 6:
            print_all_register_addresses(base_addr);
            break;
        case 7:
            // 解除内存映射并关闭文件
            if (munmap((void *)((char *)base_addr - (BASE_ADDRESS % PAGE_SIZE)), PAGE_SIZE) == -1)
            {
                perror("解除内存映射失败");
            }
            close(open("/dev/mem", O_RDWR | O_SYNC));
            return 0;
        default:
            printf("无效的选择，请重新输入。\n");
        }
    }
}