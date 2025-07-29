#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

// 寄存器物理地址定义
#define AXI_GPIO_RTC0_ADDR 0x41230000
#define RTC_BASE_ADDR 0x62000000
#define REG1_OFFSET 0x02 // 秒计数低16位
#define REG2_OFFSET 0x04 // 秒计数高16位

#define REG13_OFFSET 0x1A // PPS中断使能寄存器
#define REG14_OFFSET 0x1C // PPS中断清除寄存器

#define REG16_OFFSET 0x20 // 中断响应寄存器
#define REG17_OFFSET 0x22 // REG-17 偏移量（假设每寄存器占4字节，依次递增）
#define REG18_OFFSET 0x24 // REG-18
#define REG19_OFFSET 0x26 // REG-19
#define REG20_OFFSET 0x28 // REG-20
#define REG21_OFFSET 0x2A // REG-21
#define REG22_OFFSET 0x2C // REG-22
#define REG25_OFFSET 0x32 // REG-25
#define REG26_OFFSET 0x34 // REG-26

#define REG33_OFFSET 0x42 // 时钟源选择寄存器

#define REG35_OFFSET 0x46 // 设置秒使能

#define REG36_OFFSET 0x48 // PPS输入源选择寄存器

#define REG64_OFFSET 0x80 // 当前时钟源状态寄存器

#define INTERRUPT_ENABLE_BIT (1 << 1)
#define INTERRUPT_FLAG_BIT (1 << 0)
#define INTERRUPT_CLEAR_BITS (INTERRUPT_FLAG_BIT | INTERRUPT_ENABLE_BIT)
void *map_memory(unsigned long phys_addr);
// 映射后的内存指针
void *axi_gpio_rtc0_map;
void *reg1_map;
void *reg2_map;
void *reg13_map;
void *reg14_map;
void *reg16_map;
void *reg17_map; // 新增寄存器指针
void *reg18_map;
void *reg19_map;
void *reg20_map;
void *reg21_map;
void *reg22_map;
void *reg25_map;
void *reg26_map;
void *reg33_map;
void *reg35_map;
void *reg36_map;
void *reg64_map;

// 中断处理线程
void *interrupt_monitor_thread(void *arg)
{
    // 使能中断位
    *(volatile unsigned int *)reg13_map |= INTERRUPT_ENABLE_BIT;
    LOG("中断使能位已设置\n");

    while (1)
    {
        // 检测中断响应位
        unsigned int status = *(volatile unsigned int *)reg14_map;
        if (status & INTERRUPT_FLAG_BIT)
        {
            LOG("检测到中断到来(状态码: 0x%08X)\n", status);

            // 清除中断(设置对应清除位)
            *(volatile unsigned int *)reg14_map |= INTERRUPT_CLEAR_BITS;
            LOG("中断已清除\n");

             // 检查中断响应寄存器
             unsigned int interrupt_response_status = *(volatile unsigned int *)reg16_map;
             if (interrupt_response_status & INTERRUPT_FLAG_BIT)
             {
                 LOG("中断响应寄存器正确响应\n");
             }
             else
             {
                 LOG("中断响应寄存器未正确响应\n");
             }
        }
        usleep(100); // 避免忙等待,降低CPU占用
    }
    return NULL;
}

// 初始化内存映射
void init_memory_mapping()
{
    axi_gpio_rtc0_map = map_memory(AXI_GPIO_RTC0_ADDR);
    reg1_map = map_memory(RTC_BASE_ADDR + REG1_OFFSET);
    reg2_map = map_memory(RTC_BASE_ADDR + REG2_OFFSET);
    reg13_map = map_memory(RTC_BASE_ADDR + REG13_OFFSET);
    reg14_map = map_memory(RTC_BASE_ADDR + REG14_OFFSET);
    reg16_map = map_memory(RTC_BASE_ADDR + REG16_OFFSET);
    reg17_map = map_memory(RTC_BASE_ADDR + REG17_OFFSET); // 新增映射
    reg18_map = map_memory(RTC_BASE_ADDR + REG18_OFFSET);
    reg19_map = map_memory(RTC_BASE_ADDR + REG19_OFFSET);
    reg20_map = map_memory(RTC_BASE_ADDR + REG20_OFFSET);
    reg21_map = map_memory(RTC_BASE_ADDR + REG21_OFFSET);
    reg22_map = map_memory(RTC_BASE_ADDR + REG22_OFFSET);
    reg25_map = map_memory(RTC_BASE_ADDR + REG25_OFFSET);
    reg26_map = map_memory(RTC_BASE_ADDR + REG26_OFFSET);
    reg33_map = map_memory(RTC_BASE_ADDR + REG33_OFFSET);
    reg35_map = map_memory(RTC_BASE_ADDR + REG35_OFFSET);
    reg36_map = map_memory(RTC_BASE_ADDR + REG36_OFFSET); 
    reg64_map = map_memory(RTC_BASE_ADDR + REG64_OFFSET);
}

// 解除内存映射
void uninit_memory_mapping()
{
    munmap(axi_gpio_rtc0_map, MAP_SIZE);
    munmap(reg1_map, MAP_SIZE);
    munmap(reg2_map, MAP_SIZE);
    munmap(reg13_map, MAP_SIZE);
    munmap(reg14_map, MAP_SIZE);
    munmap(reg16_map, MAP_SIZE);
    munmap(reg17_map, MAP_SIZE);
    munmap(reg18_map, MAP_SIZE);
    munmap(reg19_map, MAP_SIZE);
    munmap(reg20_map, MAP_SIZE);
    munmap(reg21_map, MAP_SIZE);
    munmap(reg22_map, MAP_SIZE);
    munmap(reg25_map, MAP_SIZE);
    munmap(reg26_map, MAP_SIZE);
    munmap(reg33_map, MAP_SIZE);
    munmap(reg35_map, MAP_SIZE);
    munmap(reg36_map, MAP_SIZE); 
    munmap(reg64_map, MAP_SIZE);
}

// 映射内存函数
void *map_memory(unsigned long phys_addr)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC,0666);
    if (fd < 0)
    {
        perror("无法打开 /dev/mem");
        exit(EXIT_FAILURE);
    }
    void *map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phys_addr & ~MAP_MASK);
    close(fd);
    if (map_base == MAP_FAILED)
    {
        perror("内存映射失败");
        exit(EXIT_FAILURE);
    }
    return (char *)map_base + (phys_addr & MAP_MASK);
}

// 本地校时测试
void test_local_time_calibration()
{
    LOG("\n=== 本地校时测试 ===\n");
    int choice;
    LOG("请输入秒值\r");
    if (scanf("%d", &choice) != 1)
    {
        LOG("错误:请输入有效数字！\n");
        while (getchar() != '\n')
            ; // 清空输入缓冲区
        return;
    }
    // 设置不复位RTC0
    *(volatile unsigned int *)axi_gpio_rtc0_map = 1;

    // 设置本地校时时间(假设当前秒计数为300秒)
    unsigned short local_seconds_low = choice & 0xFFFF;
    unsigned short local_seconds_high = (choice >> 16) & 0xFFFF;
    *(volatile unsigned short *)reg1_map = local_seconds_low;
    *(volatile unsigned short *)reg2_map = local_seconds_high;
    LOG("设置本地校时时间:%d秒\n", local_seconds_high * 0x10000 + local_seconds_low);
    // 设置秒使能
    *(volatile unsigned int *)reg35_map = 0x55;
    LOG("秒使能已设置\n");
    // 验证校时结果(简化示例,实际需读取状态寄存器)
    unsigned short read_low = *(volatile unsigned short *)reg1_map;
    unsigned short read_high = *(volatile unsigned short *)reg2_map;
    unsigned int current_seconds = read_high * 0x10000 + read_low;
    LOG("校时后当前秒计数:%d\n", current_seconds);
}

// 比较校时测试
void test_compare_time_calibration()
{
    
    LOG("\n=== 比较校时测试 ===\n");
    // 设置不复位RTC0
    *(volatile unsigned int *)axi_gpio_rtc0_map = 1;

    int compare_value;
    LOG("请输入比较值\r");
    if (scanf("%d", &compare_value) != 1)
    {
        LOG("错误:请输入有效数字！\n");
        while (getchar() != '\n')
            ; // 清空输入缓冲区
        return;
    }

    // 设置比较寄存器
    unsigned short compare_low = compare_value & 0xFFFF;
    unsigned short compare_high = (compare_value >> 16) & 0xFFFF;
    *(volatile unsigned short *)reg25_map = compare_low;
    *(volatile unsigned short *)reg26_map = compare_high;
    LOG("设置比较寄存器值:%d\n", compare_high * 0x10000 + compare_low);

    // 切换到外部时钟源比较模式(假设外部时钟源为0x11)
    *(volatile unsigned char *)reg33_map = 0x11;
    LOG("切换到外部时钟源进行时间比较\n");

    // 清除原有中断并使能比较中断
    *(volatile unsigned int *)reg14_map |= (1 << 1); // 清除中断
    *(volatile unsigned int *)reg13_map |= (1 << 1); // 使能中断

    // 模拟比较校时过程(实际需根据硬件协议处理)
    LOG("等待外部时钟源同步...(模拟成功)\n");
    unsigned char current_source = *(volatile unsigned char *)reg64_map;
    if (current_source == 0x11)
    {
        LOG("比较校时成功,当前使用外部时钟源1\n");
    }
    else
    {
        LOG("比较校时失败,当前时钟源:0x%02X\n", current_source);
    }
}

// 选择时钟源
void set_clock_source()
{
    int choice;
    while (1)
    {
        print_clock_source_menu();
        if (scanf("%d", &choice) != 1)
        {
            LOG("错误:请输入有效数字！\n");
            while (getchar() != '\n')
                ; // 清空输入缓冲区
            continue;
        }

        switch (choice)
        {
        case 1:
            set_external_clock();
            break;
        case 2:
            set_pps_input();
            break;
        case 3:
            set_current_clock();
            break;
        default:
            LOG("错误:无效指令,请重新输入\n");
        }
        break; // 返回上一级菜单
    }
}

// 外部时钟源设置
void set_external_clock()
{
    int choice;
    while (1)
    {
        print_external_clock_menu();
        if (scanf("%d", &choice) != 1)
        {
            LOG("错误:请输入有效数字！\n");
            while (getchar() != '\n')
                ; // 清空输入缓冲区
            continue;
        }

        switch (choice)
        {
        case 1:
            *(volatile unsigned char *)reg33_map = 0x11;
            LOG("外部时钟源已设置为1\n");
            break; 
        case 2:
            *(volatile unsigned char *)reg33_map = 0x22;
            LOG("外部时钟源已设置为2\n");
            break; 
        default:
            LOG("错误:无效指令,请重新输入\n");
        }
        break; // 返回上一级菜单
    }
}

// PPS 输入源设置
void set_pps_input()
{
    int choice;
    while (1)
    {
        print_pps_input_menu();
        if (scanf("%d", &choice) != 1)
        {
            LOG("错误:请输入有效数字！\n");
            while (getchar() != '\n')
                ; // 清空输入缓冲区
            continue;
        }

        switch (choice)
        {
        case 1:
            *(volatile unsigned char *)reg36_map = 0x11;
            LOG("PPS 输入源已设置为1\n");
            break; 
        case 2:
            *(volatile unsigned char *)reg36_map = 0x22;
            LOG("PPS 输入源已设置为2\n");
            break; 
        default:
            LOG("错误:无效指令,请重新输入\n");
        }
        break; // 返回上一级菜单
    }
}

// RTC 模块当前使用时钟源设置
void set_current_clock()
{
    int choice;
    while (1)
    {
        print_current_clock_menu();
        if (scanf("%d", &choice) != 1)
        {
            LOG("错误:请输入有效数字！\n");
            while (getchar() != '\n')
                ; // 清空输入缓冲区
            continue;
        }

        switch (choice)
        {
        case 1:
            *(volatile unsigned char *)reg64_map = 0x11;
            LOG("RTC 模块当前使用时钟源已设置为外部时钟1\n");
            break; 
        case 2:
            *(volatile unsigned char *)reg64_map = 0x22;
            LOG("RTC 模块当前使用时钟源已设置为外部时钟2\n");
            break; 
        case 3:
            *(volatile unsigned char *)reg64_map = 0x33;
            LOG("RTC 模块当前使用时钟源已设置为内部时钟\n");
            break; 
        default:
            LOG("错误:无效指令,请重新输入\n");
        }
        break; // 返回上一级菜单
    }
}

// 新增寄存器值打印函数
void print_register_values()
{
    LOG("\n=== 寄存器值 ===\n");
    LOG("REG-17 (0x%08X): 0x%08X (%d)\n",
           RTC_BASE_ADDR + REG17_OFFSET,
           *(volatile unsigned int *)reg17_map,
           *(volatile unsigned int *)reg17_map);
    LOG("REG-18 (0x%08X): 0x%08X (%d)\n",
           RTC_BASE_ADDR + REG18_OFFSET,
           *(volatile unsigned int *)reg18_map,
           *(volatile unsigned int *)reg18_map);
    LOG("REG-19 (0x%08X): 0x%08X (%d)\n",
           RTC_BASE_ADDR + REG19_OFFSET,
           *(volatile unsigned int *)reg19_map,
           *(volatile unsigned int *)reg19_map);
    LOG("REG-20 (0x%08X): 0x%08X (%d)\n",
           RTC_BASE_ADDR + REG20_OFFSET,
           *(volatile unsigned int *)reg20_map,
           *(volatile unsigned int *)reg20_map);
    LOG("REG-21 (0x%08X): 0x%08X (%d)\n",
           RTC_BASE_ADDR + REG21_OFFSET,
           *(volatile unsigned int *)reg21_map,
           *(volatile unsigned int *)reg21_map);
    LOG("REG-22 (0x%08X): 0x%08X (%d)\n",
           RTC_BASE_ADDR + REG22_OFFSET,
           *(volatile unsigned int *)reg22_map,
           *(volatile unsigned int *)reg22_map);
    LOG("REG-25 (0x%08X): 0x%08X (%d)\n",
           RTC_BASE_ADDR + REG25_OFFSET,
           *(volatile unsigned int *)reg25_map,
           *(volatile unsigned int *)reg25_map);
    LOG("REG-26 (0x%08X): 0x%08X (%d)\n",
           RTC_BASE_ADDR + REG26_OFFSET,
           *(volatile unsigned int *)reg26_map,
           *(volatile unsigned int *)reg26_map);
}

// 打印菜单
void print_menu()
{
    LOG("\n=== 校时功能测试菜单 ===\n");
    LOG("1. 本地校时测试\n");
    LOG("2. 比较校时测试\n");
    LOG("3. 时钟源设置\n");
    LOG("4. 打印REG-17/18/19/20/21/22/25/26寄存器值\n"); // 新增选项
    LOG("5. 退出程序\n");
    LOG("请输入指令（1-5）：");
}

// 打印时钟源设置菜单
void print_clock_source_menu()
{
    LOG("\n=== 时钟源设置菜单 ===\n");
    LOG("1. 设置外部时钟源\n");
    LOG("2. 设置 PPS 输入源\n");
    LOG("3. 设置 RTC 模块当前使用时钟源\n");
    LOG("请输入指令（1-3）：");
}

// 打印外部时钟源设置菜单
void print_external_clock_menu()
{
    LOG("\n=== 外部时钟源设置 ===\n");
    LOG("1. 设置为外部时钟源1\n");
    LOG("2. 设置为外部时钟源2\n");
    LOG("请输入指令（1-2）：");
}

// 打印 PPS 输入源设置菜单
void print_pps_input_menu()
{
    LOG("\n=== PPS 输入源设置 ===\n");
    LOG("1. 设置为 PPS 输入1\n");
    LOG("2. 设置为 PPS 输入2\n");
    LOG("请输入指令（1-2）：");
}

// 打印 RTC 模块当前使用时钟源设置菜单
void print_current_clock_menu()
{
    LOG("\n=== RTC 模块当前使用时钟源设置 ===\n");
    LOG("1. 设置为外部时钟1\n");
    LOG("2. 设置为外部时钟2\n");
    LOG("3. 设置为内部时钟\n");
    LOG("请输入指令（1-3）：");
}

int main()
{
    pthread_t interrupt_thread_id;
    init_memory_mapping();
    // 创建中断监测线程
    if (pthread_create(&interrupt_thread_id, NULL, interrupt_monitor_thread, NULL) != 0)
    {
        perror("创建线程失败");
        exit(EXIT_FAILURE);
    }

    int choice;
    while (1)
    {
        print_menu();
        if (scanf("%d", &choice) != 1)
        {
            LOG("错误:请输入有效数字！\n");
            while (getchar() != '\n')
                ; // 清空输入缓冲区
            continue;
        }

        switch (choice)
        {
        case 1:
            test_local_time_calibration();
            break;
        case 2:
            test_compare_time_calibration();
            break;
        case 3: // 设置时钟源
            set_clock_source();
            break;
        case 4: // 打印寄存器值
            print_register_values();
            break;
        case 5: 
            LOG("退出程序\n");
            // 这里可添加线程终止逻辑(实际需根据硬件设计处理)
            pthread_exit(NULL);
            goto EXIT;
        default:
            LOG("错误:无效指令,请重新输入\n");
        }
    }

EXIT:
    uninit_memory_mapping();
    return 0;
}