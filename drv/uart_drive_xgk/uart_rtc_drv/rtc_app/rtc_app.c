#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <inttypes.h>

// 驱动中定义的IOCTL命令（需与驱动保持一致）
#define RTC_IOCTL_MAGIC 'a'

// 写操作命令
#define IOCTL_SET_CLK_SOURCE      _IOW(RTC_IOCTL_MAGIC, 0, uint8_t)
#define IOCTL_SET_SECOND_EN       _IO(RTC_IOCTL_MAGIC, 1)
#define IOCTL_SET_SECOND_VALUE    _IOW(RTC_IOCTL_MAGIC, 2, uint64_t)
#define IOCTL_SET_PPS_SOURCE      _IOW(RTC_IOCTL_MAGIC, 3, uint8_t)
#define IOCTL_SET_PPS_TRIGGER     _IOW(RTC_IOCTL_MAGIC, 4, uint8_t)
#define IOCTL_CLR_PPS1_STATE      _IO(RTC_IOCTL_MAGIC, 5)
#define IOCTL_CLR_PPS2_STATE      _IO(RTC_IOCTL_MAGIC, 6)
#define IOCTL_SET_COMPARE_EN      _IO(RTC_IOCTL_MAGIC, 7)
#define IOCTL_SET_COMPARE_MODE    _IOW(RTC_IOCTL_MAGIC, 8, uint8_t)
#define IOCTL_SET_COMPARE_TIMES   _IOW(RTC_IOCTL_MAGIC, 9, uint64_t)
#define IOCTL_SET_COMPARE_VALUE   _IOW(RTC_IOCTL_MAGIC, 10, uint64_t)

// 读操作命令
#define IOCTL_GET_CLK_SOURCE      _IOR(RTC_IOCTL_MAGIC, 11, uint8_t)
#define IOCTL_GET_EXT_CLK_STATE   _IOR(RTC_IOCTL_MAGIC, 12, uint8_t)
#define IOCTL_GET_SECOND_VALUE    _IOR(RTC_IOCTL_MAGIC, 13, uint64_t)
#define IOCTL_GET_MICRO_VALUE     _IOR(RTC_IOCTL_MAGIC, 14, uint64_t)
#define IOCTL_GET_PPS1_STATE      _IOR(RTC_IOCTL_MAGIC, 15, uint8_t)
#define IOCTL_GET_PPS2_STATE      _IOR(RTC_IOCTL_MAGIC, 16, uint8_t)
#define IOCTL_GET_RTC_TIMER       _IOR(RTC_IOCTL_MAGIC, 17, uint64_t)
#define IOCTL_GET_DEBUG_SECOND    _IOR(RTC_IOCTL_MAGIC, 18, uint64_t)
#define IOCTL_GET_DEBUG_MICRO     _IOR(RTC_IOCTL_MAGIC, 19, uint64_t)
#define IOCTL_GET_DEBUG_RTC       _IOR(RTC_IOCTL_MAGIC, 20, uint64_t)
// 新增中断相关IOCTL命令
#define IOCTL_GET_IRQ_STATUS      _IOR(RTC_IOCTL_MAGIC, 21, uint8_t)  // 读取中断状态
#define IOCTL_SET_IRQ_MASK        _IOW(RTC_IOCTL_MAGIC, 22, uint8_t)  // 设置中断屏蔽
#define IOCTL_CLEAR_IRQ           _IOW(RTC_IOCTL_MAGIC, 23, uint8_t)  // 清除中断
#define IOCTL_COMPARE_CNT          _IOW(RTC_IOCTL_MAGIC, 24, uint8_t)  // 读取两个pps秒脉冲之间比较中断的计数值，基地址+0xb0
#define IOCTL_GET_PPS_STATE          _IOW(RTC_IOCTL_MAGIC, 25, uint8_t) //获取pps状态，包括0x11 0x22 0x33
// 设备节点路径（与驱动中创建的一致）
#define RTC_DEV_PATH "/dev/rtc_dev"
//统计比较中断次数所需变量
static uint64_t current_second_counting = 0;  // 正在统计的秒（Unix时间戳）
static int compare_ordinal = 0;               // 当前秒内比较中断的序号（第几次）
int fd; // 全局文件描述符，供信号处理函数使用
void print_hex64(uint64_t val, const char *name);
void print_hex(uint8_t val, const char *name);
// 信号处理函数：响应中断产生的异步通知
static void catch_sigio(int signum) {
    uint8_t pps1_state;
    uint64_t now_micsecond=0;
    uint64_t now_second;
    int ret;
    uint8_t irq_status;  // 新增：用于存储中断状态
    uint8_t count;  // 新增：用于保存比较中断次数
    uint8_t pps_state;  // 新增：用于保存pss源状态
    if (signum != SIGIO) {
        printf("收到未知信号: %d\n", signum);
        return;
    }

    printf("\n===== 收到中断异步通知 =====\n");

        // 8.15新增 读取当前秒值
    ret = ioctl(fd, IOCTL_GET_SECOND_VALUE, &now_second);
    if (ret < 0) {
        perror("IOCTL_GET_SECOND_VALUE failed");
        // 如果失败，使用上次的秒（避免误判）
        now_second = current_second_counting;
    } else {
        //print_hex64(now_second, "Current second value");
       // printf("(Unix timestamp: %" PRIu64 ")\n", now_second);  // 使用PRIu64宏
        printf("Current second value %" PRIu64 "\n", now_second);
    }
        // 8.15新增 读取当前微秒值
    ret = ioctl(fd, IOCTL_GET_MICRO_VALUE, &now_micsecond);
    if (ret < 0) {
        perror("IOCTL_GET_MICRO_VALUE failed");
    } else {
       // printf("Current microsecond value: %llu\n", (unsigned long long)now_micsecond);
       printf("Timestamp: %" PRIu64 ".%06llu\n", now_second, (unsigned long long)now_micsecond);
    }
    printf("\n");


  // 读取pps状态
    ret = ioctl(fd, IOCTL_GET_PPS_STATE, &pps_state);
    if (ret < 0) {
        perror("IOCTL_GET_PPS_STATE failed");
    } else {
        printf("PPS state: 0x%02X\n", pps_state);
    }
    printf("\n");
    // 新增：读取中断状态
    ret = ioctl(fd, IOCTL_GET_IRQ_STATUS, &irq_status);
    if (ret < 0) {
        perror("IOCTL_GET_IRQ_STATUS failed");
    } else {
        printf("中断状态: %x\n", irq_status);
        // 判断中断类型
        if (irq_status & 0x01) {
            printf("  - PPS秒脉冲中断触发!\n");
        }
        if (irq_status & 0x02) {
            printf("  - 比较中断触发!\n");
            ret = ioctl(fd, IOCTL_COMPARE_CNT, &count);
                if (ret < 0) {
                        perror("IOCTL_COMPARE_CNT failed");
                    } else {
                        printf("  - (这是第 %" PRIu64 " 秒内的第 %d 次)\n",
                            now_second, count);
                    }

        }

    }

       // 清除中断
    ret = ioctl(fd, IOCTL_CLEAR_IRQ, &irq_status);
    if (ret < 0) {
        perror("IOCTL_CLEAR_IRQ failed");
    } else {
        printf("中断已清除\n");
    }

    printf("==========================\n");
}

// 打印二进制数据（辅助调试）
void print_hex(uint8_t val, const char *name) {
    printf("%s: 0x%02X\n", name, val);
}

void print_hex64(uint64_t val, const char *name) {
    printf("%s: 0x%016lX\n", name, val);
}

int main() {
    int ret;
    uint8_t val8;
    uint64_t val64;
    int flags;
    uint8_t irq_status;
    current_second_counting = 0;
    compare_ordinal = 0;
    // 打开设备
    fd = open(RTC_DEV_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open RTC device");
        return -1;
    }
    printf("RTC device opened successfully (fd=%d)\n\n", fd);

    // -------------------------- 设置异步通知 --------------------------
    // 注册SIGIO信号处理函数
    if (signal(SIGIO, catch_sigio) == SIG_ERR) {
        perror("signal failed");
        close(fd);
        return -1;
    }

    // 设置接收SIGIO信号的进程ID
    if (fcntl(fd, F_SETOWN, getpid()) == -1) {
        perror("fcntl F_SETOWN failed");
        close(fd);
        return -1;
    }

    // 启用异步通知
    flags = fcntl(fd, F_GETFL);
    if (fcntl(fd, F_SETFL, flags | O_ASYNC) == -1) {
        perror("fcntl F_SETFL O_ASYNC failed");
        close(fd);
        return -1;
    }
    printf("异步通知已启用，等待中断事件...\n\n");

    // -------------------------- 测试：时钟源配置 --------------------------
    printf("=== Testing Clock Source Configuration ===\n");
    // 1. 设置外部时钟1（0x11）
    val8 = 0x11;
    ret = ioctl(fd, IOCTL_SET_CLK_SOURCE, &val8);
    if (ret < 0) {
        perror("IOCTL_SET_CLK_SOURCE failed");
    } else {
        printf("Set clock source to external clock 1 (0x11)\n");
    }

    // 2. 读取当前时钟源
    ret = ioctl(fd, IOCTL_GET_CLK_SOURCE, &val8);
    if (ret < 0) {
        perror("IOCTL_GET_CLK_SOURCE failed");
    } else {
        print_hex(val8, "Current clock source");
        switch (val8) {
            case 0x11: printf("(External clock 1)\n"); break;
            case 0x22: printf("(External clock 2)\n"); break;
            case 0x33: printf("(Internal reference clock)\n"); break;
            default: printf("(Unknown)\n");
        }
    }

    // 3. 读取外部时钟状态
    ret = ioctl(fd, IOCTL_GET_EXT_CLK_STATE, &val8);
    if (ret < 0) {
        perror("IOCTL_GET_EXT_CLK_STATE failed");
    } else {
        print_hex(val8, "External clock state");
        uint8_t clk1 = (val8 >> 4) & 0x0F;
        uint8_t clk2 = val8 & 0x0F;
        printf("Clock 1 state: %s\n", (clk1 == 0x05) ? "Locked" : "Unlocked");
        printf("Clock 2 state: %s\n", (clk2 == 0x05) ? "Locked" : "Unlocked");
    }
    printf("\n");

    // -------------------------- 测试：时间设置与读取 --------------------------
    printf("=== Testing Time Configuration ===\n");
    // 1. 设置秒值（例如：1620000000 对应2021-05-03 00:00:00）
    val64 = 1620000000ULL;
    ret = ioctl(fd, IOCTL_SET_SECOND_VALUE, &val64);
    if (ret < 0) {
        perror("IOCTL_SET_SECOND_VALUE failed");
    } else {
        print_hex64(val64, "Set second value");
    }

    // 2. 使能秒设置（驱动内部执行0x55→0xAA序列）
    ret = ioctl(fd, IOCTL_SET_SECOND_EN);
    if (ret < 0) {
        perror("IOCTL_SET_SECOND_EN failed");
    } else {
        printf("Second setting enabled\n");
    }

    // 3. 读取当前秒值
    ret = ioctl(fd, IOCTL_GET_SECOND_VALUE, &val64);
    if (ret < 0) {
        perror("IOCTL_GET_SECOND_VALUE failed");
    } else {
        print_hex64(val64, "Current second value");
        printf("(Unix timestamp: %" PRIu64 ")\n", val64);  // 使用PRIu64宏
    }

    // 4. 读取当前微秒值
    ret = ioctl(fd, IOCTL_GET_MICRO_VALUE, &val64);
    if (ret < 0) {
        perror("IOCTL_GET_MICRO_VALUE failed");
    } else {
        print_hex64(val64, "Current microsecond value");
    }
    printf("\n");

    // -------------------------- 测试：PPS配置 --------------------------
    printf("=== Testing PPS Configuration ===\n");
    // 1. 设置PPS输入源为输入1（0x11）
   // val8 = 0x11;
    //ret = ioctl(fd, IOCTL_SET_PPS_SOURCE, &val8);
   // if (ret < 0) {
   //     perror("IOCTL_SET_PPS_SOURCE failed");
   // } else {
   //     printf("Set PPS source to input 1 (0x11)\n");
  //  }

    // 2. 设置PPS触发模式为上升沿（0x55）
    val8 = 0x55;
    ret = ioctl(fd, IOCTL_SET_PPS_TRIGGER, &val8);
    if (ret < 0) {
        perror("IOCTL_SET_PPS_TRIGGER failed");
    } else {
        printf("Set PPS trigger mode to rising edge (0x55)\n");
    }

    // 3. 清除PPS1接收状态
   // ret = ioctl(fd, IOCTL_CLR_PPS1_STATE);
   // if (ret < 0) {
   //     perror("IOCTL_CLR_PPS1_STATE failed");
   // } else {
   //     printf("PPS1 state cleared\n");
   // }

    // 4. 读取PPS1状态
    ret = ioctl(fd, IOCTL_GET_PPS1_STATE, &val8);
    if (ret < 0) {
        perror("IOCTL_GET_PPS1_STATE failed");
    } else {
        print_hex(val8, "PPS1 state");
        printf("(State: %s)\n", 
               (val8 == 0x55) ? "Normal" : 
               (val8 == 0xAA) ? "Cleared" : "Error");
    }
    printf("\n");

    // -------------------------- 测试：比较器配置 --------------------------
    printf("=== Testing Comparator Configuration ===\n");

    // 2. 设置比较值为1000000us（1秒）
    //8月15日 改为100ms
    val64 = 100000ULL;
    ret = ioctl(fd, IOCTL_SET_COMPARE_VALUE, &val64);
    if (ret < 0) {
        perror("IOCTL_SET_COMPARE_VALUE failed");
    } else {
        print_hex64(val64, "Set compare value (us)");
    }

    // 4. 设置比较模式为多次比较（0xAA）
  //  val8 = 0xAA;//启动
  //  ret = ioctl(fd, IOCTL_SET_COMPARE_MODE, &val8);
  //  if (ret < 0) {
   //     perror("IOCTL_SET_COMPARE_MODE failed");
  //  } else {
  //      printf("Set compare mode to multiple (0xAA)\n");
 //   }
    

   // val8 = 0x55;//停止
   // ret = ioctl(fd, IOCTL_SET_COMPARE_MODE, &val8);
   // if (ret < 0) {
   //     perror("IOCTL_SET_COMPARE_MODE failed");
  //  } else {
  //      printf("Set compare mode to multiple (0x55)\n");
  //  }
    printf("\n");


// -------------------------- 测试：中断屏蔽控制 --------------------------
printf("=== Testing Interrupt Mask Control ===\n");

// 1. 读取当前中断状态
ret = ioctl(fd, IOCTL_GET_IRQ_STATUS, &irq_status);
if (ret < 0) {
    perror("IOCTL_GET_IRQ_STATUS failed");
} else {
    printf("当前中断状态: 0x%02X\n", irq_status);
}

// 2. 屏蔽PPS秒脉冲中断（设置bit 2）
val8 = 0x04;  // 0b0100
ret = ioctl(fd, IOCTL_SET_IRQ_MASK, &val8);
if (ret < 0) {
    perror("IOCTL_SET_IRQ_MASK failed");
} else {
    printf("已屏蔽PPS秒脉冲中断 (0x04)\n");
}

// 3. 屏蔽比较中断（设置bit 3）
val8 = 0x08;  // 0b1000
ret = ioctl(fd, IOCTL_SET_IRQ_MASK, &val8);
if (ret < 0) {
    perror("IOCTL_SET_IRQ_MASK failed");
} else {
    printf("已屏蔽比较中断 (0x08)\n");
}

// 4. 同时屏蔽PPS秒脉冲中断和比较中断（设置bit 2和bit 3）
val8 = 0x0C;  // 0b1100
ret = ioctl(fd, IOCTL_SET_IRQ_MASK, &val8);
if (ret < 0) {
    perror("IOCTL_SET_IRQ_MASK failed");
} else {
    printf("已同时屏蔽PPS秒脉冲中断和比较中断 (0x0C)\n");
}

// 5. 不屏蔽任何中断（清除bit 2和bit 3）
val8 = 0x00;
ret = ioctl(fd, IOCTL_SET_IRQ_MASK, &val8);
if (ret < 0) {
    perror("IOCTL_SET_IRQ_MASK failed");
} else {
    printf("已清除所有中断屏蔽 (0x00)\n");
}
printf("\n");

// -------------------------- 测试：调试信息读取 --------------------------
    printf("=== Testing Debug Information ===\n");
    // 1. 读取调试用秒值锁存
    ret = ioctl(fd, IOCTL_GET_DEBUG_SECOND, &val64);
    if (ret < 0) {
        perror("IOCTL_GET_DEBUG_SECOND failed");
    } else {
        print_hex64(val64, "Debug second value (latched)");
    }

    // 2. 读取调试用微秒值锁存
    ret = ioctl(fd, IOCTL_GET_DEBUG_MICRO, &val64);
    if (ret < 0) {
        perror("IOCTL_GET_DEBUG_MICRO failed");
    } else {
        print_hex64(val64, "Debug microsecond value (latched)");
    }

    // 3. 读取RTC计时器
    ret = ioctl(fd, IOCTL_GET_RTC_TIMER, &val64);
    if (ret < 0) {
        perror("IOCTL_GET_RTC_TIMER failed");
    } else {
        print_hex64(val64, "RTC timer value");
    }



    // -------------------------- 等待中断事件 --------------------------
    printf("=== 等待中断事件（按Ctrl+C退出） ===\n");
    while (1) {
        pause(); // 等待信号
    }

    // 关闭设备（实际不会执行到这里，因为上面是无限循环）
    close(fd);
    printf("\nRTC device closed\n");
    return 0;
}
