#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/of_gpio.h>

// LSIO_GPIO1_IO12对应的GPIO编号（1*32 + 12）
// 10 对应 1*32 + 10 = 42
#define GPIO_NUM 42
// 中断触发方式（上升沿触发，可根据需求修改）
#define IRQ_TRIGGER IRQF_TRIGGER_RISING

// tasklet结构体及处理函数声明
static struct tasklet_struct gpio_tasklet;

// tasklet处理函数（底半部，打印时间）
static void gpio_tasklet_handler(unsigned long data)
{
    printk("gpio_tasklet_handler\n");
    // struct timeval tv;
    // do_gettimeofday(&tv); // 获取当前时间
    // 打印时间（秒.微秒）
    // printk(KERN_INFO "GPIO中断触发时间：%ld.%06ld\n", tv.tv_sec, tv.tv_usec);
}

// 中断处理函数（顶半部，仅调度tasklet）
static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
    // 调度tasklet执行（非阻塞，立即返回）
    tasklet_schedule(&gpio_tasklet);
    return IRQ_HANDLED;
}

// 驱动初始化
static int __init gpio_irq_init(void)
{
    int irq_num;
    int ret;

    // 1. 申请GPIO
    ret = gpio_request(GPIO_NUM, "LSIO_GPIO1_IO12");
    if (ret < 0)
    {
        printk(KERN_ERR "GPIO %d 申请失败: %d\n", GPIO_NUM, ret);
        return ret;
    }

    // 2. 配置GPIO为输入
    ret = gpio_direction_input(GPIO_NUM);
    if (ret < 0)
    {
        printk(KERN_ERR "GPIO %d 配置输入失败: %d\n", GPIO_NUM, ret);
        gpio_free(GPIO_NUM);
        return ret;
    }

    // 3. 获取GPIO对应的中断号
    irq_num = gpio_to_irq(GPIO_NUM);
    if (irq_num < 0)
    {
        printk(KERN_ERR "GPIO %d 映射中断号失败: %d\n", GPIO_NUM, irq_num);
        gpio_free(GPIO_NUM);
        return irq_num;
    }
    printk(KERN_INFO "GPIO %d 对应的中断号: %d\n", GPIO_NUM, irq_num);

    // 4. 初始化tasklet
    tasklet_init(&gpio_tasklet, gpio_tasklet_handler, 0);

    // 5. 注册中断
    ret = request_irq(irq_num,
                      gpio_irq_handler,
                      IRQ_TRIGGER,
                      "gpio1_io12_irq",
                      NULL);
    if (ret < 0)
    {
        printk(KERN_ERR "中断注册失败: %d\n", ret);
        tasklet_kill(&gpio_tasklet);
        gpio_free(GPIO_NUM);
        return ret;
    }

    printk(KERN_INFO "GPIO中断驱动初始化完成\n");
    return 0;
}

// 驱动退出
static void __exit gpio_irq_exit(void)
{
    int irq_num = gpio_to_irq(GPIO_NUM);
    // 释放资源（与初始化顺序相反）
    free_irq(irq_num, NULL);     // 释放中断
    tasklet_kill(&gpio_tasklet); // 销毁tasklet
    gpio_free(GPIO_NUM);         // 释放GPIO
    printk(KERN_INFO "GPIO中断驱动退出\n");
}

module_init(gpio_irq_init);
module_exit(gpio_irq_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LSIO_GPIO1_IO12中断监听驱动（tasklet机制）");