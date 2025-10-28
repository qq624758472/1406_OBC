#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/fcntl.h>

// 寄存器物理基地址（文档定义）
#define RTC_BASE_PHYS 0x72300000
// 新增中断相关寄存器物理基地址
#define IRQ_STATUS_BASE_PHYS 0x72250000  // 中断状态寄存器基地址
#define IRQ_CTRL_BASE_PHYS   0x72260000  // 中断控制寄存器基地址
// 寄存器地址范围（覆盖所有偏移量，最大偏移0xAC）
#define RTC_REG_SIZE 0x100
// 添加中断寄存器地址范围定义
#define IRQ_REG_SIZE 0x100

#define PCI_VENDOR_ID_EXAMPLE 0x11aa
#define PCI_DEVICE_ID_EXAMPLE 0x11aa

// 设备结构体
struct rtc_dev {
    dev_t dev_num;          // 设备号
    struct cdev cdev;       // cdev结构体
    struct class *class;    // 设备类
    struct device *device;  // 设备实例
    void __iomem *reg_base; // RTC寄存器虚拟地址映射
    void __iomem *irq_status_base; // 中断状态寄存器虚拟地址映射
    void __iomem *irq_ctrl_base;   // 中断控制寄存器虚拟地址映射
    struct mutex lock;      // 互斥锁，保护寄存器并发访问
    struct fasync_struct *async_queue; // 异步通知队列

    // 新增字段：控制中断是否启用
    bool irq_enabled;
};

static struct rtc_dev *rtc_device;

// IOCTL命令定义（命令码0xAE，避免冲突）
#define RTC_IOCTL_MAGIC 'a'

// 软件输出寄存器 - 写操作命令
#define IOCTL_SET_CLK_SOURCE      _IOW(RTC_IOCTL_MAGIC, 0, uint8_t)   // 基地址+0x00
#define IOCTL_SET_SECOND_EN       _IO(RTC_IOCTL_MAGIC, 1)             // 基地址+0x04（特殊顺序）
#define IOCTL_SET_SECOND_VALUE    _IOW(RTC_IOCTL_MAGIC, 2, uint64_t)  // 基地址+0x08/0x0C（64位）
#define IOCTL_SET_PPS_SOURCE      _IOW(RTC_IOCTL_MAGIC, 3, uint8_t)   // 基地址+0x10
#define IOCTL_SET_PPS_TRIGGER     _IOW(RTC_IOCTL_MAGIC, 4, uint8_t)   // 基地址+0x14
#define IOCTL_CLR_PPS1_STATE      _IO(RTC_IOCTL_MAGIC, 5)             // 基地址+0x18（写0x55）
#define IOCTL_CLR_PPS2_STATE      _IO(RTC_IOCTL_MAGIC, 6)             // 基地址+0x1C（写0x55）
#define IOCTL_SET_COMPARE_EN      _IO(RTC_IOCTL_MAGIC, 7)             // 基地址+0x20（特殊顺序）
#define IOCTL_SET_COMPARE_MODE    _IOW(RTC_IOCTL_MAGIC, 8, uint8_t)   // 基地址+0x24
#define IOCTL_SET_COMPARE_TIMES   _IOW(RTC_IOCTL_MAGIC, 9, uint64_t)  // 基地址+0x28/0x2C（64位）
#define IOCTL_SET_COMPARE_VALUE   _IOW(RTC_IOCTL_MAGIC, 10, uint64_t) // 基地址+0x30/0x34（64位）

// 状态输入寄存器 - 读操作命令
#define IOCTL_GET_CLK_SOURCE      _IOR(RTC_IOCTL_MAGIC, 11, uint8_t)  // 基地址+0x6C
#define IOCTL_GET_EXT_CLK_STATE   _IOR(RTC_IOCTL_MAGIC, 12, uint8_t)  // 基地址+0x70
#define IOCTL_GET_SECOND_VALUE    _IOR(RTC_IOCTL_MAGIC, 13, uint64_t) // 基地址+0x74/0x78（64位）
#define IOCTL_GET_MICRO_VALUE     _IOR(RTC_IOCTL_MAGIC, 14, uint64_t) // 基地址+0x7C/0x80（64位）
#define IOCTL_GET_PPS1_STATE      _IOR(RTC_IOCTL_MAGIC, 15, uint8_t)  // 基地址+0x84
#define IOCTL_GET_PPS2_STATE      _IOR(RTC_IOCTL_MAGIC, 16, uint8_t)  // 基地址+0x88
#define IOCTL_GET_RTC_TIMER       _IOR(RTC_IOCTL_MAGIC, 17, uint64_t) // 基地址+0x8C/0x90（64位）
#define IOCTL_GET_DEBUG_SECOND    _IOR(RTC_IOCTL_MAGIC, 18, uint64_t) // 基地址+0x98/0x9C（64位）
#define IOCTL_GET_DEBUG_MICRO     _IOR(RTC_IOCTL_MAGIC, 19, uint64_t) // 基地址+0xA0/0xA4（64位）
#define IOCTL_GET_DEBUG_RTC       _IOR(RTC_IOCTL_MAGIC, 20, uint64_t) // 基地址+0xA8/0xAC（64位）
#define IOCTL_GET_IRQ_STATUS      _IOR(RTC_IOCTL_MAGIC, 21, uint8_t)  // 读取中断状态(0x72250000)
#define IOCTL_SET_IRQ_MASK        _IOW(RTC_IOCTL_MAGIC, 22, uint8_t)  // 设置中断屏蔽(0x72260000)
#define IOCTL_CLEAR_IRQ           _IOW(RTC_IOCTL_MAGIC, 23, uint8_t)  // 清除中断(0x72260000)
#define IOCTL_COMPARE_CNT          _IOW(RTC_IOCTL_MAGIC, 24, uint8_t)  // 读取两个pps秒脉冲之间比较中断的计数值，基地址+0xb0
#define IOCTL_GET_PPS_STATE          _IOW(RTC_IOCTL_MAGIC, 25, uint8_t) //获取pps状态，包括0x11 0x22 0x33
// 更新最大命令号
#define RTC_IOCTL_MAXNR 25

////////////////////////////////////8.14改添加二次修改pcie所需变量
#define DRV_NAME "pcie_irq_example"
#define PCI_DEV_NAME "MS_PCI_DEV"

#define PCI_BAR_0 0
#define PCI_BAR_1 1
#define PCI_BAR_3 3

#define LED_CONTROL 1
#define SRAM_WRITE 2
#define SRAM_READ 3
#define DIP_SWITCH 4
#define INTERRUPT_CTRL 6
#define INTERRUPT_CNTR 7

#define READ 0
#define CLEAR 0

#define INTERRUPT_OFFSET 0x52

#define SUCCESS 0
#define ENABLE 1
#define DISABLE 0
static struct ms_pci_resource *res;  // 设备资源
static long intCounter = 0;          // 中断计数器
struct pci_dev *samplePdev;

// 文件操作函数声明
static int rtc_open(struct inode *inode, struct file *filp);
static int rtc_release(struct inode *inode, struct file *filp);
static long rtc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int rtc_fasync(int fd, struct file *filp, int mode);

// 文件操作结构体
static const struct file_operations rtc_fops = {
    .owner = THIS_MODULE,
    .open = rtc_open,
    .release = rtc_release,
    .unlocked_ioctl = rtc_ioctl,
    .fasync = rtc_fasync,  // 添加异步通知支持
};

// 打开设备
static int rtc_open(struct inode *inode, struct file *filp) {
    filp->private_data = rtc_device;
    ///启用中断
    //rtc_device->irq_enabled = true;
    return 0;
}

// 释放设备
static int rtc_release(struct inode *inode, struct file *filp) {
    // 释放异步通知队列
    rtc_fasync(-1, filp, 0);
    return 0;
}

// 异步通知处理函数
static int rtc_fasync(int fd, struct file *filp, int mode) {
    struct rtc_dev *dev = filp->private_data;
    return fasync_helper(fd, filp, mode, &dev->async_queue);
}

// IOCTL核心处理函数（实现所有寄存器功能）
static long rtc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    struct rtc_dev *dev = filp->private_data;
    uint8_t val8;
    uint32_t val32;
    uint64_t val64;
    int ret = 0;

    // 检查命令合法性
    if (_IOC_TYPE(cmd) != RTC_IOCTL_MAGIC)
        return -ENOTTY;

    if (_IOC_NR(cmd) > RTC_IOCTL_MAXNR)
        return -ENOTTY;

    // 检查用户空间指针有效性
    if ((_IOC_DIR(cmd) & _IOC_READ) && !access_ok((void __user *)arg, _IOC_SIZE(cmd)))
        return -EFAULT;
    if ((_IOC_DIR(cmd) & _IOC_WRITE) && !access_ok((void __user *)arg, _IOC_SIZE(cmd)))
        return -EFAULT;

    // 加锁保护寄存器操作
    if (!mutex_trylock(&dev->lock))
        return -EBUSY;

    switch (cmd) {
        // -------------------------- 软件输出寄存器（写操作） --------------------------
        // 1. 设置外部时钟源（基地址+0x00）
        case IOCTL_SET_CLK_SOURCE:
            if (_IOC_SIZE(cmd) != sizeof(val8)) {
                ret = -EINVAL;
                break;
            }
            if (copy_from_user(&val8, (uint8_t __user *)arg, sizeof(val8))) {
                ret = -EFAULT;
                break;
            }
            writeb(val8, dev->reg_base + 0x00);
            break;

        // 2. 时钟参数秒设置使能（基地址+0x04，先写0x55再写0xAA）
        case IOCTL_SET_SECOND_EN:
            writeb(0x55, dev->reg_base + 0x04);
            writeb(0xAA, dev->reg_base + 0x04);
            break;

        // 3. 设置秒值（64位，0x08低32位，0x0C高32位）
        case IOCTL_SET_SECOND_VALUE:
            if (_IOC_SIZE(cmd) != sizeof(val64)) {
                ret = -EINVAL;
                break;
            }
            if (copy_from_user(&val64, (uint64_t __user *)arg, sizeof(val64))) {
                ret = -EFAULT;
                break;
            }
            val32 = (uint32_t)(val64 & 0xFFFFFFFF);          // 低32位
            writel(val32, dev->reg_base + 0x08);
            val32 = (uint32_t)(val64 >> 32);                 // 高32位
            writel(val32, dev->reg_base + 0x0C);
            break;

        // 4. 设置PPS输入源（基地址+0x10）
        case IOCTL_SET_PPS_SOURCE:
            if (_IOC_SIZE(cmd) != sizeof(val8)) {
                ret = -EINVAL;
                break;
            }
            if (copy_from_user(&val8, (uint8_t __user *)arg, sizeof(val8))) {
                ret = -EFAULT;
                break;
            }
            writeb(val8, dev->reg_base + 0x10);
            break;

        // 5. 设置PPS触发模式（基地址+0x14）
        case IOCTL_SET_PPS_TRIGGER:
            if (_IOC_SIZE(cmd) != sizeof(val8)) {
                ret = -EINVAL;
                break;
            }
            if (copy_from_user(&val8, (uint8_t __user *)arg, sizeof(val8))) {
                ret = -EFAULT;
                break;
            }
            writeb(val8, dev->reg_base + 0x14);
            break;

        // 6. 清除PPS1接收状态（基地址+0x18，写0x55）
        case IOCTL_CLR_PPS1_STATE:
            writeb(0x55, dev->reg_base + 0x18);
            break;

        // 7. 清除PPS2接收状态（基地址+0x1C，写0x55）
        case IOCTL_CLR_PPS2_STATE:
            writeb(0x55, dev->reg_base + 0x1C);
            break;

        // 8. 比较器使能（基地址+0x20，先写0xAA再写0x55）
        /*case IOCTL_SET_COMPARE_EN:
            writeb(0xAA, dev->reg_base + 0x20);
            writeb(0x55, dev->reg_base + 0x20);
            break;*/

        // 9. 设置比较模式（基地址+0x24）
        case IOCTL_SET_COMPARE_MODE:
            if (_IOC_SIZE(cmd) != sizeof(val8)) {
                ret = -EINVAL;
                break;
            }
            if (copy_from_user(&val8, (uint8_t __user *)arg, sizeof(val8))) {
                ret = -EFAULT;
                break;
            }
            writeb(val8, dev->reg_base + 0x24);
            break;

        // 10. 设置比较次数（64位，0x28低32位，0x2C高32位）
        /*case IOCTL_SET_COMPARE_TIMES:
            if (_IOC_SIZE(cmd) != sizeof(val64)) {
                ret = -EINVAL;
                break;
            }
            if (copy_from_user(&val64, (uint64_t __user *)arg, sizeof(val64))) {
                ret = -EFAULT;
                break;
            }
            val32 = (uint32_t)(val64 & 0xFFFFFFFF);
            writel(val32, dev->reg_base + 0x28);
            val32 = (uint32_t)(val64 >> 32);
            writel(val32, dev->reg_base + 0x2C);
            break;*/

        // 11. 设置比较值（64位，0x30低32位，0x34高32位，单位us）
        case IOCTL_SET_COMPARE_VALUE:
            if (_IOC_SIZE(cmd) != sizeof(val64)) {
                ret = -EINVAL;
                break;
            }
            if (copy_from_user(&val64, (uint64_t __user *)arg, sizeof(val64))) {
                ret = -EFAULT;
                break;
            }
            val32 = (uint32_t)(val64 & 0xFFFFFFFF);
            writel(val32, dev->reg_base + 0x30);
            val32 = (uint32_t)(val64 >> 32);
            writel(val32, dev->reg_base + 0x34);
            break;

        // -------------------------- 状态输入寄存器（读操作） --------------------------
        // 1. 获取当前工作时钟源（基地址+0x6C）
        case IOCTL_GET_CLK_SOURCE:
            if (_IOC_SIZE(cmd) != sizeof(val8)) {
                ret = -EINVAL;
                break;
            }
            val8 = readb(dev->reg_base + 0x6C);
            if (copy_to_user((uint8_t __user *)arg, &val8, sizeof(val8)))
                ret = -EFAULT;
            break;

        // 2. 获取外部时钟状态（基地址+0x70）
        case IOCTL_GET_EXT_CLK_STATE:
            if (_IOC_SIZE(cmd) != sizeof(val8)) {
                ret = -EINVAL;
                break;
            }
            val8 = readb(dev->reg_base + 0x70);
            if (copy_to_user((uint8_t __user *)arg, &val8, sizeof(val8)))
                ret = -EFAULT;
            break;

        // 3. 获取时间秒值（64位，0x74低32位，0x78高32位）
        case IOCTL_GET_SECOND_VALUE:
            if (_IOC_SIZE(cmd) != sizeof(val64)) {
                ret = -EINVAL;
                break;
            }
            val32 = readl(dev->reg_base + 0x74);  // 低32位
            val64 = val32;
            val32 = readl(dev->reg_base + 0x78);  // 高32位
            val64 |= (uint64_t)val32 << 32;
            if (copy_to_user((uint64_t __user *)arg, &val64, sizeof(val64)))
                ret = -EFAULT;
            break;

        // 4. 获取时间微秒值（64位，0x7C低32位，0x80高32位）
        case IOCTL_GET_MICRO_VALUE:
            if (_IOC_SIZE(cmd) != sizeof(val64)) {
                ret = -EINVAL;
                break;
            }
            val32 = readl(dev->reg_base + 0x7C);  // 低32位
            val64 = val32;
            val32 = readl(dev->reg_base + 0x80);  // 高32位
            val64 |= (uint64_t)val32 << 32;
            if (copy_to_user((uint64_t __user *)arg, &val64, sizeof(val64)))
                ret = -EFAULT;
            break;

        // 5. 获取PPS1接收状态（基地址+0x84）
        case IOCTL_GET_PPS1_STATE:
            if (_IOC_SIZE(cmd) != sizeof(val8)) {
                ret = -EINVAL;
                break;
            }
            val8 = readb(dev->reg_base + 0x84);
            if (copy_to_user((uint8_t __user *)arg, &val8, sizeof(val8)))
                ret = -EFAULT;
            break;

        // 6. 获取PPS2接收状态（基地址+0x88）
        case IOCTL_GET_PPS2_STATE:
            if (_IOC_SIZE(cmd) != sizeof(val8)) {
                ret = -EINVAL;
                break;
            }
            val8 = readb(dev->reg_base + 0x88);
            if (copy_to_user((uint8_t __user *)arg, &val8, sizeof(val8)))
                ret = -EFAULT;
            break;
        case IOCTL_GET_PPS_STATE:
            if (_IOC_SIZE(cmd) != sizeof(val8)) {
                ret = -EINVAL;
                break;
            }
            val8 = readb(dev->reg_base + 0x94);
            if (copy_to_user((uint8_t __user *)arg, &val8, sizeof(val8)))
                ret = -EFAULT;
            break;




        // 7. 获取RTC计时器值（64位，0x8C低32位，0x90高32位）
        case IOCTL_GET_RTC_TIMER:
            if (_IOC_SIZE(cmd) != sizeof(val64)) {
                ret = -EINVAL;
                break;
            }
            val32 = readl(dev->reg_base + 0x8C);  // 低32位
            val64 = val32;
            val32 = readl(dev->reg_base + 0x90);  // 高32位
            val64 |= (uint64_t)val32 << 32;
            if (copy_to_user((uint64_t __user *)arg, &val64, sizeof(val64)))
                ret = -EFAULT;
            break;

        // 8. 获取调试用秒值锁存（64位，0x98低32位，0x9C高32位）
        case IOCTL_GET_DEBUG_SECOND:
            if (_IOC_SIZE(cmd) != sizeof(val64)) {
                ret = -EINVAL;
                break;
            }
            val32 = readl(dev->reg_base + 0x98);  // 低32位
            val64 = val32;
            val32 = readl(dev->reg_base + 0x9C);  // 高32位
            val64 |= (uint64_t)val32 << 32;
            if (copy_to_user((uint64_t __user *)arg, &val64, sizeof(val64)))
                ret = -EFAULT;
            break;

        // 9. 获取调试用微秒值锁存（64位，0xA0低32位，0xA4高32位）
        case IOCTL_GET_DEBUG_MICRO:
            if (_IOC_SIZE(cmd) != sizeof(val64)) {
                ret = -EINVAL;
                break;
            }
            val32 = readl(dev->reg_base + 0xA0);  // 低32位
            val64 = val32;
            val32 = readl(dev->reg_base + 0xA4);  // 高32位
            val64 |= (uint64_t)val32 << 32;
            if (copy_to_user((uint64_t __user *)arg, &val64, sizeof(val64)))
                ret = -EFAULT;
            break;

        // 10. 获取调试用RTC计时器锁存（64位，0xA8低32位，0xAC高32位）
        case IOCTL_GET_DEBUG_RTC:
            if (_IOC_SIZE(cmd) != sizeof(val64)) {
                ret = -EINVAL;
                break;
            }
            val32 = readl(dev->reg_base + 0xA8);  // 低32位
            val64 = val32;
            val32 = readl(dev->reg_base + 0xAC);  // 高32位
            val64 |= (uint64_t)val32 << 32;
            if (copy_to_user((uint64_t __user *)arg, &val64, sizeof(val64)))
                ret = -EFAULT;
            break;


   // }

    // -------------------------- 中断相关寄存器操作 --------------------------
    // 读取中断状态（0x72250000 + 0x90）
    case IOCTL_GET_IRQ_STATUS:
        if (_IOC_SIZE(cmd) != sizeof(val8)) {
            ret = -EINVAL;
            break;
        }
        // 读取0x72250000+0x90地址的值（添加0x90偏移）
        val8 = readb(dev->irq_status_base + 0x90);
        // 只保留低2位，这两位用于指示中断状态
        val8 &= 0x03;
        if (copy_to_user((uint8_t __user *)arg, &val8, sizeof(val8)))
            ret = -EFAULT;
        break;
    
        // 设置中断屏蔽（0x72260000 + 0xA0）
        case IOCTL_SET_IRQ_MASK:
            if (_IOC_SIZE(cmd) != sizeof(val8)) {
                ret = -EINVAL;
                break;
            }
            if (copy_from_user(&val8, (uint8_t __user *)arg, sizeof(val8))) {
                ret = -EFAULT;
                break;
            }
            // 只保留低4位，第2、3位用于中断屏蔽控制
            val8 &= 0x0C;
            // 写入0x72260000+0xA0地址（添加0xA0偏移）
            writeb(val8, dev->irq_ctrl_base + 0xA0);
            break;
    
            // 清除中断（0x72260000 + 0xA0）
            case IOCTL_CLEAR_IRQ:
                if (_IOC_SIZE(cmd) != sizeof(val8)) {
                    ret = -EINVAL;
                    break;
                }
                if (copy_from_user(&val8, (uint8_t __user *)arg, sizeof(val8))) {
                    ret = -EFAULT;
                    break;
                }
                // 只保留低2位，用于清除相应中断
                val8 &= 0x03;
                // 写1再写0清除中断（脉冲信号）（添加0xA0偏移）
                writeb(val8, dev->irq_ctrl_base + 0xA0);
                writeb(0x00, dev->irq_ctrl_base + 0xA0);
                break;
    
            //读取比较中断的次数（基地址+0xb0）
            case IOCTL_COMPARE_CNT:
                if (_IOC_SIZE(cmd) != sizeof(val8)) {
                    ret = -EINVAL;
                    break;
                }
            val8 = readb(dev->reg_base + 0xb0);
            if (copy_to_user((uint8_t __user *)arg, &val8, sizeof(val8)))
                ret = -EFAULT;
            break;
        // 未定义命令
        default:
            ret = -ENOTTY;
            break;
    }

    mutex_unlock(&dev->lock);  // 释放锁
    return ret;
}

// 模块初始化函数
int  rtc_init(void) {
    int ret;

    // 1. 分配设备结构体
    rtc_device = kzalloc(sizeof(struct rtc_dev), GFP_KERNEL);
    if (!rtc_device) {
        ret = -ENOMEM;
        goto err_alloc;
    }

    // 初始化互斥锁
    mutex_init(&rtc_device->lock);

    // 2. 动态分配设备号
    ret = alloc_chrdev_region(&rtc_device->dev_num, 0, 1, "rtc_device");
    if (ret < 0)
        goto err_chrdev;

    // 3. 初始化cdev并关联文件操作
    cdev_init(&rtc_device->cdev, &rtc_fops);
    rtc_device->cdev.owner = THIS_MODULE;
    ret = cdev_add(&rtc_device->cdev, rtc_device->dev_num, 1);
    if (ret < 0)
        goto err_cdev;

    // 4. 创建设备类
    rtc_device->class = class_create(THIS_MODULE, "rtc_class");
    if (IS_ERR(rtc_device->class)) {
        ret = PTR_ERR(rtc_device->class);
        goto err_class;
    }

    // 5. 创建设备节点（/dev/rtc_dev）
    rtc_device->device = device_create(rtc_device->class, NULL, 
                                      rtc_device->dev_num, NULL, "rtc_dev");
    if (IS_ERR(rtc_device->device)) {
        ret = PTR_ERR(rtc_device->device);
        goto err_device;
    }

    // 6. 映射RTC寄存器物理地址到虚拟地址
    rtc_device->reg_base = ioremap(RTC_BASE_PHYS, RTC_REG_SIZE);
    if (!rtc_device->reg_base) {
        ret = -ENOMEM;
        goto err_ioremap;
    }

    // 7. 映射中断状态寄存器物理地址到虚拟地址
    rtc_device->irq_status_base = ioremap(IRQ_STATUS_BASE_PHYS, IRQ_REG_SIZE);
    if (!rtc_device->irq_status_base) {
        ret = -ENOMEM;
        goto err_irq_status_ioremap;
    }

    // 8. 映射中断控制寄存器物理地址到虚拟地址
    rtc_device->irq_ctrl_base = ioremap(IRQ_CTRL_BASE_PHYS, IRQ_REG_SIZE);
    if (!rtc_device->irq_ctrl_base) {
        ret = -ENOMEM;
        goto err_irq_ctrl_ioremap;
    }
    //9.控制中断暂不启用
    //rtc_device->irq_enabled = false;
    printk(KERN_INFO "RTC driver initialized successfully\n");
    return 0;

    // 错误处理
err_irq_ctrl_ioremap:
    iounmap(rtc_device->irq_status_base);
err_irq_status_ioremap:
    iounmap(rtc_device->reg_base);
err_ioremap:
    device_destroy(rtc_device->class, rtc_device->dev_num);
err_device:
    class_destroy(rtc_device->class);
err_class:
    cdev_del(&rtc_device->cdev);
err_cdev:
    unregister_chrdev_region(rtc_device->dev_num, 1);
err_chrdev:
    mutex_destroy(&rtc_device->lock);  // 销毁锁
    kfree(rtc_device);
err_alloc:
    printk(KERN_ERR "RTC driver initialization failed: %d\n", ret);
    return ret;
}

// 模块退出函数
void  rtc_exit(void) {
    // 释放资源
    iounmap(rtc_device->reg_base);
    iounmap(rtc_device->irq_status_base);
    iounmap(rtc_device->irq_ctrl_base);
    device_destroy(rtc_device->class, rtc_device->dev_num);
    class_destroy(rtc_device->class);
    cdev_del(&rtc_device->cdev);
    unregister_chrdev_region(rtc_device->dev_num, 1);
    mutex_destroy(&rtc_device->lock);  // 销毁锁
    kfree(rtc_device);
    printk(KERN_INFO "RTC driver exited\n");
}


//////////////////////////////////////////////////////8.14二次修改pci
// PCI设备ID表（用于匹配设备）
static struct pci_device_id ms_pci_tbl[] = {
    {0x11aa, 0x11aa, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
    {
        0,
    }};
MODULE_DEVICE_TABLE(pci, ms_pci_tbl);

//
struct ms_pci_resource
{
    /* PCI BAR #0 */
    u32 bar0_base;
    u32 bar0_size;
    volatile void __iomem *base_addr0; // virtual address to represent PCI BAR #0

    /* PCI BAR #1 */
    u32 bar1_base;
    u32 bar1_size;
    volatile void __iomem *base_addr1; // virtual address to represent PCI BAR #1
};
void interrupt_control(struct pci_dev *pdev, unsigned char data)
{
    if (data == ENABLE)
    {
        pci_read_config_byte(pdev, INTERRUPT_OFFSET, &data);

        data = *(&data) | 0x01; /* Enable the first bit */

        pci_write_config_byte(pdev, INTERRUPT_OFFSET, data);

        pr_info("pci write config success :: MSI Interrupt Enabled\n");
    }
    else if (data == DISABLE)
    {
        pci_read_config_byte(pdev, INTERRUPT_OFFSET, &data);

        data = *(&data) & 0xFE; /* Mask the first bit */

        pci_write_config_byte(pdev, INTERRUPT_OFFSET, data);

        pr_info("pci write config success :: MSI Interrupt Disabled\n");

        intCounter = CLEAR; /* Clear the interrupt Counter */
    }
    else
    {
        pr_info("Invalid Interrupt Data Flag recieved\n");
    }
}

/**
 * ms_pci_interrupt() - PCIe interrupt Handler
 *
 *  @irq : PCIe interrupt number
 */
static irqreturn_t ms_pci_interrupt(int irq, void *dev_id)
{
    intCounter++;
    //if (!rtc_device->irq_enabled) {
    unsigned int regData = readb(res->base_addr0);
   // pr_info("intCounter = %ld, regData = 0x%x\n", intCounter, regData);
    
    // 可选：写寄存器清除中断标志
    // writeb(0x01, res->base_addr0 + 0x04);
    // 发送异步通知给应用程序
    if (rtc_device && rtc_device->async_queue) {
        kill_fasync(&rtc_device->async_queue, SIGIO, POLL_IN);
    }
    return IRQ_HANDLED;
//}
}
/**
 * ms_pci_probe() - Initialize char device Module
 *
 *  @pdev : Platform Device
 *  @ent  : PCI Device ID List Entry
 */
// PCI设备probe函数：初始化设备并注册中断
static int ms_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    int i;

    pr_info("pci device found\n");

    i = pci_enable_device(pdev);
    if (i)
        return i;

    pci_set_master(pdev);

    res = (struct ms_pci_resource *)kmalloc(sizeof(*res), GFP_KERNEL);
    if (res == NULL)
    {
        pr_info("Unable to allocate memory for PCI resource\n");
        return -ENOMEM;
    }

    /* PCI BAR 0# resources */
    //其实是映射的中断位置BAR3
    res->bar0_base = pci_resource_start(pdev, PCI_BAR_3);
    pr_info("PCI BAR #0 BASE = 0x%x\n", res->bar0_base);

    if (!res->bar0_base || ((pci_resource_flags(pdev, PCI_BAR_3) & IORESOURCE_MEM) == 0))
    {
        dev_err(&pdev->dev, "no I/O resource at PCI BAR #0\n");
        goto err_out_free_res;
    }

    res->bar0_size = pci_resource_len(pdev, PCI_BAR_3);
    pr_info("PCI BAR #0 SIZE = 0x%x\n", res->bar0_size);

    if (request_mem_region(res->bar0_base, res->bar0_size, DRV_NAME) == NULL)
    {
        pr_info("PCI BAR #0 Space not available\n");
        goto err_out_free_res;
    }

    res->base_addr0 = (void *)ioremap_cache(res->bar0_base, res->bar0_size);
    pr_info("base_addr0 = 0x%p\n", res->base_addr0);
    if (res->base_addr0 == NULL)
    {
        pr_info("Unable to map BAR #0 address");
        goto err_out_free_reg;
    }
#if 0
#endif

    if (pci_enable_msi(pdev))
    {
        pr_info("Unable to enable MSI interrupt 0x%x\n", pdev->irq);
        goto err_out_free_reg;
    }
    pr_info("MSI interrupt 0x%x\n", pdev->irq);

    if (request_irq(pdev->irq, ms_pci_interrupt, IRQF_SHARED, DRV_NAME, pdev))
    {
        pr_info("unable to register irq 0x%x\n", pdev->irq);
        goto err_out_free_irq;
    }

   if (rtc_init() != 0) {
        pr_err("Failed to initialize rtc device\n");
        goto err_out_free_irq;  // 或者设计为可恢复
    }
    samplePdev = pdev;
    
    return 0;

err_out_free_irq:
    free_irq(pdev->irq, pdev);
    pci_disable_msi(pdev);
    iounmap(res->base_addr0);
    // iounmap(res->base_addr1);
err_out_free_reg:
    release_mem_region(res->bar0_base, res->bar0_size);
    // release_mem_region(res->bar1_base, res->bar1_size);
err_out_free_res:
    kfree((const void *)res);
    return -ENODEV;
}

/**
 * ms_pci_remove() - Uninitialize the PCIe device
 *
 *  @pdev : Platform Device
 */
 // PCI设备remove函数：释放资源
static void ms_pci_remove(struct pci_dev *pdev)
{
    rtc_exit();  // 释放字符设备
    pci_clear_master(pdev);
    pci_disable_device(pdev);
    free_irq(pdev->irq, pdev);
    pci_disable_msi(pdev);
    iounmap(res->base_addr0);
    // iounmap(res->base_addr1);
    release_mem_region(res->bar0_base, res->bar0_size);
    // release_mem_region(res->bar1_base, res->bar1_size);
    kfree((const void *)res);
}

static struct pci_driver ms_pci_driver = {
    .name = "pcie_irq_example",
    .probe = ms_pci_probe,
    .remove = ms_pci_remove,
    .id_table = ms_pci_tbl,
};


module_pci_driver(ms_pci_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Driver Developer");
MODULE_DESCRIPTION("RTC Driver with Async Notification Support");
MODULE_VERSION("1.0");

