#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/delay.h>

#define DEVICE_NAME "i2c_controller"
#define CLASS_NAME "i2c_controller_class"
#define I2C_NUM_CHANNELS 5

// I2C基地址定义
#define BASE_ADDR_I2C(n) (0x72100000 + ((n) * 0x10000)) // 0~4对应I2C1~I2C5

// 寄存器地址
#define I2C_CTRL_REG 0x00
#define I2C_STAT_REG 0x04
#define I2C_DATA_REG 0x08

// 控制位定义
#define I2C_CTRL_CR2 (1 << 7)
#define I2C_CTRL_ENS1 (1 << 6)
#define I2C_CTRL_STA (1 << 5)
#define I2C_CTRL_STO (1 << 4)
#define I2C_CTRL_SI (1 << 3)
#define I2C_CTRL_AA (1 << 2)
#define I2C_CTRL_CR1 (1 << 1)
#define I2C_CTRL_CR0 (1 << 0)

// 状态码定义
#define I2C_STATUS_START_TRANSMITTED 0x08
#define I2C_STATUS_REPEATED_START_TRANSMITTED 0x10
#define I2C_STATUS_SLA_W_ACK 0x18
#define I2C_STATUS_DATA_TRANSMITTED_ACK 0x28
#define I2C_STATUS_SLA_R_ACK 0x40
#define I2C_STATUS_DATA_RECEIVED_ACK 0x50
#define I2C_STATUS_DATA_RECEIVED_NACK 0x58
#define I2C_STATUS_STOP_TRANSMITTED 0xE0

// IOCTL命令定义
#define I2C_IOCTL_MAGIC 'i'
#define I2C_SET_CHANNEL _IOW(I2C_IOCTL_MAGIC, 0, int)
#define I2C_WRITE_BYTE _IOW(I2C_IOCTL_MAGIC, 1, struct i2c_data)
#define I2C_READ_BYTE _IOWR(I2C_IOCTL_MAGIC, 2, struct i2c_data)
#define I2C_READ_TWO_BYTES _IOWR(I2C_IOCTL_MAGIC, 3, struct i2c_two_bytes)

// 数据结构定义
struct i2c_data
{
    uint8_t dev_addr;
    uint8_t reg_addr;
    uint8_t data;
};

struct i2c_two_bytes
{
    uint8_t dev_addr;
    uint8_t reg_addr;
    uint8_t data1;
    uint8_t data2;
};

// 设备私有数据结构
struct i2c_controller_dev
{
    dev_t dev_num;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    void __iomem *base_addr[I2C_NUM_CHANNELS];
    int current_channel;
    // spinlock_t lock;
    struct mutex lock;
};

static struct i2c_controller_dev *i2c_dev;

// 寄存器操作函数
static uint8_t i2c_get_reg(int channel, uint8_t offset)
{
    return readb(i2c_dev->base_addr[channel] + offset);
}

static void i2c_set_reg(int channel, uint8_t offset, uint8_t val)
{
    writeb(val, i2c_dev->base_addr[channel] + offset);
}

static uint8_t i2c_get_ctrl(int channel)
{
    return i2c_get_reg(channel, I2C_CTRL_REG);
}

static void i2c_set_ctrl(int channel, uint8_t val)
{
    i2c_set_reg(channel, I2C_CTRL_REG, val);
}

static uint8_t i2c_get_data(int channel)
{
    return i2c_get_reg(channel, I2C_DATA_REG);
}

static void i2c_set_data(int channel, uint8_t val)
{
    i2c_set_reg(channel, I2C_DATA_REG, val);
}

static uint8_t i2c_get_status(int channel)
{
    return i2c_get_reg(channel, I2C_STAT_REG);
}

// 等待SI位设置
static int i2c_wait_si(int channel, const char *step)
{
    int timeout = 1000;
    while (!(i2c_get_ctrl(channel) & I2C_CTRL_SI))
    {
        if (--timeout <= 0)
        {
            dev_err(i2c_dev->device, "%s: Timeout waiting for SI\n", step);
            return -ETIMEDOUT;
        }
        usleep_range(10, 20);
    }
    return 0;
}

// I2C初始化函数
static void i2c_init_controller(int channel)
{
//    unsigned long flags;
    mutex_lock(&i2c_dev->lock);

    // 重置控制寄存器
    i2c_set_ctrl(channel, 0x00);
    // 使能I2C控制器
    i2c_set_ctrl(channel, I2C_CTRL_ENS1 | I2C_CTRL_CR2 | I2C_CTRL_CR1 | I2C_CTRL_CR0);

    mutex_unlock(&i2c_dev->lock);
}

// I2C写一个字节
static int i2c_master_write_byte(int channel, uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
//    unsigned long flags;
    int ret;

    mutex_lock(&i2c_dev->lock);

    // 发送START条件
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) | I2C_CTRL_STA);
    ret = i2c_wait_si(channel, "Write start condition");
    if (ret < 0)
        goto out;

    // 发送设备地址(写模式)
    i2c_set_data(channel, dev_addr << 1);
    ret = i2c_wait_si(channel, "Write start condition");
    if (ret < 0)
        goto out;
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~(I2C_CTRL_STA | I2C_CTRL_SI));
    ret = i2c_wait_si(channel, "Write device address");
    if (ret < 0)
        goto out;

    if (i2c_get_status(channel) != I2C_STATUS_SLA_W_ACK)
    {
        dev_err(i2c_dev->device, "SLA+W not acknowledged (0x%02X)\n",
                i2c_get_status(channel));
        ret = -EIO;
        printk("send addr error\n");
        goto out;
    }
    printk("send addr success\n");
    // 发送寄存器地址
    i2c_set_data(channel, reg_addr);
    ret = i2c_wait_si(channel, "Write device address");
    if (ret < 0)
        goto out;
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~I2C_CTRL_SI);

    ret = i2c_wait_si(channel, "Write register address");
    if (ret < 0)
        goto out;

    if (i2c_get_status(channel) != I2C_STATUS_DATA_TRANSMITTED_ACK)
    {
        dev_err(i2c_dev->device, "Register address not acknowledged (0x%02X)\n",
                i2c_get_status(channel));
        ret = -EIO;
        printk("send reg error\n");
        goto out;
    }
    printk("send reg success\n");
    // 发送数据
    i2c_set_data(channel, data);
    ret = i2c_wait_si(channel, "Write register address");
    if (ret < 0)
        goto out;
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~I2C_CTRL_SI);
    ret = i2c_wait_si(channel, "Write data");
    if (ret < 0)
        goto out;

    if (i2c_get_status(channel) != I2C_STATUS_DATA_TRANSMITTED_ACK)
    {
        dev_err(i2c_dev->device, "Data not acknowledged (0x%02X)\n",
                i2c_get_status(channel));
        ret = -EIO;
        printk("send data error\n");
        goto out;
    }
    printk("send data success\n");
    // 发送STOP条件
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) | I2C_CTRL_STO);
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~I2C_CTRL_SI);

    ret = i2c_wait_si(channel, "Write data");
    if (ret < 0)
        goto out;
    // 等待STOP完成
    ret = 1000;
    while (--ret > 0)
    {
        if (i2c_get_status(channel) == I2C_STATUS_STOP_TRANSMITTED)
            break;
        usleep_range(10, 20);
    }

    if (ret <= 0)
    {
        dev_warn(i2c_dev->device, "Timeout waiting for STOP\n");
    }

    //每次写完一次后，读第一次就报错了，可能没有清SI信号，加上，试下看是否解决。
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~I2C_CTRL_SI);

    ret = 0;

out:
    mutex_unlock(&i2c_dev->lock);
    return ret;
}

// I2C读一个字节
static int i2c_master_read_byte(int channel, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    //unsigned long flags;
    int ret;

    mutex_lock(&i2c_dev->lock);

    // 第一步：写寄存器地址
    // 发送START条件
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) | I2C_CTRL_STA);
    ret = i2c_wait_si(channel, "Write start condition");
    if (ret < 0)
        goto out;

    // 发送设备地址(写模式)
    i2c_set_data(channel, dev_addr << 1);
    ret = i2c_wait_si(channel, "Write device address");
    if (ret < 0)
        goto out;
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~(I2C_CTRL_STA | I2C_CTRL_SI));
    ret = i2c_wait_si(channel, "Clear START & SI");
    if (ret < 0)
        goto out;
    
    if (i2c_get_status(channel) != I2C_STATUS_SLA_W_ACK) {
        dev_err(i2c_dev->device, "Register address not acknowledged (0x%02X)\n",i2c_get_status(channel));       return -1;
        ret = -EIO;
        printk("i2c I2C_STATUS_SLA_W_ACK error\n");
        goto out;
    }
    
    // 发送寄存器地址
    i2c_set_data(channel, reg_addr);
    
    ret = i2c_wait_si(channel, "write register address");
    if (ret < 0)
        goto out;
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~I2C_CTRL_SI);
    ret = i2c_wait_si(channel, "Read register address");
    if (ret < 0)
        goto out;

    if (i2c_get_status(channel) != I2C_STATUS_DATA_TRANSMITTED_ACK)
    {
        dev_err(i2c_dev->device, "Register address not acknowledged (0x%02X)\n",
                i2c_get_status(channel));
        ret = -EIO;
        printk("i2c set reg error\n");
        goto out;
    }

    ////Repeated START
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) | I2C_CTRL_STA);
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~I2C_CTRL_SI);
    ret = i2c_wait_si(channel, "Repeated start");
    if (ret < 0)
        goto out;

    // 发送设备地址(读模式)
    i2c_set_data(channel, (dev_addr << 1) | 1);
    ret = i2c_wait_si(channel, "Repeated start");
    if (ret < 0)
        goto out;
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~(I2C_CTRL_STA | I2C_CTRL_SI));
    ret = i2c_wait_si(channel, "Repeated start");
    if (ret < 0)
        goto out;

    if (i2c_get_status(channel) != I2C_STATUS_SLA_R_ACK)
    {
        dev_err(i2c_dev->device, "SLA+R not acknowledged (0x%02X)\n",
                i2c_get_status(channel));
        ret = -EIO;
        printk("i2c set addr error\n");
        goto out;
    }

    // 准备接收数据，发送NACK
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) | I2C_CTRL_AA);// 使能ACK
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~I2C_CTRL_SI);
    ret = i2c_wait_si(channel, "Read data");
    if (ret < 0)
        goto out;

    if (i2c_get_status(channel) != I2C_STATUS_DATA_RECEIVED_ACK)
    {
        dev_err(i2c_dev->device, "Data receive failed (0x%02X)\n",
                i2c_get_status(channel));
        ret = -EIO;
        goto out;
    }

    // 读取第一个字节数据
    *data = i2c_get_data(channel);
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~(I2C_CTRL_AA | I2C_CTRL_SI));
    ret = i2c_wait_si(channel, "Read data");
    if (ret < 0)
        goto out;
    if (i2c_get_status(channel) != I2C_STATUS_DATA_RECEIVED_NACK)
    {
        printk("[ERROR] [DATA2 NACK STATUS] STATUS=0x%02X\n", i2c_get_status(channel));
        return -1;
    }

    // 第四步：接收第二个字节（返回NACK，告知从设备结束传输）
    *data = i2c_get_data(channel);

    // 发送STOP条件
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) | I2C_CTRL_STO);
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~I2C_CTRL_SI);
    ret = i2c_wait_si(channel, "stop");
    if (ret < 0)
        goto out;

    // 等待STOP完成
    ret = 1000;
    while (--ret > 0)
    {
        if (i2c_get_status(channel) == I2C_STATUS_STOP_TRANSMITTED)
            break;
        usleep_range(10, 20);
    }

    if (ret <= 0)
    {
        dev_warn(i2c_dev->device, "Timeout waiting for STOP\n");
    }

    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~I2C_CTRL_SI);
    ret = i2c_wait_si(channel, "Read device address (read)");
    if (ret < 0)
        goto out;
    ret = 0;

out:
    mutex_unlock(&i2c_dev->lock);
    return ret;
}

// I2C读两个字节
static int i2c_master_read_two_bytes(int channel, uint8_t dev_addr, uint8_t reg_addr,
                                     uint8_t *data1, uint8_t *data2)
{
    //unsigned long flags;
    int ret;

    mutex_lock(&i2c_dev->lock);

    // 第一步：写寄存器地址
    // 发送START条件
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) | I2C_CTRL_STA);
    ret = i2c_wait_si(channel, "Read start condition");
    if (ret < 0)
        goto out;

    // 发送设备地址(写模式)
    i2c_set_data(channel, dev_addr << 1);
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~(I2C_CTRL_STA | I2C_CTRL_SI));
    ret = i2c_wait_si(channel, "Read start condition");
    if (ret < 0)
        goto out;
    if (i2c_get_status(channel) != I2C_STATUS_SLA_W_ACK)
    {
        printk("[ERROR] [SLA+W STATUS] STATUS=0x%02X\n", i2c_get_status(channel));
        ret = -EIO;
        goto out;
    }
    
    // 发送寄存器地址
    i2c_set_data(channel, reg_addr);
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~I2C_CTRL_SI);
    ret = i2c_wait_si(channel, "Write register address");
    if (ret < 0)
        goto out;

    if (i2c_get_status(channel) != I2C_STATUS_DATA_TRANSMITTED_ACK)
    {
        dev_err(i2c_dev->device, "Register address not acknowledged (0x%02X)\n",
                i2c_get_status(channel));
        ret = -EIO;
        printk("i2c set reg error\n");
        goto out;
    }

    // 第二步：读数据
    // 发送重复START
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) | I2C_CTRL_STA);
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~I2C_CTRL_SI);

    ret = i2c_wait_si(channel, "Repeated start");
    if (ret < 0)
        goto out;

    if (i2c_get_status(channel) != I2C_STATUS_REPEATED_START_TRANSMITTED)
    {
        dev_err(i2c_dev->device, "Repeated start failed (0x%02X)\n",
                i2c_get_status(channel));
        ret = -EIO;
        printk("i2c set restart error\n");
        goto out;
    }

    // 发送设备地址(读模式)
    i2c_set_data(channel, (dev_addr << 1) | 1);
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~(I2C_CTRL_STA | I2C_CTRL_SI));
    ret = i2c_wait_si(channel, "Write device address (read mode)");
    if (ret < 0)
        goto out;

    if (i2c_get_status(channel) != I2C_STATUS_SLA_R_ACK)
    {
        dev_err(i2c_dev->device, "SLA+R not acknowledged (0x%02X)\n",
                i2c_get_status(channel));
        ret = -EIO;
        printk("i2c set addr error\n");
        goto out;
    }

    // 准备接收数据，发送NACK
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) | I2C_CTRL_AA);// 使能ACK
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~I2C_CTRL_SI);
    ret = i2c_wait_si(channel, "Read data");
    if (ret < 0)
        goto out;

    if (i2c_get_status(channel) != I2C_STATUS_DATA_RECEIVED_ACK)
    {
        dev_err(i2c_dev->device, "Data receive failed (0x%02X)\n",
                i2c_get_status(channel));
        ret = -EIO;
        goto out;
    }

    // 读取第一个字节数据
    *data1 = i2c_get_data(channel);
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~(I2C_CTRL_AA | I2C_CTRL_SI));
    ret = i2c_wait_si(channel, "Read data");
    if (ret < 0)
        goto out;
    if (i2c_get_status(channel) != I2C_STATUS_DATA_RECEIVED_NACK)
    {
        printk("[ERROR] [DATA2 NACK STATUS] STATUS=0x%02X\n", i2c_get_status(channel));
        return -1;
    }

    // 第四步：接收第二个字节（返回NACK，告知从设备结束传输）
    *data2 = i2c_get_data(channel);

    // 发送STOP条件
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) | I2C_CTRL_STO);
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~I2C_CTRL_SI);
    ret = i2c_wait_si(channel, "stop");
    if (ret < 0)
        goto out;

    // 等待STOP完成
    ret = 1000;
    while (--ret > 0)
    {
        if (i2c_get_status(channel) == I2C_STATUS_STOP_TRANSMITTED)
            break;
        usleep_range(10, 20);
    }

    if (ret <= 0)
    {
        dev_warn(i2c_dev->device, "Timeout waiting I2C_STATUS_STOP_TRANSMITTED for STOP\n");
    }

    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~I2C_CTRL_SI);
    // ret = i2c_wait_si(channel, "Read device address (read)");
    // if (ret < 0)
    //     goto out;
    ret = 0;

out:
    mutex_unlock(&i2c_dev->lock);
    return ret;
}

// 文件操作函数
static int i2c_open(struct inode *inode, struct file *file)
{
    // 默认使用第一个通道
    i2c_dev->current_channel = 0;
    return 0;
}

static int i2c_release(struct inode *inode, struct file *file)
{
    return 0;
}

static long i2c_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    int channel;
    struct i2c_data data;
    struct i2c_two_bytes two_bytes;

    // 检查命令合法性
    if (_IOC_TYPE(cmd) != I2C_IOCTL_MAGIC)
        return -ENOTTY;

    switch (cmd)
    {
    case I2C_SET_CHANNEL:
        if (copy_from_user(&channel, (int __user *)arg, sizeof(channel)))
            return -EFAULT;

        // 转换为0-4的索引（用户空间是1-5）
        channel--;
        if (channel < 0 || channel >= I2C_NUM_CHANNELS)
            return -EINVAL;

        i2c_dev->current_channel = channel;
        break;

    case I2C_WRITE_BYTE:
        if (copy_from_user(&data, (struct i2c_data __user *)arg, sizeof(data)))
            return -EFAULT;

        ret = i2c_master_write_byte(i2c_dev->current_channel,
                                    data.dev_addr, data.reg_addr, data.data);
        break;

    case I2C_READ_BYTE:
        if (copy_from_user(&data, (struct i2c_data __user *)arg, sizeof(data)))
            return -EFAULT;

        ret = i2c_master_read_byte(i2c_dev->current_channel,
                                   data.dev_addr, data.reg_addr, &data.data);
        if (ret == 0 && copy_to_user((struct i2c_data __user *)arg, &data, sizeof(data)))
            ret = -EFAULT;
        break;

    case I2C_READ_TWO_BYTES:
        if (copy_from_user(&two_bytes, (struct i2c_two_bytes __user *)arg, sizeof(two_bytes)))
            return -EFAULT;

        ret = i2c_master_read_two_bytes(i2c_dev->current_channel,
                                        two_bytes.dev_addr, two_bytes.reg_addr,
                                        &two_bytes.data1, &two_bytes.data2);
        if (ret == 0 && copy_to_user((struct i2c_two_bytes __user *)arg, &two_bytes, sizeof(two_bytes)))
            ret = -EFAULT;
        break;

    default:
        ret = -ENOTTY;
        break;
    }

    return ret;
}

// 文件操作结构体
static const struct file_operations i2c_fops = {
    .owner = THIS_MODULE,
    .open = i2c_open,
    .release = i2c_release,
    .unlocked_ioctl = i2c_ioctl,
};

// 模块初始化函数
static int __init i2c_controller_init(void)
{
    int ret;
    int i;

    // 分配设备结构体内存
    i2c_dev = kzalloc(sizeof(struct i2c_controller_dev), GFP_KERNEL);
    if (!i2c_dev)
        return -ENOMEM;

    // 初始化自旋锁
    mutex_init(&i2c_dev->lock);

    // 申请设备号
    ret = alloc_chrdev_region(&i2c_dev->dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0)
    {
        dev_err(NULL, "Failed to allocate char device region\n");
        goto fail_alloc;
    }

    // 初始化cdev
    cdev_init(&i2c_dev->cdev, &i2c_fops);
    i2c_dev->cdev.owner = THIS_MODULE;

    // 添加cdev到系统
    ret = cdev_add(&i2c_dev->cdev, i2c_dev->dev_num, 1);
    if (ret < 0)
    {
        dev_err(NULL, "Failed to add cdev\n");
        goto fail_cdev;
    }

    // 创建类
    i2c_dev->class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(i2c_dev->class))
    {
        dev_err(NULL, "Failed to create class\n");
        ret = PTR_ERR(i2c_dev->class);
        goto fail_class;
    }

    // 创建设备节点
    i2c_dev->device = device_create(i2c_dev->class, NULL, i2c_dev->dev_num,
                                    NULL, DEVICE_NAME);
    if (IS_ERR(i2c_dev->device))
    {
        dev_err(NULL, "Failed to create device\n");
        ret = PTR_ERR(i2c_dev->device);
        goto fail_device;
    }

    // 映射I2C控制器寄存器
    for (i = 0; i < I2C_NUM_CHANNELS; i++)
    {
        i2c_dev->base_addr[i] = ioremap(BASE_ADDR_I2C(i), 0x1000);
        if (!i2c_dev->base_addr[i])
        {
            dev_err(NULL, "Failed to ioremap I2C channel %d\n", i + 1);
            ret = -ENOMEM;
            goto fail_ioremap;
        }

        // 初始化I2C控制器
        i2c_init_controller(i);
        dev_info(i2c_dev->device, "I2C channel %d mapped to 0x%lx\n",
                 i + 1, (unsigned long)BASE_ADDR_I2C(i));
    }

    dev_info(i2c_dev->device, "I2C controller driver initialized\n");
    return 0;

fail_ioremap:
    for (i--; i >= 0; i--)
        iounmap(i2c_dev->base_addr[i]);
    device_destroy(i2c_dev->class, i2c_dev->dev_num);
fail_device:
    class_destroy(i2c_dev->class);
fail_class:
    cdev_del(&i2c_dev->cdev);
fail_cdev:
    unregister_chrdev_region(i2c_dev->dev_num, 1);
fail_alloc:
    kfree(i2c_dev);
    return ret;
}

// 模块退出函数
static void __exit i2c_controller_exit(void)
{
    int i;

    // 解除映射
    for (i = 0; i < I2C_NUM_CHANNELS; i++)
    {
        if (i2c_dev->base_addr[i])
            iounmap(i2c_dev->base_addr[i]);
    }

    // 清理设备
    device_destroy(i2c_dev->class, i2c_dev->dev_num);
    class_destroy(i2c_dev->class);
    cdev_del(&i2c_dev->cdev);
    unregister_chrdev_region(i2c_dev->dev_num, 1);
    kfree(i2c_dev);

    dev_info(NULL, "I2C controller driver exited\n");
}

module_init(i2c_controller_init);
module_exit(i2c_controller_exit);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("I2C Controller Character Device Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
