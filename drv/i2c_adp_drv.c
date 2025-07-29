#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#define I2C_NUM_CHANNELS 5
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
#define I2C_STATUS_SLA_W_NACK 0x20
#define I2C_STATUS_DATA_TRANSMITTED_ACK 0x28
#define I2C_STATUS_DATA_TRANSMITTED_NACK 0x30
#define I2C_STATUS_ARBITRATION_LOST 0x38
#define I2C_STATUS_SLA_R_ACK 0x40
#define I2C_STATUS_SLA_R_NACK 0x48
#define I2C_STATUS_DATA_RECEIVED_ACK 0x50
#define I2C_STATUS_DATA_RECEIVED_NACK 0x58
#define I2C_STATUS_STOP_TRANSMITTED 0xE0

// 设备私有数据结构
struct i2c_platform_dev
{
    void __iomem *base_addr[I2C_NUM_CHANNELS];
    struct i2c_adapter adapters[I2C_NUM_CHANNELS];
    struct mutex lock[I2C_NUM_CHANNELS];
};

static struct i2c_platform_dev *i2c_dev;

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
    int timeout = 10000;
    while (!(i2c_get_ctrl(channel) & I2C_CTRL_SI))
    {
        if (--timeout <= 0)
        {
            dev_err(&i2c_dev->adapters[channel].dev,
                    "%s: Timeout waiting for SI\n", step);
            return -ETIMEDOUT;
        }
        usleep_range(10, 20);
    }
    return 0;
}

// 初始化I2C控制器
static void i2c_hw_init(int channel)
{
    // 重置控制器
    i2c_set_ctrl(channel, 0x00);
    udelay(10);

    // 使能I2C控制器
    i2c_set_ctrl(channel, I2C_CTRL_ENS1 | I2C_CTRL_CR2 |
                              I2C_CTRL_CR1 | I2C_CTRL_CR0);
    udelay(10);
}

// 传输单个I2C消息
static int i2c_transfer_msg(int channel, struct i2c_msg *msg)
{
    int ret;
    int i;
    uint8_t status;

    // 发送START条件
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) | I2C_CTRL_STA);
    ret = i2c_wait_si(channel, "Start condition");
    if (ret < 0)
        return ret;

    status = i2c_get_status(channel);
    if (status != I2C_STATUS_START_TRANSMITTED &&
        status != I2C_STATUS_REPEATED_START_TRANSMITTED)
    {
        dev_err(&i2c_dev->adapters[channel].dev,
                "Invalid start status: 0x%02x\n", status);
        return -EIO;
    }

    // 发送设备地址
    if (msg->flags & I2C_M_RD)
    {
        // 读操作
        i2c_set_data(channel, (msg->addr << 1) | 0x01);
    }
    else
    {
        // 写操作
        i2c_set_data(channel, (msg->addr << 1) & 0xFE);
    }

    // 清除START和SI位
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~(I2C_CTRL_STA | I2C_CTRL_SI));

    // 等待地址确认
    ret = i2c_wait_si(channel, "Address ACK");
    if (ret < 0)
        return ret;

    status = i2c_get_status(channel);
    if ((msg->flags & I2C_M_RD) && status != I2C_STATUS_SLA_R_ACK)
    {
        dev_err(&i2c_dev->adapters[channel].dev,
                "SLA+R not acknowledged: 0x%02x\n", status);
        return -EIO;
    }

    if (!(msg->flags & I2C_M_RD) && status != I2C_STATUS_SLA_W_ACK)
    {
        dev_err(&i2c_dev->adapters[channel].dev,
                "SLA+W not acknowledged: 0x%02x\n", status);
        return -EIO;
    }

    // 处理数据
    if (msg->flags & I2C_M_RD)
    {
        // 读数据
        for (i = 0; i < msg->len; i++)
        {
            // 最后一个字节发送NACK
            if (i == msg->len - 1)
            {
                i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~(I2C_CTRL_AA | I2C_CTRL_SI));
            }
            else
            {
                i2c_set_ctrl(channel, (i2c_get_ctrl(channel) | I2C_CTRL_AA) & ~I2C_CTRL_SI);
            }

            ret = i2c_wait_si(channel, "Read data");
            if (ret < 0)
                return ret;

            status = i2c_get_status(channel);
            if ((i != msg->len - 1 && status != I2C_STATUS_DATA_RECEIVED_ACK) ||
                (i == msg->len - 1 && status != I2C_STATUS_DATA_RECEIVED_NACK))
            {
                dev_err(&i2c_dev->adapters[channel].dev,
                        "Data receive error: 0x%02x\n", status);
                return -EIO;
            }

            msg->buf[i] = i2c_get_data(channel);
        }
    }
    else
    {
        // 写数据
        for (i = 0; i < msg->len; i++)
        {
            i2c_set_data(channel, msg->buf[i]);
            i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~I2C_CTRL_SI);

            ret = i2c_wait_si(channel, "Write data");
            if (ret < 0)
                return ret;

            status = i2c_get_status(channel);
            if (status != I2C_STATUS_DATA_TRANSMITTED_ACK)
            {
                dev_err(&i2c_dev->adapters[channel].dev,
                        "Data transmit error: 0x%02x\n", status);
                return -EIO;
            }
        }
    }

    // 发送STOP条件
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) | I2C_CTRL_STO);
    i2c_set_ctrl(channel, i2c_get_ctrl(channel) & ~I2C_CTRL_SI);
    usleep_range(100, 200);

    return 0;
}

// I2C传输函数，实现标准接口
static int i2c_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
    int channel = (int)(adap->nr % I2C_NUM_CHANNELS);
    int i, ret = 0;

    mutex_lock(&i2c_dev->lock[channel]);

    // 初始化控制器
    i2c_hw_init(channel);

    // 处理所有消息
    for (i = 0; i < num; i++)
    {
        ret = i2c_transfer_msg(channel, &msgs[i]);
        if (ret < 0)
            break;
    }

    mutex_unlock(&i2c_dev->lock[channel]);

    return (ret < 0) ? ret : num;
}

// 获取I2C功能
static u32 i2c_func(struct i2c_adapter *adap)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

// I2C算法结构体
static const struct i2c_algorithm i2c_algorithm = {
    .master_xfer = i2c_master_xfer,
    .functionality = i2c_func,
};

// 平台设备探针函数
static int i2c_platform_probe(struct platform_device *pdev)
{
    int i, ret;

    // 分配设备结构体内存
    i2c_dev = devm_kzalloc(&pdev->dev, sizeof(struct i2c_platform_dev), GFP_KERNEL);
    if (!i2c_dev)
        return -ENOMEM;

    // 映射I2C控制器寄存器
    for (i = 0; i < I2C_NUM_CHANNELS; i++)
    {
        i2c_dev->base_addr[i] = devm_ioremap(&pdev->dev, BASE_ADDR_I2C(i), 0x1000);
        if (!i2c_dev->base_addr[i])
        {
            dev_err(&pdev->dev, "Failed to ioremap I2C channel %d\n", i + 1);
            return -ENOMEM;
        }

        // 初始化互斥锁
        mutex_init(&i2c_dev->lock[i]);

        // 初始化I2C适配器
        i2c_dev->adapters[i].owner = THIS_MODULE;
        i2c_dev->adapters[i].class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
        i2c_dev->adapters[i].algo = &i2c_algorithm;
        i2c_dev->adapters[i].nr = i;
        snprintf(i2c_dev->adapters[i].name, sizeof(i2c_dev->adapters[i].name),
                 "i2c-controller-%d", i + 1);

        // 注册I2C适配器
        ret = i2c_add_adapter(&i2c_dev->adapters[i]);
        if (ret < 0)
        {
            dev_err(&pdev->dev, "Failed to add I2C adapter %d\n", i + 1);
            // 清理已注册的适配器
            for (i--; i >= 0; i--)
                i2c_del_adapter(&i2c_dev->adapters[i]);
            return ret;
        }

        dev_info(&pdev->dev, "I2C adapter %d registered at 0x%lx\n",
                 i + 1, (unsigned long)BASE_ADDR_I2C(i));
    }

    platform_set_drvdata(pdev, i2c_dev);
    return 0;
}

// 平台设备移除函数
static int i2c_platform_remove(struct platform_device *pdev)
{
    int i;

    for (i = 0; i < I2C_NUM_CHANNELS; i++)
    {
        i2c_del_adapter(&i2c_dev->adapters[i]);
        mutex_destroy(&i2c_dev->lock[i]);
    }

    return 0;
}

// 平台设备匹配表
static const struct of_device_id i2c_of_match[] = {
    {
        .compatible = "vendor,i2c-controller",
    },
    {/* sentinel */}};
MODULE_DEVICE_TABLE(of, i2c_of_match);

// 平台驱动结构体
static struct platform_driver i2c_platform_driver = {
    .driver = {
        .name = "i2c-controller",
        .of_match_table = i2c_of_match,
    },
    .probe = i2c_platform_probe,
    .remove = i2c_platform_remove,
};

module_platform_driver(i2c_platform_driver);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Standard I2C Controller Driver for i2c-tools");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
