#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/ioport.h>

// // 替换为实际PCIe设备的Vendor ID和Device ID
// #define PCI_VENDOR_ID_CUSTOM 0x1234
// #define PCI_DEVICE_ID_CUSTOM_I2C 0x5678

// 假设PCIe设备的BAR0对应I2C控制器寄存器（根据硬件实际情况调整）
#define I2C_PCIE_BAR 1
#define I2C_REG_SIZE 0x1000 // BAR空间大小（根据硬件调整）

// 寄存器地址（相对于BAR的偏移,根据硬件手册调整）
#define I2C_CTRL_REG 0x00
#define I2C_STAT_REG 0x04
#define I2C_DATA_REG 0x08

// 控制位和状态码定义（根据硬件手册调整）
#define I2C_CTRL_CR2 (1 << 7)
#define I2C_CTRL_ENS1 (1 << 6)
#define I2C_CTRL_STA (1 << 5)
#define I2C_CTRL_STO (1 << 4)
#define I2C_CTRL_SI (1 << 3)
#define I2C_CTRL_AA (1 << 2)
#define I2C_CTRL_CR1 (1 << 1)
#define I2C_CTRL_CR0 (1 << 0)

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

// 设备私有数据结构（关联PCIe设备和I2C适配器）
struct i2c_pcie_dev
{
    int current_channel;
    struct pci_dev *pci_dev;    // PCIe设备指针
    void __iomem *base_addr;    // 映射后的I2C控制器寄存器地址
    struct i2c_adapter adapter; // I2C适配器
    struct mutex lock;          // 互斥锁
};

// 寄存器操作函数（基于PCIe映射的地址）
static uint8_t i2c_get_reg(struct i2c_pcie_dev *dev, uint8_t offset)
{
    return readb(dev->base_addr + offset);
}

static void i2c_set_reg(struct i2c_pcie_dev *dev, uint8_t offset, uint8_t val)
{
    writeb(val, dev->base_addr + offset);
}

static uint8_t i2c_get_ctrl(struct i2c_pcie_dev *dev)
{
    return i2c_get_reg(dev, I2C_CTRL_REG);
}

static void i2c_set_ctrl(struct i2c_pcie_dev *dev, uint8_t val)
{
    i2c_set_reg(dev, I2C_CTRL_REG, val);
}

static uint8_t i2c_get_data(struct i2c_pcie_dev *dev)
{
    return i2c_get_reg(dev, I2C_DATA_REG);
}

static void i2c_set_data(struct i2c_pcie_dev *dev, uint8_t val)
{
    i2c_set_reg(dev, I2C_DATA_REG, val);
}

static uint8_t i2c_get_status(struct i2c_pcie_dev *dev)
{
    return i2c_get_reg(dev, I2C_STAT_REG);
}

// 等待SI位设置
static int i2c_wait_si(struct i2c_pcie_dev *dev, const char *step)
{
    int timeout = 10000;
    while (!(i2c_get_ctrl(dev) & I2C_CTRL_SI))
    {
        if (--timeout <= 0)
        {
            dev_err(&dev->pci_dev->dev, "%s: Timeout waiting for SI\n", step);
            return -ETIMEDOUT;
        }
        usleep_range(10, 20);
    }
    return 0;
}

// 初始化I2C控制器硬件
static void i2c_hw_init(struct i2c_pcie_dev *dev)
{
    // 重置控制器
    i2c_set_ctrl(dev, 0x00);
    udelay(10);

    // 使能I2C控制器（根据硬件手册调整控制位）
    i2c_set_ctrl(dev, I2C_CTRL_ENS1 | I2C_CTRL_CR2 |
                          I2C_CTRL_CR1 | I2C_CTRL_CR0);
    udelay(10);
}
// 传输单个I2C消息（包含寄存器地址发送步骤）
static int i2c_transfer_msg(struct i2c_pcie_dev *dev, struct i2c_msg *msg)
{
    int ret;
    int i;
    uint8_t status;
    uint8_t reg_addr; // 寄存器地址

    // 检查消息长度：至少需要包含1个字节的寄存器地址
    if (msg->len < 1)
    {
        dev_err(&dev->pci_dev->dev, "消息长度错误,至少需要1字节寄存器地址\n");
        return -EINVAL;
    }

    // 提取寄存器地址（msg->buf[0]固定为寄存器地址）
    reg_addr = msg->buf[0];
    // 数据部分从msg->buf[1]开始,长度为msg->len - 1
    uint8_t *data_buf = &msg->buf[1];
    uint32_t data_len = msg->len - 1;

    // 1. 发送START条件
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) | I2C_CTRL_STA);
    ret = i2c_wait_si(dev, "发送START条件");
    if (ret < 0)
        return ret;

    // 2. 发送设备地址（先以写模式发送,用于传输寄存器地址）
    i2c_set_data(dev, (msg->addr << 1) & 0xFE);                           // 写模式（最低位0）
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~(I2C_CTRL_STA | I2C_CTRL_SI)); // 清除START和SI

    // 等待设备地址ACK
    ret = i2c_wait_si(dev, "等待设备地址ACK");
    if (ret < 0)
        return ret;

    status = i2c_get_status(dev);
    if (status != I2C_STATUS_SLA_W_ACK)
    {
        dev_err(&dev->pci_dev->dev, "设备地址写模式未ACK: 0x%02x\n", status);
        return -EIO;
    }

    // 3. 发送寄存器地址（关键步骤：补充寄存器地址发送）
    i2c_set_data(dev, reg_addr);
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~I2C_CTRL_SI); // 清除SI
    ret = i2c_wait_si(dev, "发送寄存器地址");
    if (ret < 0)
        return ret;

    status = i2c_get_status(dev);
    if (status != I2C_STATUS_DATA_TRANSMITTED_ACK)
    {
        dev_err(&dev->pci_dev->dev, "寄存器地址发送未ACK: 0x%02x\n", status);
        return -EIO;
    }

    // 4. 处理数据传输（读/写）
    if (msg->flags & I2C_M_RD)
    {
        // 读操作：需要发送重复START,切换为读模式

        // 4.1 发送重复START
        i2c_set_ctrl(dev, i2c_get_ctrl(dev) | I2C_CTRL_STA); // 重复START
        i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~I2C_CTRL_SI); // 清除SI
        ret = i2c_wait_si(dev, "发送重复START");
        if (ret < 0)
            return ret;

        status = i2c_get_status(dev);
        if (status != I2C_STATUS_REPEATED_START_TRANSMITTED)
        {
            dev_err(&dev->pci_dev->dev, "重复START失败: 0x%02x\n", status);
            return -EIO;
        }

        // 4.2 发送设备地址（读模式）
        i2c_set_data(dev, (msg->addr << 1) | 0x01);                           // 读模式（最低位1）
        i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~(I2C_CTRL_STA | I2C_CTRL_SI)); // 清除START和SI
        ret = i2c_wait_si(dev, "等待读模式地址ACK");
        if (ret < 0)
            return ret;

        status = i2c_get_status(dev);
        if (status != I2C_STATUS_SLA_R_ACK)
        {
            dev_err(&dev->pci_dev->dev, "设备地址读模式未ACK: 0x%02x\n", status);
            return -EIO;
        }

        // 4.3 读取数据（存储到msg->buf[1..n]）
        for (i = 0; i < data_len; i++)
        {
            // 最后一个字节发送NACK,其他发送ACK
            if (i == data_len - 1)
            {
                i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~(I2C_CTRL_AA | I2C_CTRL_SI));
            }
            else
            {
                i2c_set_ctrl(dev, (i2c_get_ctrl(dev) | I2C_CTRL_AA) & ~I2C_CTRL_SI);
            }

            ret = i2c_wait_si(dev, "读取数据");
            if (ret < 0)
                return ret;

            status = i2c_get_status(dev);
            if ((i != data_len - 1 && status != I2C_STATUS_DATA_RECEIVED_ACK) ||
                (i == data_len - 1 && status != I2C_STATUS_DATA_RECEIVED_NACK))
            {
                dev_err(&dev->pci_dev->dev, "数据读取错误: 0x%02x\n", status);
                return -EIO;
            }

            data_buf[i] = i2c_get_data(dev); // 存储到msg->buf[1..]
        }
    }
    else
    {
        // 写操作：直接发送数据（从msg->buf[1]开始）
        for (i = 0; i < data_len; i++)
        {
            i2c_set_data(dev, data_buf[i]);                      // 发送数据
            i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~I2C_CTRL_SI); // 清除SI

            ret = i2c_wait_si(dev, "发送数据");
            if (ret < 0)
                return ret;

            status = i2c_get_status(dev);
            if (status != I2C_STATUS_DATA_TRANSMITTED_ACK)
            {
                dev_err(&dev->pci_dev->dev, "数据发送错误: 0x%02x\n", status);
                return -EIO;
            }
        }
    }

    // 5. 发送STOP条件
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) | I2C_CTRL_STO);
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~I2C_CTRL_SI); // 清除SI
    ret = i2c_wait_si(dev, "等待STOP完成");
    if (ret < 0)
        return ret;

    // 等待STOP状态确认
    ret = 1000;
    while (--ret > 0)
    {
        if (i2c_get_status(dev) == I2C_STATUS_STOP_TRANSMITTED)
            break;
        usleep_range(10, 20);
    }
    if (ret <= 0)
    {
        dev_warn(&dev->pci_dev->dev, "等待STOP超时\n");
    }

    // 清除SI,为下一次操作准备
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~I2C_CTRL_SI);

    return 0;
}

// I2C读取任意长度字节（使用dev结构体,替代channel参数）
static int i2c_master_read_bytes(struct i2c_pcie_dev *dev, uint8_t dev_addr,
                                 uint8_t reg_addr, uint8_t *data_buf, uint32_t len)
{
    int ret;
    int i;
    uint8_t status;

    // 参数合法性检查
    if (!dev || !data_buf || len == 0)
        return -EINVAL;

    mutex_lock(&dev->lock);

    // 1. 发送START条件（通过dev操作当前通道）
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) | I2C_CTRL_STA);
    ret = i2c_wait_si(dev, "Read start condition");
    if (ret < 0)
        goto out;

    // 2. 发送设备地址(写模式)
    i2c_set_data(dev, (dev_addr << 1) & 0xFE); // 写模式（最低位0）
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~(I2C_CTRL_STA | I2C_CTRL_SI));
    ret = i2c_wait_si(dev, "Send device address (write mode)");
    if (ret < 0)
        goto out;

    status = i2c_get_status(dev);
    if (status != I2C_STATUS_SLA_W_ACK)
    {
        printk("[ERROR] [SLA+W STATUS] 通道%d, 状态=0x%02X\n",
               dev->current_channel, status);
        ret = -EIO;
        goto out;
    }

    // 3. 发送寄存器地址
    i2c_set_data(dev, reg_addr);
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~I2C_CTRL_SI);
    ret = i2c_wait_si(dev, "Write register address");
    if (ret < 0)
        goto out;

    status = i2c_get_status(dev);
    if (status != I2C_STATUS_DATA_TRANSMITTED_ACK)
    {
        dev_err(&dev->pci_dev->dev,
                "通道%d: 寄存器地址未ACK (0x%02X)\n",
                dev->current_channel, status);
        ret = -EIO;
        goto out;
    }

    // 4. 发送重复START
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) | I2C_CTRL_STA);
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~I2C_CTRL_SI);
    ret = i2c_wait_si(dev, "Repeated start");
    if (ret < 0)
        goto out;

    status = i2c_get_status(dev);
    if (status != I2C_STATUS_REPEATED_START_TRANSMITTED)
    {
        dev_err(&dev->pci_dev->dev,
                "通道%d: 重复START失败 (0x%02X)\n",
                dev->current_channel, status);
        ret = -EIO;
        goto out;
    }

    // 5. 发送设备地址(读模式)
    i2c_set_data(dev, (dev_addr << 1) | 0x01); // 读模式（最低位1）
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~(I2C_CTRL_STA | I2C_CTRL_SI));
    ret = i2c_wait_si(dev, "Send device address (read mode)");
    if (ret < 0)
        goto out;

    status = i2c_get_status(dev);
    if (status != I2C_STATUS_SLA_R_ACK)
    {
        dev_err(&dev->pci_dev->dev,
                "通道%d: SLA+R未ACK (0x%02X)\n",
                dev->current_channel, status);
        ret = -EIO;
        goto out;
    }

    // 6. 循环读取数据
    for (i = 0; i < len; i++)
    {
        // 最后一个字节发送NACK,其他发送ACK
        if (i == len - 1)
        {
            i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~(I2C_CTRL_AA | I2C_CTRL_SI));
        }
        else
        {
            i2c_set_ctrl(dev, (i2c_get_ctrl(dev) | I2C_CTRL_AA) & ~I2C_CTRL_SI);
        }

        ret = i2c_wait_si(dev, "Read data");
        if (ret < 0)
            goto out;

        status = i2c_get_status(dev);
        if ((i != len - 1 && status != I2C_STATUS_DATA_RECEIVED_ACK) ||
            (i == len - 1 && status != I2C_STATUS_DATA_RECEIVED_NACK))
        {
            printk("[ERROR] [DATA %d STATUS] 通道%d, 状态=0x%02X\n",
                   i, dev->current_channel, status);
            ret = -EIO;
            goto out;
        }

        data_buf[i] = i2c_get_data(dev);
    }

    // 7. 发送STOP条件
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) | I2C_CTRL_STO);
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~I2C_CTRL_SI);
    ret = i2c_wait_si(dev, "Send stop condition");
    if (ret < 0)
        goto out;

    // 等待STOP完成
    ret = 1000;
    while (--ret > 0)
    {
        if (i2c_get_status(dev) == I2C_STATUS_STOP_TRANSMITTED)
            break;
        usleep_range(10, 20);
    }

    if (ret <= 0)
    {
        dev_warn(&dev->pci_dev->dev, "通道%d: 等待STOP超时\n", dev->current_channel);
    }

    // 清除SI信号,为下一次操作做准备
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~I2C_CTRL_SI);
    ret = 0; // 成功

out:
    mutex_unlock(&dev->lock);
    return ret;
}

// I2C写入任意长度字节（使用dev结构体,替代channel参数）
static int i2c_master_write_bytes(struct i2c_pcie_dev *dev, uint8_t dev_addr,
                                  uint8_t reg_addr, const uint8_t *data_buf, uint32_t len)
{
    int ret;
    int i;
    uint8_t status;

    // 参数合法性检查
    if (!dev || !data_buf || len == 0)
        return -EINVAL;

    mutex_lock(&dev->lock);

    // 1. 发送START条件
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) | I2C_CTRL_STA);
    ret = i2c_wait_si(dev, "Write start condition");
    if (ret < 0)
        goto out;

    // 2. 发送设备地址(写模式)
    i2c_set_data(dev, (dev_addr << 1) & 0xFE); // 写模式（最低位0）
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~(I2C_CTRL_STA | I2C_CTRL_SI));
    ret = i2c_wait_si(dev, "Send device address (write mode)");
    if (ret < 0)
        goto out;

    status = i2c_get_status(dev);
    if (status != I2C_STATUS_SLA_W_ACK)
    {
        printk("[ERROR] [SLA+W STATUS] 通道%d, 状态=0x%02X\n",
               dev->current_channel, status);
        ret = -EIO;
        goto out;
    }

    // 3. 发送寄存器地址
    i2c_set_data(dev, reg_addr);
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~I2C_CTRL_SI);
    ret = i2c_wait_si(dev, "Write register address");
    if (ret < 0)
        goto out;

    status = i2c_get_status(dev);
    if (status != I2C_STATUS_DATA_TRANSMITTED_ACK)
    {
        dev_err(&dev->pci_dev->dev,
                "通道%d: 寄存器地址未ACK (0x%02X)\n",
                dev->current_channel, status);
        ret = -EIO;
        goto out;
    }

    // 4. 循环发送数据
    for (i = 0; i < len; i++)
    {
        i2c_set_data(dev, data_buf[i]);
        if (i == 0)
            printk("通道%d: 首字节数据=0x%02x\n", dev->current_channel, data_buf[i]);

        i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~I2C_CTRL_SI);
        ret = i2c_wait_si(dev, "Write data");
        if (ret < 0)
            goto out;

        status = i2c_get_status(dev);
        if (status != I2C_STATUS_DATA_TRANSMITTED_ACK)
        {
            dev_err(&dev->pci_dev->dev,
                    "通道%d: 数据传输错误 (0x%02x)\n",
                    dev->current_channel, status);
            ret = -EIO;
            goto out;
        }
    }
    printk("通道%d: 发送%d字节数据成功\n", dev->current_channel, len);

    // 5. 发送STOP条件
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) | I2C_CTRL_STO);
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~I2C_CTRL_SI);
    ret = i2c_wait_si(dev, "STOP ACK");
    if (ret < 0)
        goto out;

    // 等待STOP完成
    ret = 1000;
    while (--ret > 0)
    {
        if (i2c_get_status(dev) == I2C_STATUS_STOP_TRANSMITTED)
            break;
        usleep_range(10, 20);
    }

    if (ret <= 0)
    {
        dev_warn(&dev->pci_dev->dev, "通道%d: 等待STOP超时\n", dev->current_channel);
    }

    // 清除SI信号,为下一次操作做准备
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~I2C_CTRL_SI);
    ret = 0; // 成功

out:
    mutex_unlock(&dev->lock);
    return ret;
}

// I2C传输函数（标准接口,拆分读写操作）
static int i2c_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
    struct i2c_pcie_dev *dev = i2c_get_adapdata(adap);
    int i, ret = 0;
    uint8_t reg_addr;         // 寄存器地址
    const uint8_t *write_buf; // 写数据缓冲区
    uint8_t *read_buf;        // 读数据缓冲区
    uint32_t data_len;        // 数据长度（不含寄存器地址）

    // 每次传输前初始化硬件
    i2c_hw_init(dev);

    printk("i2c transfer: processing %d messages\n", num);

    for (i = 0; i < num; i++)
    {
        // 检查消息长度：至少包含1字节寄存器地址
        if (msgs[i].len < 1)
        {
            dev_err(&dev->pci_dev->dev, "消息长度错误,至少需要1字节寄存器地址\n");
            ret = -EINVAL;
            break;
        }

        // 提取寄存器地址（msg->buf[0]固定为寄存器地址）
        reg_addr = msgs[i].buf[0];
        data_len = msgs[i].len - 1;

        if (msgs[i].flags & I2C_M_RD)
        {
            // 读操作：调用读接口
            read_buf = &msgs[i].buf[1]; // 读数据存储位置（跳过寄存器地址）

            printk("读操作: 设备0x%02x, 寄存器0x%02x, 长度%d\n",
                   msgs[i].addr, reg_addr, data_len);

            ret = i2c_master_read_bytes(
                dev,          // 当前
                msgs[i].addr, // 设备地址
                reg_addr,     // 寄存器地址
                read_buf,     // 接收缓冲区
                data_len      // 数据长度
            );
        }
        else
        {
            // 写操作：调用写接口
            write_buf = &msgs[i].buf[1]; // 写数据起始位置（跳过寄存器地址）

            printk("写操作: 设备0x%02x, 寄存器0x%02x, 长度%d\n",
                   msgs[i].addr, reg_addr, data_len);

            ret = i2c_master_write_bytes(
                dev,          // 当前
                msgs[i].addr, // 设备地址
                reg_addr,     // 寄存器地址
                write_buf,    // 发送缓冲区
                data_len      // 数据长度
            );
        }

        // 检查操作结果
        if (ret < 0)
        {
            dev_err(&dev->pci_dev->dev,
                    "消息%d处理失败, 错误码: %d\n", i, ret);
            break;
        }
    }

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

// PCIe设备探针函数（核心：获取PCIe映射的地址）
static int i2c_pcie_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct i2c_pcie_dev *dev;
    int ret;

    // 分配设备私有数据
    dev = devm_kzalloc(&pdev->dev, sizeof(struct i2c_pcie_dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;

    dev->pci_dev = pdev;
    pci_set_drvdata(pdev, dev);

    // 启用PCIe设备
    ret = pci_enable_device(pdev);
    if (ret)
    {
        dev_err(&pdev->dev, "Failed to enable PCIe device: %d\n", ret);
        return ret;
    }

    // 申请PCIe BAR资源（I2C控制器寄存器空间）
    ret = pci_request_region(pdev, I2C_PCIE_BAR, "i2c-pcie-regs");
    if (ret)
    {
        dev_err(&pdev->dev, "Failed to request PCIe BAR%d: %d\n", I2C_PCIE_BAR, ret);
        goto disable_pci;
    }

    // 映射PCIe BAR物理地址到虚拟地址
    dev->base_addr = pci_ioremap_bar(pdev, I2C_PCIE_BAR);
    if (!dev->base_addr)
    {
        dev_err(&pdev->dev, "Failed to ioremap PCIe BAR%d\n", I2C_PCIE_BAR);
        ret = -ENOMEM;
        goto release_region;
    }

    // 初始化互斥锁
    mutex_init(&dev->lock);

    // 初始化I2C适配器
    strscpy(dev->adapter.name, "i2c-over-pcie", sizeof(dev->adapter.name));
    dev->adapter.owner = THIS_MODULE;
    dev->adapter.class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
    dev->adapter.algo = &i2c_algorithm;
    dev->adapter.dev.parent = &pdev->dev;
    dev->adapter.nr = -1; // 自动分配编号
    i2c_set_adapdata(&dev->adapter, dev);

    // 注册I2C适配器
    ret = i2c_add_adapter(&dev->adapter);
    if (ret)
    {
        dev_err(&pdev->dev, "Failed to add I2C adapter: %d\n", ret);
        goto iounmap_bar;
    }

    dev_info(&pdev->dev, "PCIe I2C controller registered as /dev/i2c-%d\n",
             dev->adapter.nr);
    return 0;

iounmap_bar:
    iounmap(dev->base_addr);
release_region:
    pci_release_region(pdev, I2C_PCIE_BAR);
disable_pci:
    pci_disable_device(pdev);
    return ret;
}

// PCIe设备移除函数
static void i2c_pcie_remove(struct pci_dev *pdev)
{
    struct i2c_pcie_dev *dev = pci_get_drvdata(pdev);

    // 注销I2C适配器
    i2c_del_adapter(&dev->adapter);

    // 清理资源
    iounmap(dev->base_addr);
    pci_release_region(pdev, I2C_PCIE_BAR);
    pci_disable_device(pdev);
    mutex_destroy(&dev->lock);

    dev_info(&pdev->dev, "PCIe I2C controller removed\n");
}
#define PCI_VENDOR_ID_CUSTOM 0x11aa
#define PCI_DEVICE_ID_CUSTOM_I2C 0x11aa
// PCIe设备ID表（用于匹配设备）
static const struct pci_device_id i2c_pcie_ids[] = {
    {PCI_DEVICE(PCI_VENDOR_ID_CUSTOM, PCI_DEVICE_ID_CUSTOM_I2C)},
    {0} // 终止符
};
MODULE_DEVICE_TABLE(pci, i2c_pcie_ids);

// PCIe驱动结构体
static struct pci_driver i2c_pcie_driver = {
    .name = "i2c-over-pcie",
    .id_table = i2c_pcie_ids,
    .probe = i2c_pcie_probe,
    .remove = i2c_pcie_remove,
};

module_pci_driver(i2c_pcie_driver);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("I2C Driver for PCIe-connected External I2C Controller");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
