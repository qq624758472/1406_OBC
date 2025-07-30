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
#define PCI_VENDOR_ID_CUSTOM 0x11aa
#define PCI_DEVICE_ID_CUSTOM_I2C 0x11aa

// 假设PCIe设备的BAR0对应I2C控制器寄存器（根据硬件实际情况调整）
#define I2C_PCIE_BAR 1
#define I2C_REG_SIZE 0x1000 // BAR空间大小（根据硬件调整）

// 寄存器地址（相对于BAR的偏移，根据硬件手册调整）
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

// 传输单个I2C消息
static int i2c_transfer_msg(struct i2c_pcie_dev *dev, struct i2c_msg *msg)
{
    int ret;
    int i;
    uint8_t status;

    // 发送START条件
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) | I2C_CTRL_STA);
    ret = i2c_wait_si(dev, "Start condition");
    if (ret < 0)
        return ret;

    // 发送设备地址（读/写模式）
    if (msg->flags & I2C_M_RD)
    {
        i2c_set_data(dev, (msg->addr << 1) | 0x01); // 读模式
    }
    else
    {
        i2c_set_data(dev, (msg->addr << 1) & 0xFE); // 写模式
    }

    // 清除START和SI位
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~(I2C_CTRL_STA | I2C_CTRL_SI));

    // 等待地址确认
    ret = i2c_wait_si(dev, "Address ACK");
    if (ret < 0)
        return ret;

    status = i2c_get_status(dev);
    if ((msg->flags & I2C_M_RD) && status != I2C_STATUS_SLA_R_ACK)
    {
        dev_err(&dev->pci_dev->dev,
                "SLA+R not acknowledged: 0x%02x\n", status);
        return -EIO;
    }
    if (!(msg->flags & I2C_M_RD) && status != I2C_STATUS_SLA_W_ACK)
    {
        dev_err(&dev->pci_dev->dev,
                "SLA+W not acknowledged: 0x%02x\n", status);
        return -EIO;
    }

    // 处理数据传输
    if (msg->flags & I2C_M_RD)
    {
        // 读数据
        for (i = 0; i < msg->len; i++)
        {
            // 最后一个字节发送NACK
            if (i == msg->len - 1)
            {
                i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~(I2C_CTRL_AA | I2C_CTRL_SI));
            }
            else
            {
                i2c_set_ctrl(dev, (i2c_get_ctrl(dev) | I2C_CTRL_AA) & ~I2C_CTRL_SI);
            }

            ret = i2c_wait_si(dev, "Read data");
            if (ret < 0)
                return ret;

            status = i2c_get_status(dev);
            if ((i != msg->len - 1 && status != I2C_STATUS_DATA_RECEIVED_ACK) ||
                (i == msg->len - 1 && status != I2C_STATUS_DATA_RECEIVED_NACK))
            {
                dev_err(&dev->pci_dev->dev,
                        "Data receive error: 0x%02x\n", status);
                return -EIO;
            }

            msg->buf[i] = i2c_get_data(dev);
        }
    }
    else
    {
        // 写数据
        for (i = 0; i < msg->len; i++)
        {
            i2c_set_data(dev, msg->buf[i]);
            i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~I2C_CTRL_SI);

            ret = i2c_wait_si(dev, "Write data");
            if (ret < 0)
                return ret;

            status = i2c_get_status(dev);
            if (status != I2C_STATUS_DATA_TRANSMITTED_ACK)
            {
                dev_err(&dev->pci_dev->dev,
                        "Data transmit error: 0x%02x\n", status);
                return -EIO;
            }
        }
    }

    // 发送STOP条件
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) | I2C_CTRL_STO);
    i2c_set_ctrl(dev, i2c_get_ctrl(dev) & ~I2C_CTRL_SI);
    usleep_range(100, 200);

    return 0;
}

// I2C传输函数（标准接口）
static int i2c_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
    struct i2c_pcie_dev *dev = i2c_get_adapdata(adap);
    int i, ret = 0;

    mutex_lock(&dev->lock);
    i2c_hw_init(dev); // 每次传输前初始化硬件

    for (i = 0; i < num; i++)
    {
        ret = i2c_transfer_msg(dev, &msgs[i]);
        if (ret < 0)
            break;
    }

    mutex_unlock(&dev->lock);
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
