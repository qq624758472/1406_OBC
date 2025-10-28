#include <linux/module.h>
#include <linux/pci.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/delay.h>

// PCI设备ID（需根据实际FPGA设备ID修改）
#define FPGA_VENDOR_ID 0x11aa
#define FPGA_DEVICE_ID 0x11aa

// I2C控制器寄存器定义
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

// I2C通道数量
#define I2C_NUM_CHANNELS 5
#define CHANNEL_OFFSET 0x1000 // 每个通道地址偏移

// 设备私有数据结构
struct fpga_i2c_dev
{
	struct pci_dev *pdev;
	void __iomem *base_addr;					   // BAR1映射的基地址
	struct i2c_adapter adapters[I2C_NUM_CHANNELS]; // 每个通道对应一个I2C适配器
	struct mutex lock;							   // 保护I2C控制器并发访问
};

// 寄存器操作封装（带通道偏移）
static u8 fpga_i2c_read_reg(struct fpga_i2c_dev *dev, int channel, u8 offset)
{
	return readb(dev->base_addr + (channel * CHANNEL_OFFSET) + offset);
}

static void fpga_i2c_write_reg(struct fpga_i2c_dev *dev, int channel, u8 offset, u8 val)
{
	writeb(val, dev->base_addr + (channel * CHANNEL_OFFSET) + offset);
}

// 等待SI标志置位（状态变化）
static int fpga_i2c_wait_si(struct fpga_i2c_dev *dev, int channel, const char *step)
{
	int timeout = 1000; // 1000ms超时
	while (!(fpga_i2c_read_reg(dev, channel, I2C_CTRL_REG) & I2C_CTRL_SI))
	{
		if (--timeout <= 0)
		{
			dev_err(&dev->pdev->dev, "%s: Timeout waiting for SI\n", step);
			return -ETIMEDOUT;
		}
		usleep_range(10, 20);
	}
	return 0;
}

// 初始化单个I2C通道控制器
static void fpga_i2c_init_channel(struct fpga_i2c_dev *dev, int channel)
{
	mutex_lock(&dev->lock);
	// 重置并使能控制器
	fpga_i2c_write_reg(dev, channel, I2C_CTRL_REG, 0x00);
	fpga_i2c_write_reg(dev, channel, I2C_CTRL_REG,
					   I2C_CTRL_ENS1 | I2C_CTRL_CR2 | I2C_CTRL_CR1 | I2C_CTRL_CR0);
	mutex_unlock(&dev->lock);
}

// I2C传输核心实现（处理单个i2c_msg）
static int fpga_i2c_xfer_msg(struct fpga_i2c_dev *dev, int channel, struct i2c_msg *msg)
{
	int ret;
	u8 addr = msg->addr << 1; // I2C地址（含读写位）
	int i;

	mutex_lock(&dev->lock);

	// 1. 发送START条件
	fpga_i2c_write_reg(dev, channel, I2C_CTRL_REG,
					   fpga_i2c_read_reg(dev, channel, I2C_CTRL_REG) | I2C_CTRL_STA);
	ret = fpga_i2c_wait_si(dev, channel, "START condition");
	if (ret)
		goto out;

	// 2. 发送设备地址（含读写位）
	fpga_i2c_write_reg(dev, channel, I2C_DATA_REG, (msg->flags & I2C_M_RD) ? (addr | 1) : addr);
	fpga_i2c_write_reg(dev, channel, I2C_CTRL_REG,
					   fpga_i2c_read_reg(dev, channel, I2C_CTRL_REG) & ~(I2C_CTRL_STA | I2C_CTRL_SI));
	ret = fpga_i2c_wait_si(dev, channel, "Address transmission");
	if (ret)
		goto out;

	// 检查地址应答状态
	if (msg->flags & I2C_M_RD)
	{
		if (fpga_i2c_read_reg(dev, channel, I2C_STAT_REG) != I2C_STATUS_SLA_R_ACK)
		{
			dev_err(&dev->pdev->dev, "SLA+R NACK (0x%02x)\n",
					fpga_i2c_read_reg(dev, channel, I2C_STAT_REG));
			ret = -EIO;
			goto out;
		}
	}
	else
	{
		if (fpga_i2c_read_reg(dev, channel, I2C_STAT_REG) != I2C_STATUS_SLA_W_ACK)
		{
			dev_err(&dev->pdev->dev, "SLA+W NACK (0x%02x)\n",
					fpga_i2c_read_reg(dev, channel, I2C_STAT_REG));
			ret = -EIO;
			goto out;
		}
	}

	// 3. 处理数据传输
	if (msg->flags & I2C_M_RD)
	{ // 读操作
		for (i = 0; i < msg->len; i++)
		{
			// 最后一个字节发送NACK，其他发送ACK
			if (i == msg->len - 1)
			{
				fpga_i2c_write_reg(dev, channel, I2C_CTRL_REG,
								   fpga_i2c_read_reg(dev, channel, I2C_CTRL_REG) &
									   ~(I2C_CTRL_AA | I2C_CTRL_SI));
			}
			else
			{
				fpga_i2c_write_reg(dev, channel, I2C_CTRL_REG,
								   (fpga_i2c_read_reg(dev, channel, I2C_CTRL_REG) | I2C_CTRL_AA) &
									   ~I2C_CTRL_SI);
			}

			ret = fpga_i2c_wait_si(dev, channel, "Data reception");
			if (ret)
				goto out;

			// 读取数据
			msg->buf[i] = fpga_i2c_read_reg(dev, channel, I2C_DATA_REG);

			// 检查状态
			if (i == msg->len - 1)
			{
				if (fpga_i2c_read_reg(dev, channel, I2C_STAT_REG) != I2C_STATUS_DATA_RECEIVED_NACK)
				{
					dev_err(&dev->pdev->dev, "Last data NACK err (0x%02x)\n",
							fpga_i2c_read_reg(dev, channel, I2C_STAT_REG));
					ret = -EIO;
					goto out;
				}
			}
			else
			{
				if (fpga_i2c_read_reg(dev, channel, I2C_STAT_REG) != I2C_STATUS_DATA_RECEIVED_ACK)
				{
					dev_err(&dev->pdev->dev, "Data ACK err (0x%02x)\n",
							fpga_i2c_read_reg(dev, channel, I2C_STAT_REG));
					ret = -EIO;
					goto out;
				}
			}
		}
	}
	else
	{ // 写操作
		for (i = 0; i < msg->len; i++)
		{
			// 发送数据
			fpga_i2c_write_reg(dev, channel, I2C_DATA_REG, msg->buf[i]);
			fpga_i2c_write_reg(dev, channel, I2C_CTRL_REG,
							   fpga_i2c_read_reg(dev, channel, I2C_CTRL_REG) & ~I2C_CTRL_SI);

			ret = fpga_i2c_wait_si(dev, channel, "Data transmission");
			if (ret)
				goto out;

			// 检查数据应答
			if (fpga_i2c_read_reg(dev, channel, I2C_STAT_REG) != I2C_STATUS_DATA_TRANSMITTED_ACK)
			{
				dev_err(&dev->pdev->dev, "Data NACK (0x%02x)\n",
						fpga_i2c_read_reg(dev, channel, I2C_STAT_REG));
				ret = -EIO;
				goto out;
			}
		}
	}

	// 4. 发送STOP条件
	fpga_i2c_write_reg(dev, channel, I2C_CTRL_REG,
					   fpga_i2c_read_reg(dev, channel, I2C_CTRL_REG) | I2C_CTRL_STO | ~I2C_CTRL_SI);
	ret = fpga_i2c_wait_si(dev, channel, "STOP condition");
	if (ret)
		goto out;

	// 等待STOP完成
	ret = 1000;
	while (--ret > 0)
	{
		if (fpga_i2c_read_reg(dev, channel, I2C_STAT_REG) == I2C_STATUS_STOP_TRANSMITTED)
			break;
		usleep_range(10, 20);
	}
	if (ret <= 0)
		dev_warn(&dev->pdev->dev, "Timeout waiting for STOP\n");

	ret = 0; // 传输成功

out:
	mutex_unlock(&dev->lock);
	return ret;
}

// I2C算法实现（Linux I2C框架要求）
static int fpga_i2c_master_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct fpga_i2c_dev *dev = i2c_get_adapdata(adap);
	int channel = adap - dev->adapters; // 计算当前通道号
	int i, ret;

	for (i = 0; i < num; i++)
	{
		ret = fpga_i2c_xfer_msg(dev, channel, &msgs[i]);
		if (ret)
			return ret;
	}
	return num; // 返回成功传输的消息数
}

// 功能描述函数
static u32 fpga_i2c_func(struct i2c_adapter *adap)
{
	// 声明支持的I2C功能
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

// I2C算法结构体（定义控制器能力）
static const struct i2c_algorithm fpga_i2c_algorithm = {
	.master_xfer = fpga_i2c_master_xfer,
	.functionality = fpga_i2c_func, // 使用函数替代直接宏定义
};

// PCI设备探测函数（设备插入时调用）
static int fpga_i2c_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct fpga_i2c_dev *dev;
	int ret, i;
	resource_size_t bar_len;

	// 1. 分配设备私有数据
	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->pdev = pdev;
	pci_set_drvdata(pdev, dev);

	// 2. 初始化互斥锁
	mutex_init(&dev->lock);

	// 3. 使能PCI设备
	ret = pci_enable_device(pdev);
	if (ret)
	{
		dev_err(&pdev->dev, "Failed to enable PCI device\n");
		return ret;
	}

	// 4. 映射BAR1地址空间（FPGA I2C控制器所在）
	bar_len = pci_resource_len(pdev, 1);							// 获取BAR1长度
	dev->base_addr = ioremap(pci_resource_start(pdev, 1), bar_len); // 映射BAR1
	if (!dev->base_addr)
	{
		dev_err(&pdev->dev, "Failed to ioremap BAR1\n");
		ret = -ENOMEM;
		goto err_disable;
	}

	// 5. 初始化每个I2C通道并注册适配器
	for (i = 0; i < I2C_NUM_CHANNELS; i++)
	{
		// 初始化硬件
		fpga_i2c_init_channel(dev, i);

		// 配置I2C适配器
		dev->adapters[i].owner = THIS_MODULE;
		dev->adapters[i].class = I2C_CLASS_HWMON | I2C_CLASS_SPD; // 支持的设备类
		dev->adapters[i].algo = &fpga_i2c_algorithm;			  // 绑定算法
		dev->adapters[i].dev.parent = &pdev->dev;				  // 父设备
		snprintf(dev->adapters[i].name, sizeof(dev->adapters[i].name),
				 "fpga-i2c-channel-%d", i);		  // 适配器名称
		i2c_set_adapdata(&dev->adapters[i], dev); // 关联私有数据

		// 注册适配器到I2C框架
		ret = i2c_add_adapter(&dev->adapters[i]);
		if (ret)
		{
			dev_err(&pdev->dev, "Failed to add adapter %d\n", i);
			goto err_unmap;
		}
		dev_info(&pdev->dev, "Registered I2C adapter %d (BAR1: 0x%llx)\n",
				 i, (unsigned long long)pci_resource_start(pdev, 1));
	}

	return 0;

err_unmap:
	iounmap(dev->base_addr);
	for (--i; i >= 0; i--)
		i2c_del_adapter(&dev->adapters[i]);
err_disable:
	pci_disable_device(pdev);
	return ret;
}

// PCI设备移除函数（设备拔出时调用）
static void fpga_i2c_pci_remove(struct pci_dev *pdev)
{
	struct fpga_i2c_dev *dev = pci_get_drvdata(pdev);
	int i;

	// 注销所有I2C适配器
	for (i = 0; i < I2C_NUM_CHANNELS; i++)
		i2c_del_adapter(&dev->adapters[i]);

	// 解除地址映射
	iounmap(dev->base_addr);

	// 禁用PCI设备
	pci_disable_device(pdev);

	dev_info(&pdev->dev, "FPGA I2C controller removed\n");
}

// PCI设备ID表（用于匹配设备）
static const struct pci_device_id fpga_i2c_pci_ids[] = {
	{PCI_DEVICE(FPGA_VENDOR_ID, FPGA_DEVICE_ID)},
	{0} // 终止符
};
MODULE_DEVICE_TABLE(pci, fpga_i2c_pci_ids);

// PCI驱动结构体
static struct pci_driver fpga_i2c_pci_driver = {
	.name = "fpga-i2c-pcie",
	.id_table = fpga_i2c_pci_ids,
	.probe = fpga_i2c_pci_probe,
	.remove = fpga_i2c_pci_remove,
};

module_pci_driver(fpga_i2c_pci_driver);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("FPGA I2C Controller PCIe Driver (Linux I2C Framework)");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
