#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>

// 定义PCI设备ID（需替换为实际设备的Vendor ID和Device ID）
// #define PCI_VENDOR_ID_EXAMPLE 0x1234
// #define PCI_DEVICE_ID_EXAMPLE 0x5678

#define PCI_VENDOR_ID_EXAMPLE 0x11aa
#define PCI_DEVICE_ID_EXAMPLE 0x11aa

// 中断处理函数：接收中断后打印日志
static irqreturn_t pcie_irq_handler(int irq, void *dev_id)
{
    struct pci_dev *pdev = (struct pci_dev *)dev_id;

    // 打印中断信息（包含设备地址）
    dev_info(&pdev->dev, "PCIe中断触发！IRQ: %d\n", irq);

    return IRQ_HANDLED; // 表示中断已处理
}

// PCI设备probe函数：初始化设备并注册中断
static int pcie_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    int ret;
    int irq;

    // 启用PCI设备
    ret = pci_enable_device(pdev);
    if (ret)
    {
        dev_err(&pdev->dev, "无法启用PCI设备\n");
        return ret;
    }

    // 分配MSI中断向量（1个向量）
    ret = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI);
    if (ret < 0)
    {
        dev_err(&pdev->dev, "无法分配MSI中断向量\n");
        goto disable_device;
    }

    // 获取中断号
    irq = pci_irq_vector(pdev, 0);
    if (irq < 0)
    {
        dev_err(&pdev->dev, "无法获取中断号\n");
        ret = irq;
        goto free_irq_vectors;
    }

    // 注册中断处理函数
    ret = request_irq(irq,
                      pcie_irq_handler,
                      0, // 中断触发方式由硬件决定（MSI无需指定）
                      "pcie_example_irq",
                      pdev); // 传递pdev作为dev_id
    if (ret)
    {
        dev_err(&pdev->dev, "中断注册失败: %d\n", ret);
        goto free_irq_vectors;
    }

    dev_info(&pdev->dev, "PCIe设备初始化完成，中断号: %d\n", irq);
    return 0;

free_irq_vectors:
    pci_free_irq_vectors(pdev);
disable_device:
    pci_disable_device(pdev);
    return ret;
}

// PCI设备remove函数：释放资源
static void pcie_remove(struct pci_dev *pdev)
{
    int irq = pci_irq_vector(pdev, 0);

    // 释放中断
    free_irq(irq, pdev);
    // 释放中断向量
    pci_free_irq_vectors(pdev);
    // 禁用设备
    pci_disable_device(pdev);

    dev_info(&pdev->dev, "PCIe设备已移除\n");
}

// PCI设备ID表（用于匹配设备）
static const struct pci_device_id pcie_id_table[] = {
    {PCI_DEVICE(PCI_VENDOR_ID_EXAMPLE, PCI_DEVICE_ID_EXAMPLE)},
    {0} // 终止符
};
MODULE_DEVICE_TABLE(pci, pcie_id_table);

// PCI驱动结构体
static struct pci_driver pcie_driver = {
    .name = "pcie_irq_example",
    .id_table = pcie_id_table,
    .probe = pcie_probe,
    .remove = pcie_remove,
};

module_pci_driver(pcie_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("PCIe设备中断处理示例驱动");
MODULE_AUTHOR("Your Name");
