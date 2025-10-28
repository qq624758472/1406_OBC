#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/can/dev.h>
#include <linux/can/core.h>

#define FPGA_VENDOR_ID 0x11aa
#define FPGA_DEVICE_ID 0x11aa

// 手动定义 can_bittiming_const 和 can_data_bittiming_const
static const struct can_bittiming_const can_bittiming_const = {
    .name = "fpga_canfd",
    .tseg1_min = 1,
    .tseg1_max = 16,
    .tseg2_min = 1,
    .tseg2_max = 8,
    .sjw_max = 4,
    .brp_min = 1,
    .brp_max = 1024,
    .brp_inc = 1,
};

static const struct can_bittiming_const can_data_bittiming_const = {
    .name = "fpga_canfd_data",
    .tseg1_min = 1,
    .tseg1_max = 16,
    .tseg2_min = 1,
    .tseg2_max = 8,
    .sjw_max = 4,
    .brp_min = 1,
    .brp_max = 1024,
    .brp_inc = 1,
};

static int fpga_canfd_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct net_device *dev;
    struct can_priv *priv;
    int ret;

    dev = alloc_candev(sizeof(*priv), 1);
    if (!dev)
        return -ENOMEM;

    priv = netdev_priv(dev);
    priv->state = CAN_STATE_STOPPED;
    priv->ctrlmode_supported = CAN_CTRLMODE_LISTENONLY |
                               CAN_CTRLMODE_LOOPBACK |
                               CAN_CTRLMODE_FD | CAN_CTRLMODE_BERR_REPORTING;
    priv->bittiming_const = &can_bittiming_const;           // 使用手动定义的 can_bittiming_const
    priv->data_bittiming_const = &can_data_bittiming_const; // 使用手动定义的 can_data_bittiming_const

    ret = pci_enable_device(pdev);
    if (ret)
    {
        goto err_free_dev;
    }

    pci_set_drvdata(pdev, dev);

    ret = register_candev(dev);
    if (ret)
    {
        goto err_disable_pci;
    }

    printk(KERN_INFO "FPGA CAN FD driver loaded\n");
    return 0;

err_disable_pci:
    pci_disable_device(pdev);
err_free_dev:
    free_candev(dev);
    return ret;
}

static void fpga_canfd_remove(struct pci_dev *pdev)
{
    struct net_device *dev = pci_get_drvdata(pdev);

    unregister_candev(dev);
    pci_disable_device(pdev);
    free_candev(dev);

    printk(KERN_INFO "FPGA CAN FD driver unloaded\n");
}

static const struct pci_device_id fpga_canfd_id_table[] = {
    {PCI_DEVICE(FPGA_VENDOR_ID, FPGA_DEVICE_ID)},
    {/* end: all zeroes */}};
MODULE_DEVICE_TABLE(pci, fpga_canfd_id_table);

static struct pci_driver fpga_canfd_driver = {
    .name = "fpga_canfd",
    .id_table = fpga_canfd_id_table,
    .probe = fpga_canfd_probe,
    .remove = fpga_canfd_remove,
};

module_pci_driver(fpga_canfd_driver);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("FPGA CAN FD Driver for i.MX8DL");
MODULE_LICENSE("GPL");