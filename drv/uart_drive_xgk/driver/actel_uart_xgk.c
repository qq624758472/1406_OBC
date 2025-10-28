// SPDX-License-Identifier: GPL-2.0
/*
 * GH1406 UART PCIe Driver
 * Fixed UART logic + PCIe BAR0-based memory access
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/timer.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/delay.h>

#define GH1406_UART_PORTS 5
#define UART_PORT_OFFSET 0x10000

// UART寄存器偏移
#define UART_TX_REG 0x00
#define UART_RX_REG 0x04
#define UART_CTRL1_REG 0x08
#define UART_CTRL2_REG 0x0C
#define UART_STATUS_REG 0x10

#define UART_CTRL_PARITY_EN (1 << 1)
#define UART_CTRL_ODD_PARITY (1 << 2) // ODD_N_EVEN = 1

// 状态位定义
#define UART_STATUS_TX_READY 0x01
#define UART_STATUS_RX_READY 0x02

// 系统时钟频率
#define SYS_CLK 20000000UL
// 轮询间隔 (ms) - 降低CPU占用
#define UART_POLL_INTERVAL 1

#define GH1406_UART_BAR_IDX 0
#define GH1406_UART_BAR_OFFSET 0x60000

struct gh1406_uart_port
{
    struct uart_port port;
    void __iomem *base;           // 端口基地址
    int port_id;                  // 端口号(0-5)
    struct timer_list poll_timer; // 轮询定时器
    bool baud_configured;
};

struct gh1406_uart_dev
{
    void __iomem *base; // UART模块基地址
    struct gh1406_uart_port ports[GH1406_UART_PORTS];
};

static struct gh1406_uart_dev *gh1406_uart_dev;

static inline u8 gh1406_uart_readb(struct gh1406_uart_port *uport, u32 offset)
{
    return readb(uport->base + offset);
}

static inline void gh1406_uart_writeb(struct gh1406_uart_port *uport, u32 offset, u8 val)
{
    writeb(val, uport->base + offset);
}

static void gh1406_uart_poll_rx(struct timer_list *t)
{
    struct gh1406_uart_port *uport = container_of(t, struct gh1406_uart_port, poll_timer);
    struct uart_port *port = &uport->port;
    struct tty_port *tty_port = &port->state->port;
    u8 status, ch;
    int max_count = 128;

    while (max_count-- > 0)
    {
        status = gh1406_uart_readb(uport, UART_STATUS_REG);
        if (!(status & UART_STATUS_RX_READY))
            break;
        ch = gh1406_uart_readb(uport, UART_RX_REG);
        port->icount.rx++;
        tty_insert_flip_char(tty_port, ch, TTY_NORMAL);
    }
    tty_flip_buffer_push(tty_port);
    mod_timer(&uport->poll_timer, jiffies + msecs_to_jiffies(UART_POLL_INTERVAL));
}

static unsigned int gh1406_uart_tx_empty(struct uart_port *port)
{
    struct gh1406_uart_port *uport = container_of(port, struct gh1406_uart_port, port);
    return (gh1406_uart_readb(uport, UART_STATUS_REG) & UART_STATUS_TX_READY) ? TIOCSER_TEMT : 0;
}

static void gh1406_uart_set_mctrl(struct uart_port *port, unsigned int mctrl) {}
static unsigned int gh1406_uart_get_mctrl(struct uart_port *port) { return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS; }
static void gh1406_uart_stop_tx(struct uart_port *port) {}

static void gh1406_uart_start_tx(struct uart_port *port)
{
    struct gh1406_uart_port *uport = container_of(port, struct gh1406_uart_port, port);
    struct circ_buf *xmit = &port->state->xmit;

    if (!uport->baud_configured)
    {
        pr_warn_once("gh1406-uart: baud rate not configured yet, skipping tx\n");
        return;
    }

    while (!uart_circ_empty(xmit))
    {
        if (!(gh1406_uart_readb(uport, UART_STATUS_REG) & UART_STATUS_TX_READY))
        {
            udelay(100);
            continue;
        }

        gh1406_uart_writeb(uport, UART_TX_REG, xmit->buf[xmit->tail]);
        xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
        port->icount.tx++;
    }
    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);
}

static void gh1406_uart_stop_rx(struct uart_port *port) {}
static void gh1406_uart_break_ctl(struct uart_port *port, int break_state) {}

static int gh1406_uart_startup(struct uart_port *port)
{
    struct gh1406_uart_port *uport = container_of(port, struct gh1406_uart_port, port);
    timer_setup(&uport->poll_timer, gh1406_uart_poll_rx, 0);
    mod_timer(&uport->poll_timer, jiffies + msecs_to_jiffies(UART_POLL_INTERVAL));
    return 0;
}

static void gh1406_uart_shutdown(struct uart_port *port)
{
    struct gh1406_uart_port *uport = container_of(port, struct gh1406_uart_port, port);
    del_timer_sync(&uport->poll_timer);
}

static void gh1406_uart_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
    struct gh1406_uart_port *uport = container_of(port, struct gh1406_uart_port, port);
    unsigned int baud;
    u16 baud_div;
    u8 ctrl2;
    unsigned long flags;

    // 获取波特率
    baud = uart_get_baud_rate(port, termios, old, 9600, port->uartclk / 16);
    pr_info("gh1406-uart: port %d, set_termios, baud %u\n", uport->port_id, baud);

    spin_lock_irqsave(&port->lock, flags);
    uart_update_timeout(port, termios->c_cflag, baud);
    // pr_info("[%s : %d] Enter here baud=%d\n",__func__,__LINE__,baud);
    // 计算分频值
    baud_div = (port->uartclk + (baud * 8)) / (baud * 16) - 1;

    // 设置波特率
    gh1406_uart_writeb(uport, UART_CTRL1_REG, baud_div & 0xFF);
    ctrl2 = gh1406_uart_readb(uport, UART_CTRL2_REG) & 0x07; // 保留低3位
    ctrl2 |= ((baud_div >> 8) & 0x1F) << 3;

    // 设置数据位
    // termios->c_cflag &= ~CSIZE;
    // termios->c_cflag |= CS8;
    ctrl2 |= 0x01; // 8位

    // 设置奇偶校验
    ctrl2 &= ~(UART_CTRL_PARITY_EN | UART_CTRL_ODD_PARITY); // 清除控制寄存器的奇偶校验位
    if (termios->c_cflag & PARENB)
    {
        ctrl2 |= UART_CTRL_PARITY_EN;
        if (termios->c_cflag & PARODD)
        {
            ctrl2 |= UART_CTRL_ODD_PARITY;
            pr_info("gh1406_uart: using odd parity\n");
        }
        else
        {
            pr_info("gh1406_uart: using even parity\n");
        }
    }

    // 设置停止位
    termios->c_cflag &= ~CSTOPB;
    gh1406_uart_writeb(uport, UART_CTRL2_REG, ctrl2);
    uport->baud_configured = true;
    // pr_info("gh1406-uart: port %d, set_termios, baud %u finished\n", uport->port_id, baud);
    spin_unlock_irqrestore(&port->lock, flags);
}

static const char *gh1406_uart_type(struct uart_port *port) { return "GH1406"; }
static void gh1406_uart_config_port(struct uart_port *port, int flags)
{
    if (flags & UART_CONFIG_TYPE)
        port->type = PORT_UNKNOWN;
}

static struct uart_ops gh1406_uart_ops = {
    .tx_empty = gh1406_uart_tx_empty,
    .set_mctrl = gh1406_uart_set_mctrl,
    .get_mctrl = gh1406_uart_get_mctrl,
    .stop_tx = gh1406_uart_stop_tx,
    .start_tx = gh1406_uart_start_tx,
    .stop_rx = gh1406_uart_stop_rx,
    .break_ctl = gh1406_uart_break_ctl,
    .startup = gh1406_uart_startup,
    .shutdown = gh1406_uart_shutdown,
    .set_termios = gh1406_uart_set_termios,
    .type = gh1406_uart_type,
    .config_port = gh1406_uart_config_port,
};

static struct uart_driver gh1406_uart_driver = {
    .owner = THIS_MODULE,
    .driver_name = "gh1406-uart",
    .dev_name = "ttyGH",
    .major = 0,
    .minor = 0,
    .nr = GH1406_UART_PORTS,
};

#define PCI_VENDOR_ID_ACTEL 0x11aa
#define PCI_DEVICE_ID_GH1406 0x11aa

static const struct pci_device_id gh1406_pci_ids[] = {
    {PCI_DEVICE(PCI_VENDOR_ID_ACTEL, PCI_DEVICE_ID_GH1406)},
    {0}};
MODULE_DEVICE_TABLE(pci, gh1406_pci_ids);

static int gh1406_uart_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    int ret, i;
    resource_size_t bar_start = pci_resource_start(pdev, GH1406_UART_BAR_IDX);
    resource_size_t bar_len = pci_resource_len(pdev, GH1406_UART_BAR_IDX);
    void __iomem *bar0_virt;

    ret = pci_enable_device(pdev);
    if (ret)
        return ret;

    if (!request_mem_region(bar_start, bar_len, "gh1406-uart"))
        return -EBUSY;

    bar0_virt = ioremap(bar_start, bar_len);
    if (!bar0_virt)
        return -ENOMEM;

    ret = uart_register_driver(&gh1406_uart_driver);
    if (ret)
        goto err_unmap;

    gh1406_uart_dev = kzalloc(sizeof(*gh1406_uart_dev), GFP_KERNEL);
    if (!gh1406_uart_dev)
    {
        ret = -ENOMEM;
        goto err_unreg_uart;
    }

    gh1406_uart_dev->base = bar0_virt + GH1406_UART_BAR_OFFSET;

    for (i = 0; i < GH1406_UART_PORTS; i++)
    {
        struct gh1406_uart_port *uport = &gh1406_uart_dev->ports[i];
        struct uart_port *port = &uport->port;

        uport->base = gh1406_uart_dev->base + i * UART_PORT_OFFSET;
        uport->port_id = i;
        uport->baud_configured = false;
        port->membase = uport->base;
        port->mapbase = bar_start + GH1406_UART_BAR_OFFSET + i * UART_PORT_OFFSET;
        port->iotype = UPIO_MEM;
        port->irq = 0;
        port->uartclk = SYS_CLK;
        port->fifosize = 128;
        port->ops = &gh1406_uart_ops;
        port->flags = UPF_SKIP_TEST | UPF_FIXED_TYPE;
        port->line = i;
        port->type = PORT_PIC32;
        port->dev = &pdev->dev;

        ret = uart_add_one_port(&gh1406_uart_driver, port);
        if (ret)
        {
            pr_err("gh1406-uart: failed to add port %d, error %d\n", i, ret);
            goto err_rem_ports;
        }
    }

    pci_set_drvdata(pdev, gh1406_uart_dev);
    return 0;

err_rem_ports:
    while (--i >= 0)
        uart_remove_one_port(&gh1406_uart_driver, &gh1406_uart_dev->ports[i].port);
    kfree(gh1406_uart_dev);
err_unreg_uart:
    uart_unregister_driver(&gh1406_uart_driver);
err_unmap:
    iounmap(bar0_virt);
    release_mem_region(bar_start, bar_len);
    return ret;
}

static void gh1406_uart_remove(struct pci_dev *pdev)
{
    int i;
    struct gh1406_uart_dev *dev = pci_get_drvdata(pdev);
    resource_size_t bar_start = pci_resource_start(pdev, GH1406_UART_BAR_IDX);
    resource_size_t bar_len = pci_resource_len(pdev, GH1406_UART_BAR_IDX);

    for (i = 0; i < GH1406_UART_PORTS; i++)
        uart_remove_one_port(&gh1406_uart_driver, &dev->ports[i].port);

    iounmap(dev->base - GH1406_UART_BAR_OFFSET);
    release_mem_region(bar_start, bar_len);
    kfree(dev);
    uart_unregister_driver(&gh1406_uart_driver);
}

static struct pci_driver gh1406_pci_driver = {
    .name = "gh1406-uart",
    .id_table = gh1406_pci_ids,
    .probe = gh1406_uart_probe,
    .remove = gh1406_uart_remove,
};

module_pci_driver(gh1406_pci_driver);

MODULE_AUTHOR("xuguokai@ucas.com.cn");
MODULE_DESCRIPTION("GH1406 UART PCIe Driver (Polling)");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
