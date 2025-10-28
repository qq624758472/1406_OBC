// pcmspi.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <string.h>
#include <linux/spi/spidev.h>
#include "pcmspi.h"

// Device & Memory
#define DEVNAME "/dev/spidev0.0"
#define GPIO_BASE 0x72200000
#define GPIO_OFFSET 0xa0
#define MAP_SIZE 0x4000
#define MAP_MASK (MAP_SIZE - 1)

// PCM Operation Cod5
#define PCMSPI_OP_WREN  0x06
#define PCMSPI_OP_WRDI  0x04
#define PCMSPI_OP_RDSR  0x05
#define PCMSPI_OP_WRSR  0x01
#define PCMSPI_OP_READ  0x03
#define PCMSPI_OP_WRITE 0x02

// Buffer & Address
#define PCMSPI_BUF_MAX     48
#define PCMSPI_ADDR_LEN    3  // 24-bit address
#define PCMSPI_OPCODE_LEN  1
#define PCMSPI_SPEED       4000000  // 4 MHz
#define PCMSPI_BITS        8
#define PCMSPI_CFGWORD     0xe9d7ffa9


int pcmspi_write_enable(void);
// int pcmspi_wait_ready(void);  // 新增：等待写完成

int pcmspi_cfg(uint32_t cfgword);
void *devmemInit(uint64_t regBase);

// Bit-banging control (for cfg)
void SETCFGD(void *addr, int d);
void SETCFGC(void *addr, int d);

int g_fd = -1;
uint8_t tx_buf[PCMSPI_BUF_MAX];
uint8_t rx_buf[PCMSPI_BUF_MAX];

void *devmemInit(uint64_t regBase) {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1) {
        perror("无法打开 /dev/mem，请使用 sudo");
        return NULL;
    }
    void *mapBase = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, regBase & ~MAP_MASK);
    if (mapBase == MAP_FAILED) {
        perror("mmap 失败");
        close(fd);
        return NULL;
    }
    close(fd);
    return mapBase;
}

int pcmspi_init(void)
{
    uint8_t mode = SPI_MODE_0;          // CPOL=0, CPHA=0
    uint8_t bits = PCMSPI_BITS;
    uint32_t speed = PCMSPI_SPEED;
    int rv = PCMSPI_EC_OK;

    g_fd = open(DEVNAME, O_RDWR);
    // printf("打开 SPI 设备 [%s]\n", DEVNAME);
    if (g_fd < 0) {
        perror("open");
        rv = PCMSPI_EC_DEVOPEN;
        goto out;
    }
    // printf("SPI 设备打开成功 (fd=%d)\n", g_fd);

    // 设置 SPI 模式
    if (ioctl(g_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("无法设置 SPI 模式");
        rv = PCMSPI_EC_IOCTL;
        goto out;
    }

    // 设置位宽
    if (ioctl(g_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        perror("无法设置 SPI 位宽");
        rv = PCMSPI_EC_IOCTL;
        goto out;
    }

    // 设置速度
    if (ioctl(g_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("无法设置 SPI 速度");
        rv = PCMSPI_EC_IOCTL;
        goto out;
    }

    // 配置 GPIO（专用接口）
  //  if (pcmspi_cfg(PCMSPI_CFGWORD) != 0) {
  //      printf("pcmspi_cfg 配置失败！请确认 /dev/mem 权限和地址\n");
   //     rv = PCMSPI_EC_CFG;
  //      goto out;
   // }
   //  printf("GPIO 配置成功\n");

out:
    if (rv != PCMSPI_EC_OK && g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
    return rv;
}



int pcmspi_read(uint8_t *buf, uint32_t uiStartAddr, int len)
{
    int rv = PCMSPI_EC_OK;
    struct spi_ioc_transfer tr;
    int bytesleft = len, translen, transoff;
    uint32_t transaddr;
    memset(&tr, 0, sizeof(tr));
    int i;

    // printf("开始读取: 地址=%#x, 长度=%d\n", uiStartAddr, len);
    if (g_fd < 0) { rv = PCMSPI_EC_DEVOPEN; goto out; }
    if (buf == NULL) { rv = PCMSPI_EC_INVALID_PARAMS; goto out; }

    while(bytesleft>0)
    {
        translen = bytesleft;
        transaddr = uiStartAddr + len - bytesleft;
        transoff = len-bytesleft;
    if (translen + PCMSPI_OPCODE_LEN + PCMSPI_ADDR_LEN > PCMSPI_BUF_MAX) {
            translen = PCMSPI_BUF_MAX - PCMSPI_OPCODE_LEN - PCMSPI_ADDR_LEN;
        }

        // printf("Read seg: addr(%#x), len(%d)\n", transaddr, translen);

        memset(tx_buf, 0, sizeof(tx_buf));
        memset(rx_buf, 0, sizeof(rx_buf));

        tx_buf[0] = PCMSPI_OP_READ;
        tx_buf[1] = (transaddr >> 16) & 0xFF;
        tx_buf[2] = (transaddr >> 8)  & 0xFF;
        tx_buf[3] = transaddr         & 0xFF;

        tr.tx_buf = (uintptr_t)tx_buf;
        tr.rx_buf = (uintptr_t)rx_buf;
        tr.len = translen + PCMSPI_OPCODE_LEN + PCMSPI_ADDR_LEN;
        tr.speed_hz = PCMSPI_SPEED;
        tr.bits_per_word = PCMSPI_BITS;
        tr.cs_change = 0;
        // printf("trbuf: ");
        // for(i=0; i<tr.len; i++)
        // {
        //     if((i%16)==0) printf("\n\t");
        //     printf("%02x ", tx_buf[i]);
        // } 
        // printf("\n");
        // printf("tr.len[%d],hz[%d],bpw[%d]\n", tr.len,tr.speed_hz,tr.bits_per_word);

        rv = ioctl(g_fd, SPI_IOC_MESSAGE(1), &tr);
        if (rv < 0) {
            perror("读取失败 (ioctl)");
            rv = transoff;
            goto out;
        }

        memcpy(buf+transoff, rx_buf + PCMSPI_OPCODE_LEN + PCMSPI_ADDR_LEN, translen);
        // printf("read out: ");
        // for(i=transoff; i<transoff+translen; i++)
        // {
        //     if((i%16)==0) printf("\n\t");
        //     printf("%02x ", buf[i]);
        // } 
        // printf("\n");
        bytesleft -= translen;
    }
    rv = len;
out:
    return rv;
}

uint8_t pcmspi_readb(uint32_t uiAddr)
{
    uint8_t data;
    if (pcmspi_read(&data, uiAddr, 1) == 1)
        return data;
    return 0xFF;
}

uint16_t pcmspi_readw(uint32_t uiAddr)
{
    uint8_t buf[2];
    if (pcmspi_read(buf, uiAddr, 2) == 2)
        return (buf[0] << 8) | buf[1];  // Big-Endian
    return 0xFFFF;
}

uint32_t pcmspi_readd(uint32_t uiAddr)
{
    uint8_t buf[4];
    if (pcmspi_read(buf, uiAddr, 4) == 4)
        return (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    return 0xFFFFFFFF;
}

int pcmspi_write_enable(void)
{
    struct spi_ioc_transfer tr;
    int i;
    memset(&tr, 0, sizeof(tr));

    // printf("发送写使能 (WREN)\n");
    if (g_fd < 0) return PCMSPI_EC_DEVOPEN;

    memset(tx_buf, 0, sizeof(tx_buf));
    tx_buf[0] = PCMSPI_OP_WREN;

    tr.tx_buf = (uintptr_t)tx_buf;
    tr.rx_buf = (uintptr_t)rx_buf;
    tr.len = 1;
    tr.speed_hz = PCMSPI_SPEED;
    tr.bits_per_word = PCMSPI_BITS;
    // printf("trbuf: ");
    // for(i=0; i<tr.len; i++) printf("[%02x]", tx_buf[i]);
    // printf("\n");
    // printf("tr.len[%d],hz[%d],bpw[%d]\n", tr.len,tr.speed_hz,tr.bits_per_word);

    int rv = ioctl(g_fd, SPI_IOC_MESSAGE(1), &tr);
    return rv < 0 ? PCMSPI_EC_IOCTL : PCMSPI_EC_OK;
}

int pcmspi_write(uint8_t *buf, uint32_t uiStartAddr, int len)
{
    int rv = PCMSPI_EC_OK;
    int bytesleft = len, translen, transoff;
    uint32_t transaddr;
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    int i;

    // printf("开始写入: 地址=%#x, 长度=%d\n", uiStartAddr, len);
    if (g_fd < 0) { rv = PCMSPI_EC_DEVOPEN; goto out; }
    if (buf == NULL) { rv = PCMSPI_EC_INVALID_PARAMS; goto out; }

    while(bytesleft>0)
    {
        translen = bytesleft;
        transaddr = uiStartAddr + len - bytesleft;
        transoff = len-bytesleft;
        if (translen + PCMSPI_OPCODE_LEN + PCMSPI_ADDR_LEN > PCMSPI_BUF_MAX) {
            translen = PCMSPI_BUF_MAX - PCMSPI_OPCODE_LEN - PCMSPI_ADDR_LEN;
        }

        // printf("Write seg: addr(%#x), len(%d)\n", transaddr, translen);
            // 发送 WREN
        rv = pcmspi_write_enable();
        if (rv < 0) {
            printf("写使能失败\n");
            goto out;
        }
        //usleep(5);
        memset(tx_buf, 0, sizeof(tx_buf));
        tx_buf[0] = PCMSPI_OP_WRITE;
        tx_buf[1] = (transaddr >> 16) & 0xFF;
        tx_buf[2] = (transaddr >> 8)  & 0xFF;
        tx_buf[3] = transaddr         & 0xFF;
        memcpy(tx_buf + PCMSPI_OPCODE_LEN + PCMSPI_ADDR_LEN, buf+transoff, translen);

        tr.tx_buf = (uintptr_t)tx_buf;
        tr.rx_buf = (uintptr_t)rx_buf;
        tr.len = translen + PCMSPI_OPCODE_LEN + PCMSPI_ADDR_LEN;
        tr.speed_hz = PCMSPI_SPEED;
        tr.bits_per_word = PCMSPI_BITS;

        // printf("trbuf: ");
        // for(i=0; i<tr.len; i++) 
        // {
        //     if(i%16) printf("\n\t");
        //     printf("[%02x]", tx_buf[i]);
        // }
        // printf("\n");
        // printf("tr.len[%d],hz[%d],bpw[%d]\n", tr.len,tr.speed_hz,tr.bits_per_word);

        rv = ioctl(g_fd, SPI_IOC_MESSAGE(1), &tr);
        if (rv < 0) {
            perror("写入失败");
            rv = transoff;
            goto out;
        }

        bytesleft -= translen;
    }
    rv = len;
out:
    return rv;
}

int pcmspi_writeb(uint32_t uiAddr, uint8_t data)
{
    return pcmspi_write(&data, uiAddr, 1);
}

int pcmspi_writew(uint32_t uiAddr, uint16_t data)
{
    uint8_t buf[2] = { (data >> 8) & 0xFF, data & 0xFF };
    return pcmspi_write(buf, uiAddr, 2);
}

int pcmspi_writed(uint32_t uiAddr, uint32_t data)
{
    uint8_t buf[4];
    buf[0] = (data >> 24) & 0xFF;
    buf[1] = (data >> 16) & 0xFF;
    buf[2] = (data >> 8)  & 0xFF;
    buf[3] = data         & 0xFF;
    return pcmspi_write(buf, uiAddr, 4);
}

int pcmspi_cfg(uint32_t cfgword)
{
    void *mapBase = devmemInit(GPIO_BASE);
    if (!mapBase) {
        printf("GPIO 内存映射失败\n");
        return -1;
    }

    void *virt_addr = (uint8_t *)mapBase + (GPIO_OFFSET & MAP_MASK);
    // printf("配置 PCM CFG: %#x\n", cfgword);

    SETCFGD(virt_addr, 0);
    SETCFGC(virt_addr, 0);

    for (int i = 31; i >= 0; i--) {
        if ((cfgword >> i) & 0x01) {
            SETCFGD(virt_addr, 1);
            // printf("1");
        } else {
            SETCFGD(virt_addr, 0);
            // printf("0");
        }
        SETCFGC(virt_addr, 1);
        SETCFGC(virt_addr, 0);
    }
    // printf("\n");

    SETCFGD(virt_addr, 0);
    SETCFGC(virt_addr, 0);

    munmap(mapBase, MAP_SIZE);
    return 0;
}

void SETCFGD(void *addr, int d)
{
    volatile uint32_t *reg = (volatile uint32_t *)addr;
    *reg = (*reg & ~(1U << 15)) | ((d & 1) << 15);
}

void SETCFGC(void *addr, int d)
{
    volatile uint32_t *reg = (volatile uint32_t *)addr;
    *reg = (*reg & ~(1U << 16)) | ((d & 1) << 16);
}

int pcmspi_exit(void)
{
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
    return PCMSPI_EC_OK;
}
