#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdint.h>

#define BASE_ADDR_UART(n) (0x72000000 + ((n - 1) * 0x10000)) // UART1~UART6
// #define PCIE_UART1_BASE  0x72000000   // UART1=72000000  UART2=72010000 UART3=0x72020000 UART4=0x72030000 UART5=0x72040000 UART6=0x72050000
#define MAP_SIZE 0x4000
#define BAUD 2400
#define MAP_MASK (MAP_SIZE - 1)
#define SYS_CLK 20000000UL // 100MHz 系统时钟

volatile uint8_t *devmemInit(uint64_t regBase);
void uart_init(volatile uint8_t *base_addr, int baud_rate);
int uart_read(volatile uint8_t *status_addr, volatile uint8_t *read_addr, uint8_t *data);
int uart_write(volatile uint8_t *status_addr, volatile uint8_t *write_addr, uint8_t data);

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("用法: %s <UART通道编号: 1~6>\n", argv[0]);
        return -1;
    }

    int uart_id = atoi(argv[1]);
    if (uart_id < 1 || uart_id > 6)
    {
        fprintf(stderr, "错误: UART通道编号必须在1到6之间\n");
        return -1;
    }

    uint64_t base_addr_val = BASE_ADDR_UART(uart_id);
    volatile uint8_t *base = devmemInit(base_addr_val);
    if (base == NULL)
    {
        fprintf(stderr, "Failed to initialize devmem\n");
        return EXIT_FAILURE;
    }

    volatile uint8_t *READ_ADDR = base + 0x04;
    volatile uint8_t *WRITE_ADDR = base + 0x00;
    volatile uint8_t *STATUS_ADDR = base + 0x10;

    uint8_t read_byte;

    uart_init(base, BAUD);

#if 0
    // 写一个字节
    if (uart_write(STATUS_ADDR, WRITE_ADDR, 0x55) != 0)
    {
        fprintf(stderr, "UART write failed\n");
    }
    else
    {
        printf("UART wrote byte: 0x55\n");
    }
#endif

#if 1
    // 读一个字节，循环直到收到0停止
    uint16_t recv = 1;
    while (recv)
    {
        if (uart_read(STATUS_ADDR, READ_ADDR, &read_byte) == 0)
        {
            printf("UART read byte: 0x%02X\n", read_byte);
            recv = read_byte;
        }
        else
        {
            usleep(1000); // 等待 1ms 避免死循环空转
        }
    }
#endif

    munmap((void *)((uintptr_t)base & ~MAP_MASK), MAP_SIZE);
    return EXIT_SUCCESS;
}

volatile uint8_t *devmemInit(uint64_t regBase)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1)
    {
        perror("Failed to open /dev/mem");
        return NULL;
    }

    void *mapBase = mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, regBase & ~MAP_MASK);
    close(fd);

    if (mapBase == MAP_FAILED)
    {
        perror("Failed to map memory");
        return NULL;
    }

    return (volatile uint8_t *)((uintptr_t)mapBase + (regBase & MAP_MASK));
}

void uart_init(volatile uint8_t *base_addr, int baud_rate)
{
    volatile uint8_t *CTRL1_ADDR = base_addr + 0x08;
    volatile uint8_t *CTRL2_ADDR = base_addr + 0x0C;

    uint16_t baud_value = (uint16_t)((SYS_CLK + (baud_rate * 8)) / (16 * baud_rate) - 1);

    // 写入低8位
    *CTRL1_ADDR = (uint8_t)(baud_value & 0xFF);

    // 写入高5位，保留低3位，且设置bit0=1，bit1=0（width=8, odd=N）
    uint8_t ctrl2_val = *CTRL2_ADDR & 0x07;       // 保留 bit[2:0]
    ctrl2_val |= ((baud_value >> 8) & 0x1F) << 3; // 高5位写 bit[7:3]
    ctrl2_val |= (1 << 0);                        // bit0 = 1
    ctrl2_val &= ~(1 << 1);                       // bit1 = 0
    *CTRL2_ADDR = ctrl2_val;
}

// 返回0表示成功读到数据，-1表示无数据可读
int uart_read(volatile uint8_t *status_addr, volatile uint8_t *read_addr, uint8_t *data)
{
    // 状态寄存器bit1为RX_READY标志位
    if ((*status_addr & 0x02) != 0)
    {
        *data = *read_addr;
        return 0;
    }
    return -1;
}

// 返回0表示成功写入，-1表示写缓冲满或不可写
int uart_write(volatile uint8_t *status_addr, volatile uint8_t *write_addr, uint8_t data)
{
    // 状态寄存器bit0为TX_READY标志位
    int timeout = 100000;
    while ((*status_addr & 0x01) == 0)
    {
        if (--timeout == 0)
        {
            return -1;
        }
        usleep(1);
    }
    *write_addr = data;
    return 0;
}
