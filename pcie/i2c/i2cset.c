#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>

#define BASE_ADDR_I2C(n) (0x72100000 + ((n - 1) * 0x10000)) // I2C1~I2C5
// #define PCIE_I2C2_BASE 0x72110000       //I2C1=0x72100000 I2C2=0x72110000 I2C3=0x72120000 I2C4=0x72130000 I2C5=0x72140000
#define MAP_SIZE 0x4000
#define MAP_MASK (MAP_SIZE - 1)

#define SLAVE_ADDR 0x50     // Example slave address
#define SLAVE_REG_ADDR 0x00 // Example register address

// I2C Registers
#define I2C_CTRL_REG 0x00
#define I2C_STAT_REG 0x04
#define I2C_DATA_REG 0x08

// Control bits
#define I2C_CTRL_CR2 (1 << 7)
#define I2C_CTRL_ENS1 (1 << 6)
#define I2C_CTRL_STA (1 << 5)
#define I2C_CTRL_STO (1 << 4)
#define I2C_CTRL_SI (1 << 3)
#define I2C_CTRL_AA (1 << 2)
#define I2C_CTRL_CR1 (1 << 1)
#define I2C_CTRL_CR0 (1 << 0)

// Status codes
#define I2C_STATUS_START_TRANSMITTED 0x08
#define I2C_STATUS_REPEATED_START_TRANSMITTED 0x10
#define I2C_STATUS_SLA_W_ACK 0x18
#define I2C_STATUS_DATA_TRANSMITTED_ACK 0x28
#define I2C_STATUS_SLA_R_ACK 0x40
#define I2C_STATUS_DATA_RECEIVED_ACK 0x50
#define I2C_STATUS_DATA_RECEIVED_NACK 0x58
#define I2C_STATUS_STOP_TRANSMITTED 0xE0

static uint8_t *i2c_base = NULL;

void *devmemInit(uint64_t regBase);
void i2c_init(void);
int i2c_master_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

int main(int argc, char *argv[])
{
    if (argc != 5)
    {
        printf("用法: %s <I2C通道编号: 1~5>  <i2c从地址> <i2c寄存器地址> <i2c写入的值>\n", argv[0]);
        printf("示例: %s 1 0x48 0x00 0xAD\n", argv[0]);
        return -1;
    }

    int i2c_id = atoi(argv[1]);
    if (i2c_id < 1 || i2c_id > 5)
    {
        fprintf(stderr, "错误: I2C通道编号必须在1到5之间\n");
        return -1;
    }

    // 解析I2C从地址（十六进制，支持0x前缀或纯数字）
    char *endptr;
    uint64_t dev_addr_val = strtoul(argv[2], &endptr, 16);
    if (*endptr != '\0' || dev_addr_val > 0xFF) // I2C从地址通常为8位（0~255）
    {
        fprintf(stderr, "错误: 无效的I2C从地址（必须是0~0xFF的十六进制值）\n");
        return -1;
    }
    uint8_t dev_addr = (uint8_t)dev_addr_val;
    printf("slaver_addr=0x%x\n", dev_addr);

    // 解析寄存器地址（十六进制）
    uint64_t reg_addr_val = strtoul(argv[3], &endptr, 16);
    if (*endptr != '\0' || reg_addr_val > 0xFF) // 寄存器地址通常为8位
    {
        fprintf(stderr, "错误: 无效的寄存器地址（必须是0~0xFF的十六进制值）\n");
        return -1;
    }
    uint8_t reg_addr = (uint8_t)reg_addr_val;
    printf("reg_addr=0x%x\n", reg_addr);

    // 解析写入值（十六进制）
    uint64_t tmpwrite_val = strtoul(argv[4], &endptr, 16);
    if (*endptr != '\0' || tmpwrite_val > 0xFF) // 写入值通常为8位
    {
        fprintf(stderr, "错误: 无效的写入值（必须是0~0xFF的十六进制值）\n");
        return -1;
    }


    uint64_t base_addr_val = BASE_ADDR_I2C(i2c_id);
    // uint8_t dev_addr = SLAVE_ADDR;
    // uint8_t reg_addr = SLAVE_REG_ADDR;
    uint8_t write_val = (uint8_t)tmpwrite_val;
    printf("write_val=0x%x\n", write_val);

    i2c_base = (uint8_t *)devmemInit(base_addr_val);
    if (i2c_base == NULL)
    {
        return EXIT_FAILURE;
    }

    i2c_init();
    if (i2c_master_write_byte(dev_addr, reg_addr, write_val) != 0)
    {
        fprintf(stderr, "[ERROR] I2C write failed\n");
        goto cleanup;
    }

cleanup:
    munmap((void *)i2c_base, MAP_SIZE);
    return EXIT_SUCCESS;
}

void *devmemInit(uint64_t regBase)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1)
    {
        perror("无法打开 /dev/mem");
        return NULL;
    }

    void *mapBase = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, regBase & ~MAP_MASK);
    if (mapBase == MAP_FAILED)
    {
        perror("mmap 失败");
        close(fd);
        return NULL;
    }

    close(fd);
    return mapBase;
}

static uint8_t i2c_get_reg(uint8_t offset)
{
    return *(uint8_t *)(i2c_base + offset);
}

static void i2c_set_reg(uint8_t offset, uint8_t val)
{
    *(i2c_base + offset) = val;
}

static uint8_t i2c_get_ctrl(void)
{
    return i2c_get_reg(I2C_CTRL_REG);
}

static void i2c_set_ctrl(uint8_t val)
{
    i2c_set_reg(I2C_CTRL_REG, val);
}

static uint8_t i2c_get_data(void)
{
    return i2c_get_reg(I2C_DATA_REG);
}

static void i2c_set_data(uint8_t val)
{
    i2c_set_reg(I2C_DATA_REG, val);
}

static uint8_t i2c_get_status(void)
{
    return i2c_get_reg(I2C_STAT_REG);
}

static void i2c_set_bits_debug(uint8_t bits, const char *info)
{
    uint8_t old = i2c_get_ctrl();
    uint8_t new_val = old | bits;
    i2c_set_ctrl(new_val);
    printf("[CTRL] %-24s: 0x%02X | 0x%02X -> 0x%02X\n", info, old, bits, new_val);
}
static void i2c_clear_bits_debug(uint8_t bits, const char *info)
{
    uint8_t old = i2c_get_ctrl();
    uint8_t new_val = old & ~bits;
    i2c_set_ctrl(new_val);
    printf("[CTRL] %-24s: 0x%02X & ~0x%02X -> 0x%02X\n", info, old, bits, new_val);
}

static int i2c_wait_si_debug(const char *step)
{
    int timeout = 1000;
    while (!(i2c_get_ctrl() & I2C_CTRL_SI))
    {
        if (--timeout <= 0)
        {
            fprintf(stderr, "[TIMEOUT] %s: waiting for SI\n", step);
            return -1;
        }
        usleep(10);
    }
    // uint8_t status = i2c_get_status();
    // printf("[STATUS] %-24s: 0x%02X\n", step, status);
    return 0;
}

void i2c_init(void)
{
    i2c_set_ctrl(0x00); // Reset control register
    i2c_set_ctrl(I2C_CTRL_ENS1 | I2C_CTRL_CR2 | I2C_CTRL_CR1 | I2C_CTRL_CR0);
    // printf("[INIT] CTRL=0x%02X\n", i2c_get_ctrl());
}

int i2c_master_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    // START & send ADDR
    printf("[WRITE] Start I2C write\n");
    i2c_set_ctrl(i2c_get_ctrl() | I2C_CTRL_STA); // Set START condition
    i2c_wait_si_debug("Write start condition");
    printf("[START STATUS] STATUS=0x%02X\n", i2c_get_status());
    i2c_set_data(dev_addr << 1); // Set device address (write mode)
    i2c_wait_si_debug("Write device address");
    i2c_clear_bits_debug(I2C_CTRL_STA | I2C_CTRL_SI, "Clear START & SI");
    printf("CTRL after reg addr: 0x%02X\n", i2c_get_ctrl());
    i2c_wait_si_debug(" ");
    if (i2c_get_status() != I2C_STATUS_SLA_W_ACK)
    {
        printf("[ERROR] [SLA+W STATUS] STATUS=0x%02X\n", i2c_get_status());
        return -1;
    }
    printf("[SLA+W STATUS] STATUS=0x%02X\n", i2c_get_status());

    // Register address
    i2c_set_data(reg_addr);
    i2c_wait_si_debug("Write register address");
    // i2c_clear_bits_debug(I2C_CTRL_SI, "Clear SI");
    i2c_base[I2C_CTRL_REG] = 0xC3;
    i2c_wait_si_debug(" ");
    if (i2c_get_status() != I2C_STATUS_DATA_TRANSMITTED_ACK)
    {
        printf("[ERROR] [REG_ADDR STATUS] STATUS=0x%02X\n", i2c_get_status());
        return -1;
    }
    printf("[REG_ADDR STATUS] STATUS=0x%02X\n", i2c_get_status());

    // Data
    printf("[DATA] Write data: 0x%02X\n", data);
    i2c_set_data(data);
    i2c_wait_si_debug("Write data");
    i2c_clear_bits_debug(I2C_CTRL_SI, "Clear SI");
    i2c_wait_si_debug(" ");
    if (i2c_get_status() != I2C_STATUS_DATA_TRANSMITTED_ACK)
    {
        printf("[ERROR] [DATA STATUS] STATUS=0x%02X\n", i2c_get_status());
        return -1;
    }
    printf("[!!DATA STATUS] STATUS=0x%02X\n", i2c_get_status());

    // STOP condition
    i2c_set_ctrl(i2c_get_ctrl() | I2C_CTRL_STO);
    i2c_clear_bits_debug(I2C_CTRL_SI, "Clear SI");
    i2c_wait_si_debug("wait STOP");
    if (i2c_get_status() == I2C_STATUS_STOP_TRANSMITTED)
    {
        printf("[STOP STATUS] STATUS=0x%02X\n", i2c_get_status());
    }
    printf("[WRITE] Done\n");
    // i2c_clear_bits_debug(I2C_CTRL_SI, "Clear SI after STOP");
    return 0;
}
