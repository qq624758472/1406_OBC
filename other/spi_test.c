#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define DEV_PATH "/dev/spidev0.0"
#define SPI_SPEED 10000000

#define CMD_WREN   0x06
#define CMD_WRITE  0x02
#define CMD_READ   0x03

int spi_fd = -1;

int spi_init(const char *device) {
    spi_fd = open(device, O_RDWR);
    if (spi_fd < 0) {
        perror("open");
        return -1;
    }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = SPI_SPEED;

    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    return 0;
}

void spi_close() {
    if (spi_fd >= 0)
        close(spi_fd);
}

// 发送 WREN
int spi_wren() {
    uint8_t cmd = CMD_WREN;
    return write(spi_fd, &cmd, 1);
}

// 写一个字节
int spi_write_byte(uint32_t addr, uint8_t data) {
    uint8_t tx[5] = {
        CMD_WRITE,
        (addr >> 16) & 0x7F,  // 确保23位地址，最高位为0
        (addr >> 8) & 0xFF,
        addr & 0xFF,
        data
    };

    if (spi_wren() < 0) {
        perror("WREN failed");
        return -1;
    }

    usleep(5);  // 短暂延时

    if (write(spi_fd, tx, sizeof(tx)) != sizeof(tx)) {
        perror("WRITE failed");
        return -1;
    }

    usleep(100);  // 写入后延时

    return 0;
}

// 读一个字节
int spi_read_byte(uint32_t addr, uint8_t *data) {
    uint8_t tx[4] = {
        CMD_READ,
        (addr >> 16) & 0x7F,
        (addr >> 8) & 0xFF,
        addr & 0xFF
    };
    uint8_t rx = 0;

    struct spi_ioc_transfer tr[2] = {
        {
            .tx_buf = (unsigned long)tx,
            .rx_buf = 0,
            .len = sizeof(tx),
            .speed_hz = SPI_SPEED,
        },
        {
            .tx_buf = 0,
            .rx_buf = (unsigned long)&rx,
            .len = 1,
            .speed_hz = SPI_SPEED,
        }
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(2), tr) < 0) {
        perror("READ failed");
        return -1;
    }

    *data = rx;
    return 0;
}

int main() {
    uint32_t addr = 0x001234;  // 测试地址
    uint8_t val = 0xA5;
    uint8_t read_val = 0;

    if (spi_init(DEV_PATH) != 0)
        return 1;

    printf("Write 0x%02X to address 0x%06X\n", val, addr);
    if (spi_write_byte(addr, val) != 0) {
        spi_close();
        return 1;
    }

    if (spi_read_byte(addr, &read_val) != 0) {
        spi_close();
        return 1;
    }

    printf("Read back 0x%02X from address 0x%06X\n", read_val, addr);

    if (read_val == val) {
        printf("SUCCESS: Value matches!\n");
    } else {
        printf("FAIL: Expected 0x%02X, got 0x%02X\n", val, read_val);
    }

    spi_close();
    return 0;
}
