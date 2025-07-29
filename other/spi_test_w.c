// spi_write_byte.c

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define DEV_PATH "/dev/spidev0.0"
#define SPI_SPEED 10000000

#define CMD_WREN  0x06
#define CMD_WRITE 0x02

int spi_fd = -1;

int spi_init() {
    spi_fd = open(DEV_PATH, O_RDWR);
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
    if (spi_fd >= 0) close(spi_fd);
}

int spi_wren() {
    uint8_t cmd = CMD_WREN;
    if (write(spi_fd, &cmd, 1) != 1) {
        perror("WREN failed");
        return -1;
    }
    usleep(1000);
    return 0;
}

int spi_write_byte(uint32_t addr, uint8_t data) {
    if (spi_wren() < 0)
        return -1;

    uint8_t tx[5] = {
        CMD_WRITE,
        (addr >> 16) & 0x7F,  // 23位地址最高位清0
        (addr >> 8) & 0xFF,
        addr & 0xFF,
        data
    };

    if (write(spi_fd, tx, sizeof(tx)) != sizeof(tx)) {
        perror("WRITE failed");
        return -1;
    }

    usleep(100);  // 等待写完成

    return 0;
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("Usage: %s <address_hex> <data_hex>\n", argv[0]);
        return 1;
    }

    uint32_t addr = (uint32_t)strtoul(argv[1], NULL, 16);
    uint8_t data = (uint8_t)strtoul(argv[2], NULL, 16);

    if (spi_init() != 0)
        return 1;

    printf("Writing 0x%02X to address 0x%06X...\n", data, addr);
    if (spi_write_byte(addr, data) != 0) {
        spi_close();
        return 1;
    }

    printf("Write success.\n");
    spi_close();
    return 0;
}
