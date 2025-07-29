// spi_read_byte.c

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define DEV_PATH "/dev/spidev0.0"
#define SPI_SPEED 10000000

#define CMD_READ 0x03

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

int main(int argc, char *argv[]) {
    if (argc != 2) {
        printf("Usage: %s <address_hex>\n", argv[0]);
        return 1;
    }

    uint32_t addr = (uint32_t)strtoul(argv[1], NULL, 16);
    uint8_t data = 0;

    if (spi_init() != 0)
        return 1;

    if (spi_read_byte(addr, &data) != 0) {
        spi_close();
        return 1;
    }

    printf("Read from address 0x%06X: 0x%02X\n", addr, data);
    spi_close();
    return 0;
}
