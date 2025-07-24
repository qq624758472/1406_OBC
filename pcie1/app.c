#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdint.h>
#include <string.h>

#define MAP_SIZE 0x4000
#define MAP_MASK (MAP_SIZE - 1)

void *devmemInit(uint64_t regBase);

int main(int argc, char *argv[]) {
    if (argc < 4 || argc > 5) {
        printf("用法: %s <基地址(hex)> <偏移(hex)> <宽度(8|16|32|64)> [写入值(hex)]\n", argv[0]);
        return EXIT_FAILURE;
    }

    uint64_t base = strtoull(argv[1], NULL, 16);
    uint64_t offset = strtoull(argv[2], NULL, 16);
    int width = atoi(argv[3]);
    int do_write = (argc == 5);
    uint64_t write_val = 0;

    if (do_write) {
        write_val = strtoull(argv[4], NULL, 16);
    }

    void *mapBase = devmemInit(base);
    if (mapBase == NULL) {
        return EXIT_FAILURE;
    }

    void *virt_addr = (uint8_t *)mapBase + (offset & MAP_MASK);
    printf("访问虚拟地址: %p (物理: 0x%lX + 0x%lX)\n", virt_addr, base, offset);

    switch (width) {
        case 8:
            if (do_write) {
                printf("原值 8位:  0x%02X\n", *(volatile uint8_t *)virt_addr);
                *(volatile uint8_t *)virt_addr = (uint8_t)write_val;
                printf("写入 8位:  0x%02X\n", (uint8_t)write_val);
            }
            printf("读取 8位:  0x%02X\n", *(volatile uint8_t *)virt_addr);
            break;
        case 16:
            if (do_write) {
                printf("原值 16位: 0x%04X\n", *(volatile uint16_t *)virt_addr);
                *(volatile uint16_t *)virt_addr = (uint16_t)write_val;
                printf("写入 16位: 0x%04X\n", (uint16_t)write_val);
            }
            printf("读取 16位: 0x%04X\n", *(volatile uint16_t *)virt_addr);
            break;
        case 32:
            if (do_write) {
                printf("原值 32位: 0x%08X\n", *(volatile uint32_t *)virt_addr);
                *(volatile uint32_t *)virt_addr = (uint32_t)write_val;
                printf("写入 32位: 0x%08X\n", (uint32_t)write_val);
            }
            printf("读取 32位: 0x%08X\n", *(volatile uint32_t *)virt_addr);
            break;
        case 64:
            if (do_write) {
                printf("原值 64位: 0x%016lX\n", *(volatile uint64_t *)virt_addr);
                *(volatile uint64_t *)virt_addr = (uint64_t)write_val;
                printf("写入 64位: 0x%016lX\n", write_val);
            }
            printf("读取 64位: 0x%016lX\n", *(volatile uint64_t *)virt_addr);
            break;
        default:
            printf("不支持的宽度: %d（仅支持 8 / 16 / 32 / 64）\n", width);
            munmap(mapBase, MAP_SIZE);
            return EXIT_FAILURE;
    }

    munmap(mapBase, MAP_SIZE);
    return EXIT_SUCCESS;
}

void *devmemInit(uint64_t regBase) {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd == -1) {
        perror("无法打开 /dev/mem");
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
