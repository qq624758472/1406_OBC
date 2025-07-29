#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <string.h>
#include <getopt.h>

// 与驱动中定义一致的数据结构
struct i2c_data
{
    uint8_t dev_addr;
    uint8_t reg_addr;
    uint8_t data;
};

struct i2c_two_bytes
{
    uint8_t dev_addr;
    uint8_t reg_addr;
    uint8_t data1;
    uint8_t data2;
};

// IOCTL命令，必须与驱动中定义一致
#define I2C_IOCTL_MAGIC 'i'
#define I2C_SET_CHANNEL _IOW(I2C_IOCTL_MAGIC, 0, int)
#define I2C_WRITE_BYTE _IOW(I2C_IOCTL_MAGIC, 1, struct i2c_data)
#define I2C_READ_BYTE _IOWR(I2C_IOCTL_MAGIC, 2, struct i2c_data)
#define I2C_READ_TWO_BYTES _IOWR(I2C_IOCTL_MAGIC, 3, struct i2c_two_bytes)
#define I2C_READ_BYTES _IOWR(I2C_IOCTL_MAGIC, 4, struct i2c_two_bytes)
// 设备节点路径
#define I2C_DEVICE "/dev/i2c_controller"

void print_usage(const char *prog_name)
{
    fprintf(stderr, "用法: %s [选项]\n", prog_name);
    fprintf(stderr, "选项:\n");
    fprintf(stderr, "  -c, --channel <num>   设置I2C通道(1-5)，必需选项\n");
    fprintf(stderr, "  -a, --address <addr>  设置从设备地址(十六进制)，必需选项\n");
    fprintf(stderr, "  -r, --register <reg>  设置寄存器地址(十六进制)，必需选项\n");
    fprintf(stderr, "  -w, --write <data>    写入数据(十六进制)，写操作\n");
    fprintf(stderr, "  -1, --read1           读取1个字节，读操作\n");
    fprintf(stderr, "  -2, --read2           读取2个字节，读操作\n");
    fprintf(stderr, "  -h, --help            显示帮助信息\n");
    fprintf(stderr, "\n示例:\n");
    fprintf(stderr, "  读操作: %s -c 1 -a 0x50 -r 0x00 -1\n", prog_name);
    fprintf(stderr, "  读操作: %s -c 2 -a 0x51 -r 0x01 -2\n", prog_name);
    fprintf(stderr, "  写操作: %s -c 3 -a 0x52 -r 0x02 -w 0xab\n", prog_name);
}

// 解析十六进制字符串
int parse_hex(const char *str, uint8_t *val)
{
    char *endptr;
    unsigned long num = strtoul(str, &endptr, 16);
    if (*endptr != '\0' || num > 0xFF)
    {
        return -1;
    }
    *val = (uint8_t)num;
    return 0;
}

int main(int argc, char *argv[])
{
    int fd;
    int channel = -1;
    uint8_t dev_addr = 0, reg_addr = 0, write_data = 0;
    int read1 = 0, read2 = 0, write = 0;
    int opt;

    // 长选项定义
    static struct option long_options[] = {
        {"channel", required_argument, 0, 'c'},
        {"address", required_argument, 0, 'a'},
        {"register", required_argument, 0, 'r'},
        {"write", required_argument, 0, 'w'},
        {"read1", no_argument, 0, '1'},
        {"read2", no_argument, 0, '2'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}};

    // 解析命令行参数
    while ((opt = getopt_long(argc, argv, "c:a:r:w:12h", long_options, NULL)) != -1)
    {
        switch (opt)
        {
        case 'c':
            channel = atoi(optarg);
            break;
        case 'a':
            if (parse_hex(optarg, &dev_addr) != 0)
            {
                fprintf(stderr, "无效的从设备地址: %s\n", optarg);
                return -1;
            }
            break;
        case 'r':
            if (parse_hex(optarg, &reg_addr) != 0)
            {
                fprintf(stderr, "无效的寄存器地址: %s\n", optarg);
                return -1;
            }
            break;
        case 'w':
            if (parse_hex(optarg, &write_data) != 0)
            {
                fprintf(stderr, "无效的写入数据: %s\n", optarg);
                return -1;
            }
            write = 1;
            break;
        case '1':
            read1 = 1;
            break;
        case '2':
            read2 = 1;
            break;
        case 'h':
            print_usage(argv[0]);
            return 0;
        default:
            print_usage(argv[0]);
            return -1;
        }
    }

    // 检查必需参数
    if (channel < 0)
    {
        fprintf(stderr, "缺少必需的参数\n");
        print_usage(argv[0]);
        return -1;
    }

    // 检查操作类型是否唯一
    if ((read1 + read2 + write) != 1)
    {
        fprintf(stderr, "必须且只能指定一种操作类型(-w, -1, -2)\n");
        print_usage(argv[0]);
        return -1;
    }

    // 检查通道范围
    if (channel < 1 || channel > 5)
    {
        fprintf(stderr, "I2C通道必须在1-5之间\n");
        return -1;
    }

    // 打开I2C设备
    fd = open(I2C_DEVICE, O_RDWR);
    if (fd < 0)
    {
        perror("无法打开I2C设备");
        fprintf(stderr, "请确保驱动已加载且设备节点存在: %s\n", I2C_DEVICE);
        return -1;
    }

    // 设置I2C通道
    if (ioctl(fd, I2C_SET_CHANNEL, &channel) < 0)
    {
        perror("设置I2C通道失败");
        close(fd);
        return -1;
    }

    // 执行写操作
    if (write)
    {
        struct i2c_data data = {
            .dev_addr = dev_addr,
            .reg_addr = reg_addr,
            .data = write_data};

        printf("写入: 通道=%d, 从设备地址=0x%02X, 寄存器=0x%02X, 数据=0x%02X\n",
               channel, dev_addr, reg_addr, write_data);

        if (ioctl(fd, I2C_WRITE_BYTE, &data) < 0)
        {
            perror("I2C写操作失败");
            close(fd);
            return -1;
        }

        printf("写操作成功\n");
    }
    // 执行读1字节操作
    else if (read1)
    {
        struct i2c_data data = {
            .dev_addr = dev_addr,
            .reg_addr = reg_addr};

        printf("读取1字节: 通道=%d, 从设备地址=0x%02X, 寄存器=0x%02X\n",
               channel, dev_addr, reg_addr);

        if (ioctl(fd, I2C_READ_BYTE, &data) < 0)
        {
            perror("I2C读操作失败");
            close(fd);
            return -1;
        }

        printf("读取成功: 0x%02X\n", data.data);
    }
    // 执行读2字节操作
    else if (read2)
    {
        struct i2c_two_bytes data = {
            .dev_addr = dev_addr,
            .reg_addr = reg_addr};

        printf("读取2字节: 通道=%d, 从设备地址=0x%02X, 寄存器=0x%02X\n",
               channel, dev_addr, reg_addr);

        if (ioctl(fd, I2C_READ_TWO_BYTES, &data) < 0)
        {
            perror("I2C读2字节操作失败");
            close(fd);
            return -1;
        }

        printf("读取成功: 0x%02X 0x%02X\n", data.data1, data.data2);
    }

    // 关闭设备
    close(fd);
    return 0;
}
