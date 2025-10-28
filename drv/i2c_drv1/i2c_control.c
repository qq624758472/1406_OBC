#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <string.h>
#include <getopt.h>

// i2c 读2个字节数据
//  ./i2cControl -c 1 -a 0x50 -r 0x00 -m -n 2
//  i2c 写3个字节数据，分别是0x11,0x22,0x33
//  ./i2cControl -c 1 -a 0x50 -r 0x00 -W -n 3 -d 0x11,0x22,0x33

// 与驱动中定义一致的数据结构
struct i2c_data
{
    uint8_t dev_addr;
    uint8_t reg_addr;
    uint32_t len;
    uint8_t *data;
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
#define I2C_READ_BYTES _IOWR(I2C_IOCTL_MAGIC, 4, struct i2c_data)
#define I2C_WRITE_BYTES _IOW(I2C_IOCTL_MAGIC, 5, struct i2c_data)

// 设备节点路径
#define I2C_DEVICE "/dev/i2c_controller"
// 最大支持的字节数
#define MAX_BYTES 256

void print_usage(const char *prog_name)
{
    fprintf(stderr, "用法: %s [选项]\n", prog_name);
    fprintf(stderr, "选项:\n");
    fprintf(stderr, "  -c, --channel <num>    设置I2C通道(1-5)，必需选项\n");
    fprintf(stderr, "  -a, --address <addr>   设置从设备地址(十六进制)，必需选项\n");
    fprintf(stderr, "  -r, --register <reg>   设置寄存器地址(十六进制)，必需选项\n");
    fprintf(stderr, "  -w, --write <data>     写入1个字节数据(十六进制)，写操作\n");
    fprintf(stderr, "  -W, --write-multi      写入多个字节数据，配合-n和-d选项\n");
    fprintf(stderr, "  -d, --data <data>      多个字节数据，用逗号分隔(如:0x12,0x34,0xab)\n");
    fprintf(stderr, "  -1, --read1            读取1个字节，读操作\n");
    fprintf(stderr, "  -2, --read2            读取2个字节，读操作\n");
    fprintf(stderr, "  -m, --read-multi       读取多个字节，配合-n选项\n");
    fprintf(stderr, "  -n, --num-bytes <num>  指定多字节操作的字节数\n");
    fprintf(stderr, "  -h, --help             显示帮助信息\n");
    fprintf(stderr, "\n示例:\n");
    fprintf(stderr, "  读1字节: %s -c 1 -a 0x50 -r 0x00 -1\n", prog_name);
    fprintf(stderr, "  读2字节: %s -c 2 -a 0x51 -r 0x01 -2\n", prog_name);
    fprintf(stderr, "  读5字节: %s -c 3 -a 0x52 -r 0x02 -m -n 5\n", prog_name);
    fprintf(stderr, "  写1字节: %s -c 3 -a 0x52 -r 0x02 -w 0xab\n", prog_name);
    fprintf(stderr, "  写3字节: %s -c 4 -a 0x53 -r 0x03 -W -n 3 -d 0x11,0x22,0x33\n", prog_name);
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

// 解析多个十六进制数据
int parse_multi_hex(const char *str, uint8_t *buf, uint32_t max_len, uint32_t *actual_len)
{
    char *token;
    char *str_copy = strdup(str);
    uint32_t count = 0;

    if (!str_copy)
        return -1;

    token = strtok(str_copy, ",");
    while (token != NULL && count < max_len)
    {
        if (parse_hex(token, &buf[count]) != 0)
        {
            free(str_copy);
            return -1;
        }
        count++;
        token = strtok(NULL, ",");
    }

    *actual_len = count;
    free(str_copy);
    return 0;
}

int main(int argc, char *argv[])
{
    int fd;
    int channel = -1;
    uint8_t dev_addr = 0, reg_addr = 0, write_data = 0;
    uint32_t num_bytes = 0;
    uint8_t *data_buf = NULL;
    char *data_str = NULL;
    int read1 = 0, read2 = 0, read_multi = 0;
    int write = 0, write_multi = 0;
    int opt;

    // 长选项定义
    static struct option long_options[] = {
        {"channel", required_argument, 0, 'c'},
        {"address", required_argument, 0, 'a'},
        {"register", required_argument, 0, 'r'},
        {"write", required_argument, 0, 'w'},
        {"write-multi", no_argument, 0, 'W'},
        {"data", required_argument, 0, 'd'},
        {"read1", no_argument, 0, '1'},
        {"read2", no_argument, 0, '2'},
        {"read-multi", no_argument, 0, 'm'},
        {"num-bytes", required_argument, 0, 'n'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}};

    // 解析命令行参数
    while ((opt = getopt_long(argc, argv, "c:a:r:w:Wd:12mn:h", long_options, NULL)) != -1)
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
        case 'W':
            write_multi = 1;
            break;
        case 'd':
            data_str = optarg;
            break;
        case '1':
            read1 = 1;
            break;
        case '2':
            read2 = 1;
            break;
        case 'm':
            read_multi = 1;
            break;
        case 'n':
            num_bytes = atoi(optarg);
            if (num_bytes < 0 || num_bytes > MAX_BYTES)
            {
                fprintf(stderr, "字节数必须在0-%d之间\n", MAX_BYTES);
                return -1;
            }
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
        fprintf(stderr, "缺少必需的参数(通道、地址或寄存器)\n");
        print_usage(argv[0]);
        return -1;
    }

    // 检查操作类型是否唯一
    int op_count = read1 + read2 + read_multi + write + write_multi;
    if (op_count != 1)
    {
        fprintf(stderr, "必须且只能指定一种操作类型(-w, -W, -1, -2, -m)\n");
        print_usage(argv[0]);
        return -1;
    }

    // 检查多字节操作的参数
    if ((read_multi || write_multi) && num_bytes < 0) //
    {
        fprintf(stderr, "多字节操作必须指定字节数(-n)\n");
        return -1;
    }

    // 为多字节操作分配缓冲区
    if (read_multi || write_multi)
    {
        if (num_bytes <= 0)
            num_bytes++;
        data_buf = malloc(num_bytes);
        if (!data_buf)
        {
            perror("内存分配失败");
            return -1;
        }
        memset(data_buf, 0, num_bytes);
    }

    // 解析多字节写入数据
    if (write_multi)
    {
        if (!data_str)
        {
            fprintf(stderr, "写入多字节数据必须指定数据(-d)\n");
            if (num_bytes)
                free(data_buf);
            return -1;
        }

        uint32_t actual_len;
        if (parse_multi_hex(data_str, data_buf, num_bytes, &actual_len) != 0)
        {
            fprintf(stderr, "无效的多字节数据格式: %s\n", data_str);
            if (num_bytes)
                free(data_buf);
            return -1;
        }

        if (actual_len != num_bytes)
        {
            fprintf(stderr, "数据个数不匹配，预期%d个，实际%d个\n", num_bytes, actual_len);
            if (num_bytes)
                free(data_buf);
            return -1;
        }
    }

    // 检查通道范围
    if (channel < 1 || channel > 5)
    {
        fprintf(stderr, "I2C通道必须在1-5之间\n");
        if (num_bytes)
            free(data_buf);
        return -1;
    }

    // 打开I2C设备
    fd = open(I2C_DEVICE, O_RDWR);
    if (fd < 0)
    {
        perror("无法打开I2C设备");
        fprintf(stderr, "请确保驱动已加载且设备节点存在: %s\n", I2C_DEVICE);
        if (num_bytes)
            free(data_buf);
        return -1;
    }

    // 设置I2C通道
    if (ioctl(fd, I2C_SET_CHANNEL, &channel) < 0)
    {
        perror("设置I2C通道失败");
        close(fd);
        if (num_bytes)
            free(data_buf);
        return -1;
    }

    // 执行写1字节操作
    if (write)
    {
        struct i2c_data data = {
            .dev_addr = dev_addr,
            .reg_addr = reg_addr,
            .data = write_data};

        printf("写入1字节: 通道=%d, 从设备地址=0x%02X, 寄存器=0x%02X, 数据=0x%02X\n",
               channel, dev_addr, reg_addr, write_data);

        if (ioctl(fd, I2C_WRITE_BYTE, &data) < 0)
        {
            perror("I2C写操作失败");
            close(fd);
            if (num_bytes)
                free(data_buf);
            return -1;
        }

        printf("写操作成功\n");
    }
    // 执行写多字节操作
    else if (write_multi)
    {
        struct i2c_data data = {
            .dev_addr = dev_addr,
            .reg_addr = reg_addr,
            .len = num_bytes,
            .data = data_buf};

        printf("写入%d字节: 通道=%d, 从设备地址=0x%02X, 寄存器=0x%02X\n", num_bytes, channel, dev_addr, reg_addr);
        printf("写入数据: ");
        if (num_bytes > 0)
        {
            for (uint32_t i = 0; i < num_bytes; i++)
            {
                printf("0x%02X ", data_buf[i]);
            }
        }
        printf("\n");

        if (ioctl(fd, I2C_WRITE_BYTES, &data) < 0)
        {
            perror("I2C多字节写操作失败");
            close(fd);
            free(data_buf);
            return -1;
        }

        printf("多字节写操作成功\n");
    }
    // 单个字节读已经废弃
#if 0
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
            free(data_buf);
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
            free(data_buf);
            return -1;
        }

        printf("读取成功: 0x%02X 0x%02X\n", data.data1, data.data2);
    }
#endif
    // 执行读多字节操作
    else if (read_multi)
    {
        struct i2c_data data = {
            .dev_addr = dev_addr,
            .reg_addr = reg_addr,
            .len = num_bytes,
            .data = data_buf};

        printf("读取%d字节: 通道=%d, 从设备地址=0x%02X, 寄存器=0x%02X\n",
               num_bytes, channel, dev_addr, reg_addr);

        if (ioctl(fd, I2C_READ_BYTES, &data) < 0)
        {
            perror("I2C多字节读操作失败");
            close(fd);
            free(data_buf);
            return -1;
        }

        printf("读取成功: ");
        for (uint32_t i = 0; i < num_bytes; i++)
        {
            printf("0x%02X ", data_buf[i]);
        }
        printf("\n");
    }

    // 清理资源
    close(fd);
    if (data_buf)
        free(data_buf);

    return 0;
}
