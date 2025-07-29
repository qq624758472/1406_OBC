/*
 * @Description:
 * @Version: 2.0
 * @Autor: ruog__
 * @Date: 2024-01-17 14:28:13
 * @LastEditors: ruog__
 * @LastEditTime: 2025-02-26 16:05:48
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include "op_common.h"
#include "xil_io.h"
#include "xil_types.h"
#include "xstatus.h"

void *ctrl_all = NULL; // 总控制器，基值0x43c60000
int ctrl_all_hand = -1;

u32 COMMO_ReadReg(u32 Idx, u32 RegOffset, void *baseAddr)
{
    return Xil_In32(((Idx) + (baseAddr) + (RegOffset)));
}
void COMMO_WriteReg(u32 Idx, u32 RegOffset, void *baseAddr, u32 Data)
{
    Xil_Out32(((Idx) + (baseAddr) + (RegOffset)), (u32)(Data));
}

/**
 * @brief 发脉冲信号
 *
 * 寄存器基值
 * 寄存器偏移
 * 哪个比特位发脉冲
 */
void COMMO_SEND_MAICHONG(void *baseAddr, u32 Offset, u32 bit)
{
    volatile u32 data = COMMO_ReadReg(0, Offset, baseAddr);
    data |= 1 << bit;
    COMMO_WriteReg(0, Offset, baseAddr, data);
    usleep(1000);
    data = COMMO_ReadReg(0, Offset, baseAddr);
    data &= ~(1 << bit);
    COMMO_WriteReg(0, Offset, baseAddr, data);
}

void printf_all(unsigned char *buf, int len)
{
    LOG("=====printf_all  START ====>\n");
    for (int i = 0; i < len; i++)
    {
        if (i != 0 && i % 16 == 0)
            LOG("\n");
        LOG(" %02x", buf[i]);
    }
    LOG("=========>\n");
}

/**
 * @brief ?????????????????len??????????????浽??6????????С?
 *
 */
void common_6char_add(union data_6Byte *value, unsigned int len)
{
    unsigned int add = value->value.u0 + len;
    if (add < value->value.u0 || add < len) // ??????????λ??1
    {
        value->value.u1++;
        if (value->value.u1 == 0) // ??????
            value->value.u2++;
    }
    value->value.u0 = add;
}

/**
 * @brief 以追加方式打开文件
 *
 * @param filePath
 * @param buf
 * @param len
 */
void common_save_file(char *filePath, void *buf, int len)
{
    int fd = open(filePath, O_RDWR | O_CREAT | O_APPEND, S_IRUSR | S_IWUSR| 0666);
    if (fd == -1)
    {
        perror("open");
        exit(EXIT_FAILURE);
    }
    u8 * p = (u8 *)buf;
    ssize_t bytes_written = write(fd, p, len);
    if (bytes_written == -1)
    {
        perror("write");
        close(fd);
        exit(EXIT_FAILURE);
    }

    close(fd);
}

/**
 * @brief 保存文件，不追加打开
 *
 * @param filePath
 * @param buf
 * @param len
 */
void common_save_file_noadd(char *filePath, void *buf, int len)
{
    int fd = open(filePath, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR| 0666);
    if (fd == -1)
    {
        perror("open");
        exit(EXIT_FAILURE);
    }

    ssize_t bytes_written = write(fd, buf, len);
    if (bytes_written == -1)
    {
        perror("write");
        close(fd);
        exit(EXIT_FAILURE);
    }

    close(fd);
}

void common_read_file(char *filePath, void *buf, int len)
{
    int fd = open(filePath, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR|0666);
    if (fd == -1)
    {
        perror("open");
        exit(EXIT_FAILURE);
    }

    ssize_t bytes_read = read(fd, buf, len);
    if (bytes_read == -1)
    {
        perror("read");
        close(fd);
        exit(EXIT_FAILURE);
    }

    close(fd);
}

uint32_t swapEndian(uint32_t value)
{
    return ((value & 0xFF000000) >> 24) |
           ((value & 0x00FF0000) >> 8) |
           ((value & 0x0000FF00) << 8) |
           ((value & 0x000000FF) << 24);
}

/**
 * @brief 6个字节的大小端转换
 *
 * @param data
 */
void swap6Endian(u8 *data)
{
    u8 tmp = 0;
    tmp = data[5];
    data[5] = data[0];
    data[0] = tmp;

    tmp = data[4];
    data[4] = data[1];
    data[1] = tmp;

    tmp = data[3];
    data[3] = data[2];
    data[2] = tmp;

    return;
}

/**
 * @brief 总控制寄存器的映射
 *
 */
u32 *common_mmap_ctr(void)
{
    if (ctrl_all != NULL)
        devm_unmap(ctrl_all);

    ctrl_all = devm_map(CTR_BASE_ADDR, 0xffff, &ctrl_all_hand);
    if (ctrl_all == NULL)
    {
        LOG("ctrl_all: mmap fail!\n");
        return -1;
    }

    return (u32 *)ctrl_all;
}
void common_unmap_ctr(void)
{
    devm_unmap(ctrl_all, 0xffff, &ctrl_all_hand);
}