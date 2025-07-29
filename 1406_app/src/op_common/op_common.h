/*
 * @Description:
 * @Version: 2.0
 * @Autor: ruog__
 * @Date: 2024-01-17 11:03:48
 * @LastEditors: ruog__
 * @LastEditTime: 2025-07-11 16:05:10
 */

#ifndef __OP_COMMON__
#define __OP_COMMON__
#include "xil_io.h"
#include "xil_types.h"

//控制当前测试的星是29星还是35星
#define CURRENT_XING_29

#define CTR_BASE_ADDR 0x43ce0000

#define XST_SUCCESS 0L
#define XST_FAILURE 1L
#define XST_DEVICE_NOT_FOUND 2L
#define XST_DEVICE_BLOCK_NOT_FOUND 3L
#define XST_INVALID_VERSION 4L
#define XST_DEVICE_IS_STARTED 5L
#define XST_DEVICE_IS_STOPPED 6L
#define XST_FIFO_ERROR 7L

// 求平均值
#define AVERAGE(x, y) ((x + y) / 2.0)

u32 COMMO_ReadReg(u32 Idx, u32 RegOffset, void *baseAddr);
void COMMO_WriteReg(u32 Idx, u32 RegOffset, void *baseAddr, u32 Data);


#define LOG(fmt, ...) printf("[%s:%d] " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) // 打印日志的宏定义

#define LOG_INFO_ENABLE // 宏定义，启用调试日志功能
#ifdef LOG_INFO_ENABLE
#define LOG2(fmt, ...) printf("[%s:%d] " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) // 打印日志的宏定义
// #define LOG2 LOG
#else
#define LOG2(fmt, ...)
#endif

/* Status of Disk Functions */
typedef unsigned short WORD;
typedef unsigned int DWORD;
typedef unsigned long long U64;
typedef unsigned char BYTE;
typedef BYTE DSTATUS;

#pragma pack(1)
typedef struct _u6
{
    unsigned int u0;
    unsigned char u1;
    unsigned char u2;
} u6;

union data_6Byte
{
    unsigned char data[6];
    u6 value;
};
#pragma pack()
void printf_all(unsigned char *buf, int len);
void printf_all_reg(unsigned char *buf, int len);
/**
 * @brief 6字节变量 加法   再原来基础上，加一个len值，等于多少，保存到这6字节数据中。
 *
 */
void common_6char_add(union data_6Byte *value, unsigned int len);

//===============文件操作接口=============
void common_save_file(char *filePath, void *buf, int len);
void common_read_file(char *filePath, void *buf, int len);

#endif