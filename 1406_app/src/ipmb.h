/*
 * @Description: 
 * @Version: 2.0
 * @Autor: ruog__
 * @Date: 2023-05-12 17:07:53
 * @LastEditors: ruog__
 * @LastEditTime: 2025-03-31 11:31:36
 */
#ifndef __IPMB__
#define __IPMB__
#include <stdio.h>



#pragma pack(1)
struct _CMD_SEND_UP_DATA
{
    //unsigned char dataValue[7];
    unsigned char projectArg[16];
};

//这个包为固定格式的，
struct _CMD_SEND_UP
{
    unsigned char head[2];
    unsigned char type;
    unsigned char mark;
    unsigned short packSer;
    unsigned int  timeSec;
    unsigned short dataLen;
    unsigned char dataValue[7];
    unsigned char dataValueArg;//提供首地址
    // struct _CMD_SEND_UP_DATA data;//这个不定义，后边
    // unsigned char checkSum;
};


//接收发送来的工程参数
struct _CMD_RECV_E3_Z7
{
    unsigned char head[2];
    unsigned char mart;
    unsigned char type;
    unsigned short dataLen;
    unsigned char dataValue; // 用来数据域的首地址 ，N=1 ，N不确定
    // unsigned char checkSum;
};
struct _CMD_RECV_E5_Z7
{
    unsigned char head[2];
    unsigned char mart;
    unsigned char type;
    unsigned short packLen;
    unsigned char dataValue[7];
    unsigned char checkSum;
};
#pragma pack()

typedef struct _CMD_SEND_UP CMD_SEND_UP;
typedef struct _CMD_SEND_UP_DATA CMD_SEND_UP_DATA;

int ipmb_func(unsigned char **sendBuf, int *sendBufLen, int num);
void ipmb_print_data(unsigned char *p, int n);
void ipmb_crteate_package(CMD_SEND_UP *p, unsigned short dataLen);

unsigned short ipmb_swap_uint16(unsigned short x);

int ipmbUnitCtrlFunc_api(unsigned char id, unsigned char en);
#endif