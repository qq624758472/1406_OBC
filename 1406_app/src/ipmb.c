/*
 * @Description: nt
 * @Version: 2.0
 * @Autor: ruog__
 * @Date: 2023-05-12 16:47:27
 * @LastEditors: ruog__
 * @LastEditTime: 2025-03-31 11:30:41
 */
#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <stdbool.h>
#include <pthread.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <stdlib.h>
#include "i2c_access.h"
#include "ipmb.h"
#include <string.h>
#include "drv_common.h"
#include "op_common/op_common.h"

/**
 * @brief 对buf中赋值bufLen个随机值。 buf内存由调用者维护。
 *
 * @param buf
 * @param bufLen  buf空间的最小值。 如果buf内存空间<bufLen，会导致段错误。
 * @return int
 */
int ipmb_genera_random(unsigned char *buf, int bufLen)
{
    int ret = -1;
    int randomData = open("/dev/urandom", O_RDONLY, 0666);
    ssize_t result = read(randomData, buf, bufLen);
    if (result < 0)
    {
        LOG("Failed to read from /dev/urandom\n");
        goto error;
    }
    ret = 0;
    LOG("===生成的随机值start===个数：%d", bufLen);
    ipmb_print_data(buf, bufLen);
    LOG("===生成的随机值end===");
error:
    close(randomData);
    return ret;
}

int ipmb_func(unsigned char **sendBuf, int *sendBufLen, int num)//num : cai
{
    // ipmbUnitCtrl_Ec_Func();

    int ret = -1;
    unsigned char sumCheck = 0;
    unsigned char IpmbDigBuf[] = {0xeb, 0x90, 0x00, 0xe5, 0x00, 0x00, 0xe5};
    unsigned char IpmbGCBuf[] = {0xeb, 0x90, 0x00, 0xe3, 0x00, 0x00, 0xe3};
    unsigned int CHMC_ADDR = 0x1;
    unsigned char readBuf[1024] = {0};
    unsigned char readBuf1[1024] = {0};
    struct _CMD_RECV_E5_Z7 tmpRecvE5;
    memset(&tmpRecvE5, 0, sizeof(struct _CMD_RECV_E5_Z7));
    struct _CMD_RECV_E3_Z7 *tmpRecvE3 = (struct _CMD_RECV_E3_Z7 *)readBuf;
#if 1
    if (i2c_open(0x3) < 0)
    {
        LOG("open i2c%d failed!\n", 0x4);
        return -1;
    }
#if 0
    //==============发送E5请求===================
    memset(readBuf, 0x0, sizeof(readBuf));
    usleep(5000);
    i2cIPMB_Write(CHMC_ADDR, sizeof(IpmbDigBuf), IpmbDigBuf);
    usleep(60000);
    i2cIPMB_Read(CHMC_ADDR, IpmbDigLen, readBuf);
    memcpy(&tmpRecvE5, readBuf, sizeof(struct _CMD_RECV_E5_Z7));
    LOG("\n\r****RECV IpmbDigLen print*****\n\r");
    print_data((char *)readBuf, IpmbDigLen);
    //  for (int cnt = 2; cnt < IpmbDigLen - 1; cnt++)
    //  {
    //      sumCheck += readBuf[cnt];
    //  }
    //  LOG("\n\rIpmbDigLen sum : 0x%x *****\n\r", sumCheck);
    // sleep(3);
    usleep(5000);
#endif
    //==============发送E3请求===================
    // int getE3Len = 6 + 16 * num + 1 ;
    int getE3Len = 6+ 1;
    LOG("==================E3====================");
    memset(readBuf, 0x0, sizeof(readBuf));
    unsigned char *p = readBuf;
    ret = i2cIPMB_Write(CHMC_ADDR, sizeof(IpmbGCBuf), IpmbGCBuf);
    LOG("ret=%d", ret);
    if(ret < 0)
    {
        LOG("i2cIPMB_Write error");
        return -1;
    }
    usleep(60000);


    ret =  i2cIPMB_Read(CHMC_ADDR, getE3Len, p);
    LOG("ret=%d", ret);
    if(ret < 0)
    {
        LOG("");
        return -1;
    }
    p+= getE3Len;
    for(int i=0;i<num;i++)
    {
        ret =  i2cIPMB_Read(CHMC_ADDR, 16, p);
        LOG("ret=%d", ret);
        if(ret < 0)
        {
            LOG("---->");
            return -1;
        }
        p+=16;
        usleep(1000);
    }
    // i2cIPMB_Read(CHMC_ADDR, 160, readBuf1);

    // 计算N的值
    int n = ipmb_swap_uint16(tmpRecvE3->dataLen) / 16;
    LOG("n:%d,tmpRecvE3->dataLen:%d", n, ipmb_swap_uint16(tmpRecvE3->dataLen));
    LOG("\n\r****getE3Len print*****\n\r");
    print_data((char *)readBuf, ipmb_swap_uint16(tmpRecvE3->dataLen) + 7);
    // print_data((char *)readBuf1, 160);
    // for (int cnt = 2; cnt < getE3Len - 1; cnt++)
    // {
    //     sumCheck += readBuf[cnt];
    // }
    // LOG("\n\rIpmbGcLen sum : 0x%x *****\n\r", sumCheck);
#endif

#if 0 // lsh 单元测试代码
    ipmb_genera_random(tmpRecvE5.dataValue,7);
    ipmb_genera_random(&tmpRecvE3->dataValue, sizeof(struct _CMD_SEND_UP_DATA) * n);
    //ipmb_genera_random(&tmpRecvE3->dataValue, ipmb_swap_uint16(tmpRecvE3->dataLen));
#endif

#if 0
    //=============进行数据的组装================
    //int n = 2;
    *sendBufLen = 7 + sizeof(struct _CMD_SEND_UP_DATA) * n + 13; // 13帧头
    *sendBuf = (unsigned char *)malloc(*sendBufLen);
    if (*sendBuf == NULL)
    {
        LOGERR("malloc error");
        goto error;
    }
    // 赋值包头
    CMD_SEND_UP *p_send = *sendBuf;
    ipmb_crteate_package(p_send, *sendBufLen - 13);
    memcpy(p_send->dataValue, tmpRecvE5.dataValue, 7);
    memcpy(&p_send->dataValueArg, &tmpRecvE3->dataValue, sizeof(struct _CMD_SEND_UP_DATA) * n);
    // ipmb_print_data(*sendBuf, *sendBufLen);
#endif
    ret = 0;
error:
    i2c_close();
    return ret;
}

// 使能IPMC或者关闭IPMC
int ipmbUnitCtrl_Ec_Func()
{
    unsigned int CHMC_ADDR = 0x1;
    unsigned char g_sum = 0;
    unsigned char IpmbPowerCtlBuf[] = {0xeb, 0x90, 0x01, 0xec, 0x00, 0x00, 0xec};

    if (i2c_open(0x3) < 0)
    {
        LOG("open i2c%d failed!\n", 0x4);
        return -1;
    }

    usleep(5000);
    // IpmbPowerCtlBuf[2] = Buf[6];//指定单元标识
    // IpmbPowerCtlBuf[3] = Buf[7];//标识操作加电还是断电
    for (int cnt = 2; cnt < (sizeof(IpmbPowerCtlBuf) - 1); cnt++)
    {
        g_sum += IpmbPowerCtlBuf[cnt];
    }
    IpmbPowerCtlBuf[sizeof(IpmbPowerCtlBuf) - 1] = g_sum;
    ipmb_print_data(IpmbPowerCtlBuf, 7);
    i2cIPMB_Write(CHMC_ADDR, sizeof(IpmbPowerCtlBuf), IpmbPowerCtlBuf);
    usleep(5000);
    i2c_close();
}

int ipmbUnitCtrlFunc(unsigned char *Buf)
{
    unsigned int CHMC_ADDR = 0x1;
    unsigned char g_sum = 0;
    unsigned char IpmbPowerCtlBuf[] = {0xeb, 0x90, 0x02, 0xe1, 0x00, 0x00, 0xe4};

    if (i2c_open(0x4) < 0)
    {
        LOG("open i2c%d failed!\n", 0x4);
        return -1;
    }

    usleep(5000);
    IpmbPowerCtlBuf[2] = Buf[6]; // 指定单元标识
    IpmbPowerCtlBuf[3] = Buf[7]; // 标识操作加电还是断电
    for (int cnt = 2; cnt < (sizeof(IpmbPowerCtlBuf) - 1); cnt++)
    {
        g_sum += IpmbPowerCtlBuf[cnt];
    }
    IpmbPowerCtlBuf[sizeof(IpmbPowerCtlBuf) - 1] = g_sum;
    ipmb_print_data(IpmbPowerCtlBuf, 7);
    i2cIPMB_Write(CHMC_ADDR, sizeof(IpmbPowerCtlBuf), IpmbPowerCtlBuf);
    usleep(5000);
    i2c_close();
}


int ipmbUnitCtrlFunc_api(unsigned char id, unsigned char en)
{
    unsigned int CHMC_ADDR = 0x1;
    unsigned char g_sum = 0;
    unsigned char IpmbPowerCtlBuf[] = {0xeb, 0x90, 0x02, 0xe1, 0x00, 0x00, 0xe4};

    if (i2c_open(0x4) < 0)
    {
        LOG("open i2c%d failed!\n", 0x4);
        return -1;
    }

    usleep(5000);
    IpmbPowerCtlBuf[2] = id; // 指定单元标识
    IpmbPowerCtlBuf[3] = en; // 标识操作加电还是断电
    for (int cnt = 2; cnt < (sizeof(IpmbPowerCtlBuf) - 1); cnt++)
    {
        g_sum += IpmbPowerCtlBuf[cnt];
    }
    IpmbPowerCtlBuf[sizeof(IpmbPowerCtlBuf) - 1] = g_sum;
    ipmb_print_data(IpmbPowerCtlBuf, 7);
    i2cIPMB_Write(CHMC_ADDR, sizeof(IpmbPowerCtlBuf), IpmbPowerCtlBuf);
    usleep(5000);
    i2c_close();
}

void ipmb_print_data(unsigned char *p, int n)
{
    // printf("N:%d", n);
    for (int i = 0; i < n; i++)
    {
        if (i % 16 == 0)
            printf("\n");
        printf(" %02x", p[i]);
    }
    printf("\n");
}

void ipmb_crteate_package(CMD_SEND_UP *p, unsigned short dataLen)
{
    p->head[0] = 0xFC;
    p->head[1] = 0x1D;

    p->type = 0x01;
    p->mark = 0xC1; // 之z7板
    p->packSer = 0; // 包序号
    p->timeSec = 0;
    p->dataLen = dataLen;
}

unsigned short ipmb_swap_uint16(unsigned short x)
{
    return (x >> 8) | (x << 8);
}
