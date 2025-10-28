#include <stdio.h>
#include <linux/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#include <errno.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <pthread.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <mtd/mtd-abi.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include "../op_common/op_common.h"
#include "drv_common.h"
#include "op_pps.h"

// RTC中断相关全局变量
int g_rtcHdl = -1;
pps_t *pg_PPSReg = NULL;
int rtc_irq_fd, rtc_irq_fd1;
unsigned int intSlotCnt = 0;
unsigned int intPpsCnt = 0;
pthread_t g_rtc_read_thrd_id = 0;
pthread_t g_rtc_read_thrd_id1 = 0;

void rtc_irq_proc2(int signum)
{
    unsigned int tmp = 0;
    // 时间片中断
    if (pg_PPSReg->IntRsp_R16 & INT_RSP_TSLOT)
    {
        // 通过中断清除寄存器写入值，清除中断响应寄存器的相应位
        pg_PPSReg->IntClr_R14 = INT_CLR_TSLOT;
        LOG("RTC ...  Local time:\n");
        LOG("Sec       = (%u)\n", ((pg_PPSReg->LocSecH_R18 << 16) + pg_PPSReg->LocSecL_R17));
        tmp = (pg_PPSReg->LocUsH_R20 << 16) + pg_PPSReg->LocUsL_R19;
        LOG("     uSec       = (%u)\n", ((tmp * 100) / 125));
        intSlotCnt += 1;

        if (intSlotCnt % 100 == 0)
        {
            LOG("rtc_irq_proc: intSlotCnt=%d!\n", intSlotCnt);
        }
    }

    // PPS中断
    if (pg_PPSReg->IntRsp_R16 & INT_RSP_PPS)
    {
        // 通过中断清除寄存器写入值，清除中断响应寄存器的相应位
        pg_PPSReg->IntClr_R14 = INT_CLR_PPS;
        LOG("PPS ***************************************************88 ...  PPS time:\n");
        LOG("Sec       = (%u)\n", ((pg_PPSReg->LocSecH_R18 << 16) + pg_PPSReg->LocSecL_R17));
        LOG("     uSec       = (%u)\n", ((pg_PPSReg->LocUsH_R20 << 16) + pg_PPSReg->LocUsL_R19));
        intPpsCnt += 1;

        if (intPpsCnt % 100 == 0)
        {
            LOG("rtc_irq_proc: intPpsCnt=%d!\n", intPpsCnt);
        }
    }

    LOG("compare reg coming......");
}

void rtc_irq_proc1()
{
    LOG("compare reg coming......");
}

void *rtc_local_sec_print(void *arg)
{
    static uint32_t sec_old = 0;
    uint32_t sec = 0;
    pthread_detach(pthread_self());
    while (1)
    {
        sec = ((pg_PPSReg->LocSecH_R18 << 16) + pg_PPSReg->LocSecL_R17);
        if (sec_old != sec)
        {
            sec_old = sec;
            LOG("Sec       = (%u)\n", sec);
            //打印独立时钟的值
            LOG("FPGA Sec       = (%u)\n", ((pg_PPSReg->LocUsH_R22 << 16) + pg_PPSReg->LocUsH_R21));
            //打印独立时钟的微秒值
            LOG("FPGA uSec       = (%u)\n", ((pg_PPSReg->LocUsH_R24 << 16) + pg_PPSReg->LocUsH_R23));
        }
        usleep(10000);
    }
}

void rtc_irq_proc()
{
    int ret = 0;
    char value_read[3];
    char len;
    int hdl = -1;
    struct timeval tv;
    struct tm *tm_info;
    char buffer[26];
    gettimeofday(&tv, NULL); //  获取当前时间
    // 将 timeval 结构中的时间转为 tm 结构
    tm_info = localtime(&tv.tv_sec);

    // 格式化时间
    strftime(buffer, 26, "%Y-%m-%d %H:%M:%S", tm_info);
    // 打印当前时间和微秒部分
    LOG("Current time: %s.%06ld\n", buffer, tv.tv_usec);
}

void pps_open()
{
    //	unsigned int *rtcAddr, data;
    int Oflags;
    int rtcEnHdl = -1;
    char path[64];
    int ret = -1;

    // memset(path, 0, sizeof(path));
    // snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", PL_GPO_RTC);
    // if (-1 == access(path, F_OK))
    // {
    //     gpio_export(PL_GPO_RTC);
    //     gpio_direction(PL_GPO_RTC, GPIO_DIR_OUT);
    // }
#if 0
    int err = pthread_create(&g_rtc_read_thrd_id, NULL, rtc_irq_read_thrd, NULL);
    if (err != 0)
    {
        LOG("Create can_sample_thrd fail: %s\n", strerror(err));
        return;
    }

    rtc_irq_fd = open("/dev/rtc1_irq", O_RDWR);
    if (rtc_irq_fd < 0)
    {
        LOG("/dev/rtc1_irq can't open!\n");
    }
#else
    signal(SIGIO, rtc_irq_proc);
    rtc_irq_fd = open("/dev/rtc1_irq", O_RDWR);
    if (rtc_irq_fd < 0)
    {
        printf("/dev/rtc1_irq can't open!\n");
    }
    // 设置进程为RTC中断设备的拥有者
    fcntl(rtc_irq_fd, F_SETOWN, getpid());
    // 获取当前文件状态标志
    Oflags = fcntl(rtc_irq_fd, F_GETFL);
    // 设置文件状态标志为异步模式
    fcntl(rtc_irq_fd, F_SETFL, Oflags | FASYNC);
#endif
}



void pps_close()
{
    devm_unmap((void *)pg_PPSReg, PPS_MEM_LEN, &g_rtcHdl);
    pg_PPSReg = NULL;
}

void pps_dump()
{
    if (pg_PPSReg == NULL)
        return;

    LOG("--------------- PPS REG INFO --------------\n");
    LOG("Ctrl_R0 = 0x%04x(%u)\n", pg_PPSReg->Ctrl_R0, pg_PPSReg->Ctrl_R0);
    LOG("Earth timing:\n");
    LOG("  SecH_R2          = 0x%04x(%u)\n", pg_PPSReg->SecH_R2, pg_PPSReg->SecH_R2);
    LOG("  SecL_R1          = 0x%04x(%u)\n", pg_PPSReg->SecL_R1, pg_PPSReg->SecL_R1);
    LOG("  UsH_R6           = 0x%04x(%u)\n", pg_PPSReg->UsH_R6, pg_PPSReg->UsH_R6);
    LOG("  UsL_R5           = 0x%04x(%u)\n", pg_PPSReg->UsL_R5, pg_PPSReg->UsL_R5);
    LOG("Center timing:\n");
    LOG("  SecDifH_R4       = 0x%04x(%u)\n", pg_PPSReg->SecDifH_R4, pg_PPSReg->SecDifH_R4);
    LOG("  SecDifL_R3       = 0x%04x(%u)\n", pg_PPSReg->SecDifL_R3, pg_PPSReg->SecDifL_R3);
    LOG("  UsDifH_R8        = 0x%04x(%u)\n", pg_PPSReg->UsDifH_R8, pg_PPSReg->UsDifH_R8);
    LOG("  UsDifL_R7        = 0x%04x(%u)\n", pg_PPSReg->UsDifL_R7, pg_PPSReg->UsDifL_R7);
    LOG("Average timing:\n");
    LOG("  AvgTimInt_R9     = 0x%04x(%u)\n", pg_PPSReg->AvgTimInt_R9, pg_PPSReg->AvgTimInt_R9);
    LOG("  AvgTimSecDif_R10 = 0x%04x(%u)\n", pg_PPSReg->AvgTimSecDif_R10, pg_PPSReg->AvgTimSecDif_R10);
    LOG("  AvgTimUsDifH_R12 = 0x%04x(%u)\n", pg_PPSReg->AvgTimUsDifH_R12, pg_PPSReg->AvgTimUsDifH_R12);
    LOG("  AvgTimUsDifL_R11 = 0x%04x(%u)\n", pg_PPSReg->AvgTimUsDifL_R11, pg_PPSReg->AvgTimUsDifL_R11);
    LOG("Interrupt Registors:\n");
    LOG("  IntEn_R13        = 0x%04x(%u)\n", pg_PPSReg->IntEn_R13, pg_PPSReg->IntEn_R13);
    LOG("  IntClr_R14       = 0x%04x(%u)\n", pg_PPSReg->IntClr_R14, pg_PPSReg->IntClr_R14);
    LOG("  IntIntv_R15      = 0x%04x(%u)\n", pg_PPSReg->IntIntv_R15, pg_PPSReg->IntIntv_R15);
    LOG("  IntRsp_R16       = 0x%04x(%u)\n", pg_PPSReg->IntRsp_R16, pg_PPSReg->IntRsp_R16);
    LOG("Local time:\n");
    LOG("  LocSecH_R18      = 0x%04x(%u)\n", pg_PPSReg->LocSecH_R18, pg_PPSReg->LocSecH_R18);
    LOG("  LocSecL_R17      = 0x%04x(%u)\n", pg_PPSReg->LocSecL_R17, pg_PPSReg->LocSecL_R17);
    LOG("  LocUsH_R20       = 0x%04x(%u)\n", pg_PPSReg->LocUsH_R20, pg_PPSReg->LocUsH_R20);
    LOG("  LocUsL_R19       = 0x%04x(%u)\n", pg_PPSReg->LocUsL_R19, pg_PPSReg->LocUsL_R19);
    LOG("independence time:\n");
    LOG("  LocUsH_R21       = 0x%04x(%u)\n", pg_PPSReg->LocUsH_R21, pg_PPSReg->LocUsH_R21);
    LOG("  LocUsH_R22       = 0x%04x(%u)\n", pg_PPSReg->LocUsH_R22, pg_PPSReg->LocUsH_R22);
    LOG("  LocUsH_R23       = 0x%04x(%u)\n", pg_PPSReg->LocUsH_R23, pg_PPSReg->LocUsH_R23);
    LOG("  LocUsH_R24       = 0x%04x(%u)\n", pg_PPSReg->LocUsH_R24, pg_PPSReg->LocUsH_R24);
    LOG("contrast time:\n");
    LOG("  LocUsH_R25       = 0x%04x(%u)\n", pg_PPSReg->LocUsH_R25, pg_PPSReg->LocUsH_R25);
    LOG("  LocUsH_R26       = 0x%04x(%u)\n", pg_PPSReg->LocUsH_R26, pg_PPSReg->LocUsH_R26);
    LOG("  LocUsH_R27       = 0x%04x(%u)\n", pg_PPSReg->LocUsH_R27, pg_PPSReg->LocUsH_R27);
    LOG("  LocUsH_R28       = 0x%04x(%u)\n", pg_PPSReg->LocUsH_R28, pg_PPSReg->LocUsH_R28);
    LOG("lianXuBiJiao BuChang:\n");
    LOG("  LocUsH_R29       = 0x%04x(%u)\n", pg_PPSReg->LocUsH_R29, pg_PPSReg->LocUsH_R29);
    LOG("  LocUsH_R30       = 0x%04x(%u)\n", pg_PPSReg->LocUsH_R30, pg_PPSReg->LocUsH_R30);
    LOG("  LocUsH_R31       = 0x%04x(%u)\n", pg_PPSReg->LocUsH_R31, pg_PPSReg->LocUsH_R31);
    LOG("  LocUsH_R32       = 0x%04x(%u)\n", pg_PPSReg->LocUsH_R32, pg_PPSReg->LocUsH_R32);
}

void assign_32bit_to_registers(unsigned int value)
{
    // 提取32位数值的低16位和高16位并赋值给寄存器

    unsigned short tmp1 = (unsigned short)(value & 0xFFFF); // 低16位
    pg_PPSReg->LocUsH_R25 = tmp1;
    unsigned short tmp2 = (unsigned short)((value >> 16) & 0xFFFF); // 高16位
    pg_PPSReg->LocUsH_R26 = tmp2;

    // 由于32位数据只有两个16位的寄存器，这两个寄存器可以设为0
    pg_PPSReg->LocUsH_R27 = 0; // 不使用
    pg_PPSReg->LocUsH_R28 = 0; // 不使用pp
    LOG("value=0x%x,tmp1=0x%x,tmp2=0x%x,pg_PPSReg->LocUsH_R25=0x%x,pg_PPSReg->LocUsH_R26=0x%x", value, tmp1, tmp2, pg_PPSReg->LocUsH_R25, pg_PPSReg->LocUsH_R26);
}

void pps_test(unsigned char type, unsigned char id, unsigned int data)
{
    pps_t ppsStRd = {0};

    if (pg_PPSReg == NULL)
        return;

    // 读取出来
    // memcpy((void *)&ppsStRd, (void *)pg_PPSReg, sizeof(pps_t));
    ppsStRd.Ctrl_R0 = pg_PPSReg->Ctrl_R0;
    usleep(10);

    ppsStRd.IntEn_R13 = pg_PPSReg->IntEn_R13;
    usleep(10);
    ppsStRd.IntClr_R14 = pg_PPSReg->IntClr_R14;
    usleep(10);
    ppsStRd.IntIntv_R15 = pg_PPSReg->IntIntv_R15;
    usleep(10);
    ppsStRd.IntRsp_R16 = pg_PPSReg->IntRsp_R16;
    usleep(10);

    // pps_dump();
    LOG("pps -> type= %x  data=%d\n", type, data);
    if (type == 1) // pps信号选择 0 :pps1， 1:pps2
    {
        if (id == 0)
            ppsStRd.Ctrl_R0 &= ~(1 << 7);
        else
            ppsStRd.Ctrl_R0 |= 1 << 7;
        pg_PPSReg->Ctrl_R0 = ppsStRd.Ctrl_R0;
    }
    else if (type == 2) // 时间比较使能
    {
        if (id == 0)
            ppsStRd.Ctrl_R0 &= ~(1 << 13);
        else
            ppsStRd.Ctrl_R0 |= 1 << 13;
        pg_PPSReg->Ctrl_R0 = ppsStRd.Ctrl_R0;
    }
    else if (type == 3) // PPS 校时使能
    {
        unsigned short ppsOutEn = id;

        // PPS校时使能
        if (data)
            ppsStRd.Ctrl_R0 = (ppsStRd.Ctrl_R0 & PPS_CTRL_TIM_EN_MASK) | PPS_CTRL_PPS_TIM_EN;
        else
            ppsStRd.Ctrl_R0 = (ppsStRd.Ctrl_R0 & PPS_CTRL_TIM_EN_MASK) & (~PPS_CTRL_PPS_TIM_EN);

        // PPS输出使能控制
        ppsOutEn = (ppsOutEn << 6) & 0x03c0;
        pg_PPSReg->Ctrl_R0 = (ppsStRd.Ctrl_R0 & PPS_CTRL_PPS_OEN_MASK) | ppsOutEn;
    }
    else if (type == 4) // 外部pps信号触发边沿 0:默认下降沿，1：上升沿
    {
        if (id == 0)
            ppsStRd.Ctrl_R0 &= ~(1 << 6);
        else
            ppsStRd.Ctrl_R0 |= 1 << 6;
        pg_PPSReg->Ctrl_R0 = ppsStRd.Ctrl_R0;
    }
    else if (type == 5) // 多次比较功能使能
    {
        LOG("pg_PPSReg->Ctrl_R0 = %d\n", pg_PPSReg->Ctrl_R0);
        if (id == 0)
            ppsStRd.Ctrl_R0 &= ~(1 << 2);
        else
            ppsStRd.Ctrl_R0 |= 1 << 2;
        pg_PPSReg->Ctrl_R0 = ppsStRd.Ctrl_R0;
        LOG("pg_PPSReg->Ctrl_R0 = %d\n", pg_PPSReg->Ctrl_R0);
    }
    else if (type == 0x10) // 时间片中断
    {
        /*
        1)	对reg15时间片中断间隔寄存器赋值,即给出时间片中断的时间间隔，
            时间片可调，间隔为5ms*（该寄存器值+1）
        2)	对reg13[0]中断使能寄存器赋值
            (以下两步在中断子程序中执行)
        3)	等待中断响应，并读取reg16中断响应寄存器，判断reg16[0]是否被置起
        4)	对reg14[0]中断清除寄存器赋值，即清除中断响应寄存器reg16
        */
        pg_PPSReg->IntIntv_R15 = (unsigned short)id; // id单位为5ms
        pg_PPSReg->IntEn_R13 = INT_EN_TSLOT;

        intSlotCnt = 0;
    }
    else if (type == 0x11) // PPS中断
    {
        /*
        1)	对reg13[1]赋值
        2)	等待中断响应，并读取reg16，判断reg16[0]是否被置起
        3)	对reg14[1]赋值，即清除中断响应寄存器reg16
        */
        LOG("pg_PPSReg->IntEn_R13 = %d\n", pg_PPSReg->IntEn_R13);
        pg_PPSReg->IntEn_R13 = INT_EN_PPS;
        LOG("pg_PPSReg->IntEn_R13 = %d\n", pg_PPSReg->IntEn_R13);
        intPpsCnt = 0;
    }
    else if (type == 0x20) // 时钟选择
    {
        // id: 00:默认时钟  01:外部100M时钟1， 02：外部100M时钟2
        if (id != 0)
            ppsStRd.Ctrl_R0 |= 1 << (11 + (id - 1));
        else
            ppsStRd.Ctrl_R0 &= ~(0x3 << 11);
        pg_PPSReg->Ctrl_R0 = ppsStRd.Ctrl_R0;
    }
    else if (type == 0x30) // 设置比较寄存器
    {
        unsigned short tmp1 = (unsigned short)(data & 0xFFFF); // 低16位
        pg_PPSReg->LocUsH_R25 = tmp1;
        unsigned short tmp2 = (unsigned short)((data >> 16) & 0xFFFF); // 高16位
        pg_PPSReg->LocUsH_R26 = tmp2;

        // 由于32位数据只有两个16位的寄存器，这两个寄存器可以设为0
        pg_PPSReg->LocUsH_R27 = 0; // 不使用
        pg_PPSReg->LocUsH_R28 = 0; // 不使用pp
        LOG("value=0x%x,tmp1=0x%x,tmp2=0x%x,pg_PPSReg->LocUsH_R25=0x%x,pg_PPSReg->LocUsH_R26=0x%x",
            data, tmp1, tmp2, pg_PPSReg->LocUsH_R25, pg_PPSReg->LocUsH_R26);
    }
    else if (type == 0x33) // 固定设置比较寄存器，防止下次忘了咋设置了。
    {
        data = 0x11223344;

        unsigned short tmp1 = (unsigned short)(data & 0xFFFF); // 低16位
        pg_PPSReg->LocUsH_R25 = tmp1;
        unsigned short tmp2 = (unsigned short)((data >> 16) & 0xFFFF); // 高16位
        pg_PPSReg->LocUsH_R26 = tmp2;

        // 由于32位数据只有两个16位的寄存器，这两个寄存器可以设为0
        pg_PPSReg->LocUsH_R27 = 0; // 不使用
        pg_PPSReg->LocUsH_R28 = 0; // 不使用pp
        LOG("value=0x%x,tmp1=0x%x,tmp2=0x%x,pg_PPSReg->LocUsH_R25=0x%x,pg_PPSReg->LocUsH_R26=0x%x",
            data, tmp1, tmp2, pg_PPSReg->LocUsH_R25, pg_PPSReg->LocUsH_R26);
    }
    else if (type == 0x40) // 设置连续比较步长
    {
        // data = 0x11223344;

        unsigned short tmp1 = (unsigned short)(data & 0xFFFF); // 低16位
        pg_PPSReg->LocUsH_R29 = tmp1;
        unsigned short tmp2 = (unsigned short)((data >> 16) & 0xFFFF); // 高16位
        pg_PPSReg->LocUsH_R30 = tmp2;

        // 由于32位数据只有两个16位的寄存器，这两个寄存器可以设为0
        pg_PPSReg->LocUsH_R31 = 0; // 不使用
        pg_PPSReg->LocUsH_R32 = 0; // 不使用
        LOG("value=0x%x,tmp1=0x%x,tmp2=0x%x,pg_PPSReg->LocUsH_R29=0x%x,pg_PPSReg->LocUsH_R30=0x%x",
            data, tmp1, tmp2, pg_PPSReg->LocUsH_R29, pg_PPSReg->LocUsH_R30);
    }

    else if (type == 0xff) // 关闭所有校时功能，关闭中断
    {

        pg_PPSReg->Ctrl_R0 = 0;
        pg_PPSReg->IntEn_R13 = 0;
    }

    usleep(100);
    pps_dump();
}