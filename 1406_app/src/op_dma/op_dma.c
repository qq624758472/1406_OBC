/*
 * @Description:
 * @Version: 2.0
 * @Autor: ruog__
 * @Date: 2023-11-29 16:55:06
 * @LastEditors: ruog__
 * @LastEditTime: 2024-09-04 11:07:57
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h> //文件操作
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <linux/limits.h>
#include <mtd/mtd-abi.h>

#include "op_dma.h"
#include "drv_common.h"
#include "../op_common/xil_io.h"
#include "../op_common/xil_types.h"
#include "../op_common/op_common.h"


// 定义互斥锁,锁住控制寄存器
pthread_mutex_t mutex;
// static volatile struct dma_regs *dma_regs0;
// static volatile struct dma_regs *dma_regs1;
// static volatile struct dma_regs *dma_regs2;
// static volatile struct dma_regs *dma_regs3;

void *dma_regs0 = NULL;
void *dma_regs1 = NULL;
void *dma_regs2 = NULL;
void *dma_regs3 = NULL;
void *dma_regs4 = NULL;
void *dma_regs5 = NULL;
void *dma_regs6 = NULL;
void *dma_regs7 = NULL;

int resetrHdl0 = -1;
int resetrHdl1 = -1;
int resetrHdl2 = -1;
int resetrHdl3 = -1;
int resetrHdl4 = -1;
int resetrHdl5 = -1;
int resetrHdl6 = -1;
int resetrHdl7 = -1;

void *dma_regs_ctr_mm2s = NULL;
void *dma_regs_ctr1_mm2s = NULL;
void *dma_regs_ctr2_mm2s = NULL;
void *dma_regs_ctr_s2mm = NULL;
void *dma_regs_ctr1_s2mm = NULL;
void *dma_regs_ctr2_s2mm = NULL;

int resetrHdl_ctr_mm2s = -1;
int resetrHdl_ctr1_mm2s = -1;
int resetrHdl_ctr2_mm2s = -1;
int resetrHdl_ctr_s2mm = -1;

void tets_dma()
{
    volatile int isWriteReady = COMMO_ReadReg(0, CHANNEL_0_DDR_DEPTH, dma_regs_ctr_mm2s);
    LOG("isWriteReady:0x%08x\n", isWriteReady);
}



int dma_ctr_uninit()
{
    devm_unmap(dma_regs_ctr_mm2s, 0xff, &resetrHdl_ctr_mm2s);
    devm_unmap(dma_regs_ctr_s2mm, 0xff, &resetrHdl_ctr_s2mm);
}
int dma_ctr1_uninit()
{
    devm_unmap(dma_regs_ctr1_mm2s, 0xff, &resetrHdl_ctr1_mm2s);
    // devm_unmap(dma_regs_ctr_s2mm, 0xffff, &resetrHdl_ctr_s2mm);
}
int dma_ctr2_uninit()
{
    devm_unmap(dma_regs_ctr2_mm2s, 0xff, &resetrHdl_ctr2_mm2s);
    // devm_unmap(dma_regs_ctr_s2mm, 0xffff, &resetrHdl_ctr_s2mm);
}

/**
 * @brief 四路DMA寄存器初始化
 *
 * @param channel
 * @return int
 */
int dma_op_init(int channel)
{
    LOG2("dma_op_init start\n");
    switch (channel)
    {
    case 0:
        dma_regs0 = devm_map(DMA0_BASE_ADDR, 0xffff, &resetrHdl0);
        if (dma_regs0 == NULL)
        {
            LOG("dma_regs0: mmap fail!\n");
            return -1;
        }
        LOG2("dma_regs0:0x%x\n", dma_regs0);
        break;
    case 1:
        dma_regs1 = devm_map(DMA1_BASE_ADDR, 0xffff, &resetrHdl1);
        if (dma_regs1 == NULL)
        {
            LOG("dma_regs1: mmap fail!\n");
            return -1;
        }
        break;
    default:
        LOG("arg is error\n");
        break;
    }
    LOG("dma_op_init SUCCESS\n");
    return 0;
}

int dma_op_uninit(int channel)
{
    switch (channel)
    {
    case 0:
        devm_unmap(dma_regs0, 0xffff, &resetrHdl0);
        break;
    case 1:
        devm_unmap(dma_regs1, 0xffff, &resetrHdl1);
        break;
    default:
        LOG("arg is error\n");
        break;
    }

    return 0;
}

u32 DMA_ReadReg(u32 channel, u32 RegOffset)
{
    switch (channel)
    {
    case 0:
        return Xil_In32(dma_regs0 + RegOffset);
    case 1:
        return Xil_In32(dma_regs1 + RegOffset);
    default:
        LOG("arg is error\n");
        break;
    }
}
void DMA_WriteReg(u32 channel, u32 RegOffset, u32 Data)
{
    switch (channel)
    {
    case 0:
        return Xil_Out32(dma_regs0 + RegOffset, (u32)(Data));
    case 1:
        return Xil_Out32(dma_regs1 + RegOffset, (u32)(Data));
    default:
        LOG("arg is error\n");
        break;
    }
}

void DMA_IP_restart(u32 dmaChannel)
{
    DMA_WriteReg(dmaChannel, MM2S_DMACR, 0x04);
}

u32 DMA_MM2S_isrunning(u32 dmaChannel)
{
    u32 asr = DMA_ReadReg(dmaChannel, MM2S_DMASR);
    LOG2("dma_regs0:0x%x, dmaChannel:0x%x, sr:0x%x\n", dma_regs0, dmaChannel, asr);
    if (!(asr & 0x1))
    {
        LOG("DMA %d is halted\n", dmaChannel);
        if (!(asr & 0x2))
            LOG("DMA %d is running\n", dmaChannel);
        return -1;
    }
    return 0;
}

/**
 * @brief 内存到内存的拷贝
 * 
 * @param dmaChannel 
 * @param diskNum 
 * @param secStart 
 * @param ByteSize 
 * @param ddrAddr 
 * @return u32 
 */
u32 DMA_MM2S_m2m_start(u32 dmaChannel, unsigned int ByteSize, u32 ddrAddr)
{
    DMA_WriteReg(dmaChannel, MM2S_DMACR, 0x04);

    if (DMA_MM2S_isrunning(dmaChannel) == -1)
        return -1;

    // 1.通过将运行/停止位设置为1（MM2S_DMACR.RS=1）来启动MM2S通道运行。暂停位（DMASR.halted）应取消断言，指示MM2S通道正在运行。
    // 2.如果需要，通过向MM2S_DMACR写入1来启用中断。IrgEn和MM2S _ DMACR。错误_IrgEn。当AXI DMA配置为直接寄存器模式时，不使用延迟中断、延迟计数和阈值计数。
    u32 cr = DMA_ReadReg(dmaChannel, MM2S_DMACR);
    u32 sr = DMA_ReadReg(dmaChannel, MM2S_DMASR);
    LOG("cr :%X, sr :%X\n", cr, sr);

    cr |= (0x1 << 0); // RS 比特位置1
    cr |= (0x1 << 12);
    cr |= (0x1 << 14);
    // sr &= ~(1 << 0);
    // 0位置 0表示正在运行
    DMA_WriteReg(dmaChannel, MM2S_DMACR, cr);
    DMA_WriteReg(dmaChannel, MM2S_DMASR, sr);

    // 3.将有效的源地址写入MM2S_SA寄存器。如果为大于32的地址空间配置AXI DMA，则对MM2S_SA MSB寄存器进行编程。如果AXI DMA未配置为数据重新对齐，则必须对齐有效地址，否则会出现未定义的结果。
    // 什么是对齐的或未对齐的是基于strean数据宽度。在Micro模式下配置AXI_DMA时，您有责任指定正确的地址。Micro DMA不考虑4K边界。
    DMA_WriteReg(dmaChannel, MM2S_DMASA, ddrAddr);
    //Indicates the number of bytes to transfer for the MM2S channel. Writing a non-zero value to this register starts theMM2S transfer.
    //设置后启动dma传输。
    DMA_WriteReg(dmaChannel, MM2S_DMALENGTH, ByteSize);

    while (1)
    {
        sr = 0;
        sr = DMA_ReadReg(dmaChannel, MM2S_DMASR);
        sr &= 0x02;
        if (sr != 0)
        {
            // LOG("Interrupt on Complete\n");
            break;
        }
    }

    sr = 0;
    cr = 0;
    cr = DMA_ReadReg(dmaChannel, MM2S_DMACR);
    sr = DMA_ReadReg(dmaChannel, MM2S_DMASR);
    LOG("cr :%X, sr :%X\n", cr, sr);
    return 0;
}

/**
 * @brief s2mm通道状态检测，异常时reset
 *
 * @return u32
 */
u32 DMA_S2MM_channel_check(u32 dmaChannel, int chan)
{
    if (chan == 2) // s2mm
    {
        register u32 cr = DMA_ReadReg(dmaChannel, S2MM_DMACR);
        register u32 sr = DMA_ReadReg(dmaChannel, S2MM_DMASR);
        // LOG("cr :%X, sr :%X\n", cr, sr);

        if (sr & 1 == 1 && (sr & 2) != 2) // 第0位是1,reset
        {
            cr |= (1 << 2);
            DMA_WriteReg(dmaChannel, S2MM_DMACR, cr);
            sr = DMA_ReadReg(dmaChannel, S2MM_DMASR);
            if ((sr & 2) == 2) // 状态是否变idle
                return 3;      // 重置后通道正常返回
            else
                return 2; // 重置后通道仍然异常返回
        }
        else
        {
            sr = DMA_ReadReg(dmaChannel, S2MM_DMASR);
            if ((sr & 2) == 2) // 状态是否变idle
                return 1;
            else
                return 0;
        }
    }
    else if (chan == 3) // mm2s
    {
    }
    return 0;
}

/**
 * @brief s2mm搬一包数据后需要置1
 *
 * @param channel
 * @return int 0: true; !0: error
 */
int dma_s2mm_complate_settarget(int channel)
{
    COMMO_WriteReg(0, 0x18, dma_regs_ctr_s2mm, 0x01);
    volatile int readDep = COMMO_ReadReg(0, 0x18, dma_regs_ctr_s2mm);
    if (readDep == 0x01)
        return 0;

    LOG("s2mm set target error\n");
    return 1;
}

/**
 * @brief 四个通道。 现在变成2个通道，最多会到3 个通道
 *
 * @param channel
 * @return int
 */
int dma_store_data_is_ready(int channel)
{
    if (channel == 0)
    {
        static int packValue = 0xff;
        pthread_mutex_lock(&mutex);
        // 逻辑将总深度减去读指针深度。所以总深度值=读指针深度值 + ddr3深度值
        volatile int readDep = COMMO_ReadReg(0, CHANNEL_0_READ_POINT, dma_regs_ctr_mm2s);
        // LOG2("dma_regs_ctr_mm2s:0x%08x\n", dma_regs_ctr_mm2s);
        volatile int isWriteReady = COMMO_ReadReg(0, CHANNEL_0_DDR_DEPTH, dma_regs_ctr_mm2s);
        pthread_mutex_unlock(&mutex);
        // LOG2("channel:%d,isWriteReady:0x%08x,packValue:0x%08x,(isWriteReady+readDep):0x%08x\n", channel, isWriteReady, packValue, (isWriteReady + readDep));
        if ((isWriteReady + readDep) >= packValue)
        {
            if (packValue >= (0xff * 4))
                packValue = 0xff;
            else
                packValue += 0xff;
            // LOG("isWriteReady:0x%08x,packValue:0x%08x\n", isWriteReady, packValue);
            return 1;
        }
        else
            return 0;
    }
    else if (channel == 1)
    {
        static int packValue1 = 0xff;
        // 逻辑将总深度减去读指针深度。所以总深度值=读指针深度值 + ddr3深度值
        pthread_mutex_lock(&mutex);
        volatile int readDep = COMMO_ReadReg(0, CHANNEL_1_READ_POINT, dma_regs_ctr1_mm2s);
        volatile int isWriteReady = COMMO_ReadReg(0, CHANNEL_1_DDR_DEPTH, dma_regs_ctr1_mm2s);
        pthread_mutex_unlock(&mutex);
        // if (isWriteReady == 0xff * 2)
        // LOG2("channel:%d,isWriteReady:0x%08x,packValue1:0x%08x,(isWriteReady+readDep):0x%08x\n", channel, isWriteReady, packValue1, (isWriteReady + readDep));
        if ((isWriteReady + readDep) >= packValue1)
        {
            if (packValue1 >= (0xff * 4))
                packValue1 = 0xff;
            else
                packValue1 += 0xff;
            // LOG("isWriteReady:0x%08x,packValue:0x%08x\n", isWriteReady, packValue);
            return 1;
        }
        else
            return 0;
    }
    else if (channel == 2)
    {
        static int packValue2 = 0xff;
        // 逻辑将总深度减去读指针深度。所以总深度值=读指针深度值 + ddr3深度值
        pthread_mutex_lock(&mutex);
        volatile int readDep = COMMO_ReadReg(0, CHANNEL_2_READ_POINT, dma_regs_ctr2_mm2s);
        volatile int isWriteReady = COMMO_ReadReg(0, CHANNEL_2_DDR_DEPTH, dma_regs_ctr2_mm2s);
        pthread_mutex_unlock(&mutex);
        // if (isWriteReady == 0xff * 2)
        // LOG2("channel:%d,isWriteReady:0x%08x,packValue1:0x%08x,(isWriteReady+readDep):0x%08x\n", channel, isWriteReady, packValue1, (isWriteReady + readDep));
        if ((isWriteReady + readDep) >= packValue2)
        {
            if (packValue2 >= (0xff * 4))
                packValue2 = 0xff;
            else
                packValue2 += 0xff;
            // LOG("isWriteReady:0x%08x,packValue:0x%08x\n", isWriteReady, packValue);
            return 1;
        }
        else
            return 0;
    }
    else if (channel == 3)
    {
        static int packValue3 = HONGWAI_PACK_SIZE(1);
        pthread_mutex_lock(&mutex);
        // 逻辑将总深度减去读指针深度。所以总深度值=读指针深度值 + ddr3深度值
        volatile int readDep = COMMO_ReadReg(0, CHANNEL_3_READ_POINT, dma_regs_ctr2_mm2s);
        volatile int isWriteReady = COMMO_ReadReg(0, CHANNEL_3_DDR_DEPTH, dma_regs_ctr2_mm2s);
        pthread_mutex_unlock(&mutex);
        // if (isWriteReady == 0xff * 2)
        LOG2("channel:%d,isWriteReady:0x%08x,packValue3:0x%08x,(isWriteReady+readDep):0x%08x\n", channel, isWriteReady, packValue3, (isWriteReady + readDep));
        if ((isWriteReady + readDep) >= packValue3)
        {
            if (packValue3 >= (HONGWAI_PACK_SIZE(4)))
                packValue3 = HONGWAI_PACK_SIZE(1);
            else
                packValue3 += HONGWAI_PACK_SIZE(1);
            LOG("isWriteReady:0x%08x,packValue:0x%08x\n", isWriteReady, packValue3);
            return 1;
        }
        else
            return 0;
    }
    LOG("error\n");
    return 0;
}

/**
 * @brief mm2s当前dma拷贝了多大数据，需要写入寄存器告诉逻辑
 *
 * @param channel
 * @return int
 */
int dma_mm2s_write_reg_readdatasize(int channel, u32 size)
{
    LOG("size:0x%04x\n", size);
    u32 offset = 0;
    int isWriteReady = 0;
    switch (channel)
    {
    case 0:
        offset = CHANNEL_0_READ_POINT;
        COMMO_WriteReg(0, offset, dma_regs_ctr_mm2s, size);
        isWriteReady = COMMO_ReadReg(0, offset, dma_regs_ctr_mm2s);
        break;
    case 1:
        offset = CHANNEL_1_READ_POINT;
        COMMO_WriteReg(0, offset, dma_regs_ctr1_mm2s, size);
        isWriteReady = COMMO_ReadReg(0, offset, dma_regs_ctr1_mm2s);
        break;
    case 2:
        offset = CHANNEL_2_READ_POINT;
        COMMO_WriteReg(0, offset, dma_regs_ctr2_mm2s, size);
        isWriteReady = COMMO_ReadReg(0, offset, dma_regs_ctr2_mm2s);
        break;
    case 3:
        offset = CHANNEL_3_READ_POINT;
        COMMO_WriteReg(0, offset, dma_regs_ctr2_mm2s, size);
        isWriteReady = COMMO_ReadReg(0, offset, dma_regs_ctr2_mm2s);
        break;
    default:
        LOG("error\n");
        exit(0);
    }

    if (isWriteReady == size)
    {
        return 1;
    }
    else
    {
        LOG("COMMO_ReadReg error\n");
        return 0;
    }
}

/**
 * @brief s2mm当前dma拷贝了多大数据，需要写入寄存器告诉逻辑
 *
 * @param channel
 * @return int
 */
int dma_s2mm_write_reg_readdatasize(int channel, u32 size)
{
    LOG("size:0x%04x\n", size);
    COMMO_WriteReg(0, 0x20, dma_regs_ctr_s2mm, size);
    int isWriteReady = COMMO_ReadReg(0, 0x20, dma_regs_ctr_s2mm);
    if (isWriteReady == size)
    {
        return 1;
    }
    else
    {
        LOG("COMMO_ReadReg error\n");
        return 0;
    }
}

/**
 * @brief 控制s2mm的启动和停止
 *
 * @param channel
 * @param stat 0:停止。 1：开始
 * @return int
 */
int dma_s2mm_start_end(int channel, int stat)
{
    int ready = COMMO_ReadReg(0, 0x18, dma_regs_ctr_s2mm);
    if (stat == 0)
    {
        ready &= !(0x1);
    }
    else
    {
        ready |= 0x1;
    }
    COMMO_WriteReg(0, 0x18, dma_regs_ctr_s2mm, ready);
    stat = COMMO_ReadReg(0, 0x18, dma_regs_ctr_s2mm);
    if (ready == stat)
    {
        return 1;
    }
    else
    {
        LOG("COMMO_ReadReg error\n");
        return 0;
    }
}

/**
 * @brief 判断当前指针状态是否可以写入数据，写到pl ddr的什么位置
 *
 * @param channel
 * @param plAddr 当返回1时有效。
 * @param currentStat 当前反转状态。初始状态为1，表示读指针在写指针前。   2：表示读指针在写指针后.
 * @return int 0：不能写入数据。1:可以写入数据
 */
int dma_s2mm_if_write_data(int channel, u32 *plAddr)
{
    int readPoint = COMMO_ReadReg(0, 0x20, dma_regs_ctr_s2mm);
    int writePoint = COMMO_ReadReg(0, 0x30, dma_regs_ctr_s2mm);
    LOG("RP:0x%02x,WP:0x%02x\n", readPoint, writePoint);
    if (writePoint <= readPoint)
    {
        if (readPoint == (0x100 * 3))
        {
            *plAddr = 0x1000000 * 3;
        }
        else
        {
            *plAddr = 0x1000000 * (readPoint / 0x100);
        }
        return 1;
    }
    else if (writePoint > readPoint)
    {
        int posion = (writePoint + 1) / 0x100; // 检测写指针在哪个帧段
        switch (posion)
        {
        case 0: // 写指针在0x0 ~ 0x100之间。
            return 0;
            break;
        case 1:                    // 在0x100 ~ 0x200之间, 可以搬移数据到0 ~ 0x100区域
            if (readPoint < 0x100) // 是否已经搬移数据到0 ~ 0x100中了。
            {
                *plAddr = 0x00;
                return 1;
            }
            else
            {
                // 已经搬移数据到0 ~ 0x100中了。 只能等待写指针让出0x100空间
                return 0;
            }
            break;
        case 2: // 写指针在0x200 ~ 0x300之间。
            if (readPoint == 0)
            {
                *plAddr = 0x00;
                return 1;
            }
            else if (readPoint == 0x100)
            {
                *plAddr = 0x1000000;
                return 1;
            }
            else if (readPoint == 0x200)
            {
                return 0;
            }
            else
            {
                LOG("ERROR\n");
                exit(0); // 程序直接退出
            }
            break;
        case 3: // 写指针在0x300 ~ 0x400之间
            if (readPoint == 0)
            {
                *plAddr = 0x00;
                return 1;
            }
            else if (readPoint == 0x100)
            {
                *plAddr = 0x1000000;
                return 1;
            }
            else if (readPoint == 0x200)
            {
                *plAddr = 0x2000000;
                return 1;
            }
            else if (readPoint == 0x300)
            {
                return 0;
            }
            else
            {
                LOG("ERROR\n");
                exit(0); // 程序直接退出
            }
            break;
        default:
            LOG("ERROR\n");
            exit(0); // 程序直接退出
            break;
        }
    }

    LOG("ERROR\n");
    return 0;
}