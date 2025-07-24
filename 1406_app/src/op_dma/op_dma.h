/*
 * @Description:
 * @Version: 2.0
 * @Autor: ruog__
 * @Date: 2024-01-25 11:24:38
 * @LastEditors: ruog__
 * @LastEditTime: 2024-09-04 11:10:05
 */
#ifndef __OP_DMA__
#define __OP_DMA__

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

#include "../op_common/xil_io.h"
#include "../op_common/xil_types.h"

#define BUF_SIZE (512 * 1024)
#define HONGWAI_PACK_SIZE(n) (n ? ((32 * 1024 - 1) * n) : 0)

// 4路dma  当前只用一路
#define DMA0_BASE_ADDR 0x40420000
#define DMA1_BASE_ADDR 0x40430000

// 软件控制读指针大小，但是逻辑的深度需要结合读指针进行计算
#define CHANNEL_0_READ_POINT (0x0c) // 读指针偏移
#define CHANNEL_0_WRIT_POINT (0x78) // 写指针偏移
#define CHANNEL_0_DDR_DEPTH (0x7c)  // ddr深度

#define CHANNEL_1_READ_POINT (0x0c) // 读指针偏移
#define CHANNEL_1_WRIT_POINT (0x78) // 写指针偏移
#define CHANNEL_1_DDR_DEPTH (0x7c)  // ddr深度

#define CHANNEL_2_READ_POINT (0x0c) // 读指针偏移
#define CHANNEL_2_WRIT_POINT (0x78) // 写指针偏移
#define CHANNEL_2_DDR_DEPTH (0x7c)  // ddr深度

#define CHANNEL_3_READ_POINT (0x0c) // 读指针偏移
#define CHANNEL_3_WRIT_POINT (0x78) // 写指针偏移
#define CHANNEL_3_DDR_DEPTH (0x7c)  // ddr深度

#define MM2S_DMACR (0x00)
#define MM2S_DMASR (0x04)
#define MM2S_DMASA (0x18)
#define MM2S_DMALENGTH (0x28)

#define S2MM_DMACR (0x30 + 0x00)
#define S2MM_DMASR (0x30 + 0x04)
#define S2MM_DMASA (0x30 + 0x18)
#define S2MM_DMALENGTH (0x30 + 0x28)

struct dma_regs
{
    unsigned long disrc;
    unsigned long disrcc;
    unsigned long didst;
    unsigned long didstc;
    unsigned long dcon;
    unsigned long dstat;
    unsigned long dcsrc;
    unsigned long dcdst;
    unsigned long dmasktrig;
};

/**
 * @brief 四路DMA寄存器初始化
 *
 * @param channel
 * @return int
 */
int dma_op_init(int channel);
int dma_op_uninit(int channel);

u32 DMA_ReadReg(u32 channel, u32 RegOffset);
void DMA_WriteReg(u32 channel, u32 RegOffset, u32 Data);

void DMA_IP_restart(u32 dmaChannel);
u32 DMA_MM2S_isrunning(u32 dmaChannel);

int dma_store_data_is_ready(int channel);

/**
 * @brief 初始化控制接口   ,寄存器初始化 一次调用. 只要有一路调用了，后续不用调用
 *
 * @param channel 有4个通道， mm2s 控制4个通道。   s2mm控制4个通道。
 * @return int
 */
int dma_ctr_init();
int dma_ctr1_init();

/**
 * @brief 当前dma拷贝了多大数据，需要写入寄存器告诉逻辑
 *
 * @param channel
 * @return int
 */
int dma_mm2s_write_reg_readdatasize(int channel, u32 size);

/**
 * @brief s2mm当前dma拷贝了多大数据，需要写入寄存器告诉逻辑
 *
 * @param channel
 * @return int
 */
int dma_s2mm_write_reg_readdatasize(int channel, u32 size);
/**
 * @brief 控制s2mm的启动和停止
 *
 * @param channel
 * @param stat 0:停止。 1：开始
 * @return int
 */
int dma_s2mm_start_end(int channel, int stat);
/**
 * @brief 判断当前指针状态是否可以写入数据，写到pl ddr的什么位置
 *
 * @param channel
 * @param plAddr 当返回1时有效。
 * @param currentStat 当前反转状态。初始状态为1，表示读指针在写指针前。   2：表示读指针在写指针后.
 * @return int 0：不能写入数据。1:可以写入数据
 */
int dma_s2mm_if_write_data(int channel, u32 *plAddr);


/**
 * @brief 内存到内存的拷贝
 * 
 * @param dmaChannel 
 * @param ByteSize 
 * @param ddrAddr 
 * @return u32 
 */
u32 DMA_MM2S_m2m_start(u32 dmaChannel, unsigned int ByteSize, u32 ddrAddr);

#endif