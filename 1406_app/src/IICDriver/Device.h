/********************************************************************
名称：板卡设备定义
功能：定义板卡各功能地址空间划分，声明外设访问接口函数
作者：yifei.zhang
*********************************************************************/

#ifndef __DEVICE_H__
#define __DEVICE_H__
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/*在VXWORKS中stdint.h的内容被定义在vxTypes.h中*/
//typedef signed char int8_t
//typedef unsigned char uint8_t;
//typedef signed short int16_t;
//typedef unsigned short uint16_t;
//typedef signed int int32_t;
//typedef unsigned int uint32_t;
//extern void *IIC_BASE_ADDR;
//#define ZY4
#ifdef ZY4
#define ADDR_BASE  0x86000000
#define ADDR_BASE1 0x88000000
#define ADDR_BASE2 0x8A000000
#define ADDR_BASE3 0x8C000000
#endif

#define DXJ4
#ifdef DXJ4
#define ADDR_BASE  0x64000000
#define ADDR_BASE1 0x66000000
#define ADDR_BASE2 0x68000000
#define ADDR_BASE3 0x6A000000
#endif
//#define HPI_ADDR_BASE 0x86000000
/*龙芯1F地址空间相关*/
#define ISAMODE
#define ISA_ADDR_BASE_1F1 0xBE000000u
#define ISA_ADDR_BASE_1F2 0xBE010000u
#define ISA_ADDR_BASE_1F3 0xBE020000u
#define PCI_ADDR_BASE_1F1 0xB0100000u
#define PCI_ADDR_BASE_1F2 0xB0200000u
#define PCI_ADDR_BASE_1F3 0xB0300000u
#ifdef PCIMODE
#define LS1F_CLOCK_EN 0x00000904
#define LS1F_INT_EN 0x00000920
#define LS1F_INT_STATUS 0x00000924
#define LS1F_OC0_BASE 0x00000200
#define LS1F_MISC_SEL0 0x00000968
#define LS1F_GPIO_OFFSET 0x00000660
#endif
#ifdef ISAMODE
#define LS1F_CLOCK_EN 0x00001904
#define LS1F_INT_EN 0x00001920
#define LS1F_INT_STATUS 0x00001924
#define LS1F_OC0_BASE 0x00001200
#define LS1F_MISC_SEL0 0x00001968
#define LS1F_GPIO_OFFSET 0x00001660
#endif

#define LPB_MISC 0xBF004100
#define GPIO_OE_7_0 (LPB_MISC + 0)
#define GPIO_OE_15_8 (LPB_MISC + 0x01)
#define GPIO_I_7_0 (LPB_MISC + 0x10)
#define GPIO_O_7_0 (LPB_MISC + 0x20)
#define GPIO_I_15_8 (LPB_MISC + 0X11)
#define GPIO_O_15_8 (LPB_MISC + 0x21)

#define EM_Q_PRIORITY 0x01
#define EM_Q_FIFO 0x00
#define LS1F_DEVICE_CLOCK 33000000U
/*用户自定义地址空间，操作系统不管理*/
#define MEM_FREE_START 0xA6400000U
#define SYSTEM_INFO_BASE MEM_FREE_START
/*当前启动镜像0x11/0x22/0x33*/
#define SYSTEM_INFO_CURRENT_IMAGE_0 (SYSTEM_INFO_BASE + 0x3C)
#define SYSTEM_INFO_CURRENT_IMAGE_1 (SYSTEM_INFO_BASE + 0x40)
#define SYSTEM_INFO_CURRENT_IMAGE_2 (SYSTEM_INFO_BASE + 0x44)
/*下次启动镜像没有NANDFLASH的板卡无效*/
#define SYSTEM_INFO_NEXT_BOOT_IMAGE_0 (SYSTEM_INFO_BASE + 0x48)
#define SYSTEM_INFO_NEXT_BOOT_IMAGE_1 (SYSTEM_INFO_BASE + 0x4C)
#define SYSTEM_INFO_NEXT_BOOT_IMAGE_2 (SYSTEM_INFO_BASE + 0x50)
#define MEM_SYSTEM_INFO_SIZE 100000U
/*0xA6500000-0xA67FFFFF空闲*/
#define MEM_SPW_DMA_BASE 0xA6800000U
#define MEM_SPW_DMA_DES_BASE 0xA6800000U
#define MEM_SPW_DMA_DES_TX_BASE (MEM_SPW_DMA_DES_BASE)
#define MEM_SPW_DMA_DES_RX_BASE (MEM_SPW_DMA_DES_BASE + 0x400u)
#define MEM_SPW_DMA_PORT_BASE (MEM_SPW_DMA_DES_BASE + 0x800u)
#define MEM_SPW_DMA_FIFO_BASE 0xA6801000U
#define MEM_SPW_DMA_FIFO_TX_BASE (MEM_SPW_DMA_FIFO_BASE)
#define MEM_SPW_DMA_FIFO_RX_BASE (MEM_SPW_DMA_FIFO_BASE + 0x10000u)
/*0xA6900000-0xA7FFFFFF空闲*/
#define FLASH_BYTESIZE_PAGE 4096u
#define FLASH_BLOCKSIZE_PAGE 128u
#define FLASH_BLOCKHEAD_CODE1 0x0000U
#define FLASH_BLOCKHEAD_CODE2 0x0028U
#define FLASH_BLOCKHEAD_CODE3 0x0050U
/*龙芯1E其他寄存器定义见用户手册*/
#define HSB_MISC_REG 0xBF003200
#define LS1E_INT_BASE HSB_MISC_REG
#define LS1E_INT_EDGE 0x04
#define LS1E_INT_STEER 0x08
#define LS1E_INT_POL 0x0c
#define LS1E_INT_SET 0x10
#define LS1E_INT_CLR 0x14
#define LS1E_INT_IEN 0x18
#define LS1E_INT_ISR 0x1c
#define LS1E300_WD_TIMER (HSB_MISC_REG + 0x30)
#define LS1E300_WD_CTRL (HSB_MISC_REG + 0x34)
#define LS1E300_WD_BACKUP_EN_MASK 0x00000400
#define INT_PCI_INTA 0x00010000u
#define INT_PCI_INTB 0x00020000u
#define INT_PCI_INTC 0x00040000u
#define INT_PCI_INTD 0x00080000u
#define INT_GPIO_INT 0x00100000u
#define INT_LS1F_ALL 0x001F0000u
/*龙芯1F中断相关*/
#define LS1F_INT_UART0_MASK 0x00000001u
#define LS1F_INT_UART1_MASK 0x00000002u
#define LS1F_INT_UART2_MASK 0x00000004u
#define LS1F_INT_UART3_MASK 0x00000008u
#define LS1F_INT_UART4_MASK 0x00000010u
#define LS1F_INT_UART5_MASK 0x00000020u
#define LS1F_INT_UART6_MASK 0x00000040u
#define LS1F_INT_UART7_MASK 0x00000080u
#define LS1F_INT_PCM_OUT0_FRAME_DONE_MASK 0x00000100u
#define LS1F_INT_PCM_OUT1_FRAME_DONE_MASK 0x00000200u
#define LS1F_INT_PCM_OUT2_FRAME_DONE_MASK 0x00000400u
#define LS1F_INT_PCM_OUT_FULL_MASK 0x00000800u
#define LS1F_INT_PCM_OUT0_PROG_MASK 0x00001000u
#define LS1F_INT_PCM_OUT1_PROG_MASK 0x00002000u
#define LS1F_INT_PCM_OUT2_PROG_MASK 0x00004000u
#define LS1F_INT_CAN_BUS_OFF_MASK 0x00010000u
#define LS1F_INT_CAN_INT_MASK 0x00040000u
#define LS1F_INT_CPU_ECC_INT_MASK 0x00080000u
#define LS1F_INT_AD_INT_MASK 0x00100000u
#define LS1F_INT_ECC_ERR_MASK 0x00200000u
#define LS1F_INT_PCM_IN_EMPTY_MASK 0x02000000u
#define LS1F_INT_PCM_IN_FULL_MASK 0x04000000u
#define LS1F_INT_PCM_IN_PROG_MASK 0x08000000u
#define LS1F_INT_PCM_OUT0_EMPTY_MASK 0x10000000u
#define LS1F_INT_PCM_OUT1_EMPTY_MASK 0x20000000u
#define LS1F_INT_PCM_OUT2_EMPTY_MASK 0x40000000u
#define LS1F_INT_SM_MASK 0x80000000u
#define LS1F_INT_PCM 0x7E007F00u
#define LS1F_INT_UART 0x000000FFu
#define LS1F_INT_VECTOR_1553B 0u
#define LS1F_INT_VECTOR_PCM 1u
#define LS1F_INT_VECTOR_UART 2u
/*DRO FPGA控制寄存器，中断和复位对应模块bit位一样*/
/*中断使能*/
#if 1
#define DRO_FPGA_IER0_RGE_ADDR (IIC_BASE_ADDR + 0x00000002)
#define DRO_FPGA_IER1_RGE_ADDR (IIC_BASE_ADDR + 0x00000004)
/*中断清除，写1清除，只写*/
#define DRO_FPGA_ICR0_RGE_ADDR (IIC_BASE_ADDR + 0x00000008)
#define DRO_FPGA_ICR1_RGE_ADDR (IIC_BASE_ADDR + 0x00000010)
/*中断状态，只读*/
#define DRO_FPGA_ISR0_RGE_ADDR (IIC_BASE_ADDR + 0x00000018)
#define DRO_FPGA_ISR1_RGE_ADDR (IIC_BASE_ADDR + 0x00000020)
/*模块复位，写1复位，只写*/
#define DRO_FPGA_RESET0_RGE_ADDR (IIC_BASE_ADDR + 0x00000028)
#define DRO_FPGA_RESET1_RGE_ADDR (IIC_BASE_ADDR + 0x00000030)
#define DRO_FPGA_GPIO_OUT0_ADDR (IIC_BASE_ADDR + 0x00000038)
#define DRO_FPGA_GPIO_OUT1_ADDR (IIC_BASE_ADDR + 0x00000040)
#define DRO_FPGA_GPIO_IN_ADDR (IIC_BASE_ADDR + 0x00000048)
#else
#define DRO_FPGA_IER0_RGE_ADDR (HPI_ADDR_BASE + 0x00000002)
#define DRO_FPGA_IER1_RGE_ADDR (HPI_ADDR_BASE + 0x00000004)
/*中断清除，写1清除，只写*/
#define DRO_FPGA_ICR0_RGE_ADDR (HPI_ADDR_BASE + 0x00000008)
#define DRO_FPGA_ICR1_RGE_ADDR (HPI_ADDR_BASE + 0x00000010)
/*中断状态，只读*/
#define DRO_FPGA_ISR0_RGE_ADDR (HPI_ADDR_BASE + 0x00000018)
#define DRO_FPGA_ISR1_RGE_ADDR (HPI_ADDR_BASE + 0x00000020)
/*模块复位，写1复位，只写*/
#define DRO_FPGA_RESET0_RGE_ADDR (HPI_ADDR_BASE + 0x00000028)
#define DRO_FPGA_RESET1_RGE_ADDR (HPI_ADDR_BASE + 0x00000030)
#define DRO_FPGA_GPIO_OUT0_ADDR (HPI_ADDR_BASE + 0x00000038)
#define DRO_FPGA_GPIO_OUT1_ADDR (HPI_ADDR_BASE + 0x00000040)
#define DRO_FPGA_GPIO_IN_ADDR (HPI_ADDR_BASE + 0x00000048)
#endif
#define DRO_CAN0_MASK 0x0001
#define DRO_CAN1_MASK 0x0002
#define DRO_IIC0_MASK 0x0004
#define DRO_IIC1_MASK 0x0008
#define DRO_IIC2_MASK 0x0010
#define DRO_IIC3_MASK 0x0020
#define DRO_UART0_MASK 0x0001
#define DRO_UART1_MASK 0x0002
#define DRO_UART2_MASK 0x0004
#define DRO_UART3_MASK 0x0008
#define DRO_UART4_MASK 0x0010
#define DRO_UART5_MASK 0x0020
#define DRO_UART6_MASK 0x0040
#define DRO_UART7_MASK 0x0080
#define DRO_UART8_MASK 0x0100
#define DRO_UART9_MASK 0x0200
#define DRO_UART10_MASK 0x0400

/*FPGA本身复位*/
//#define DRO_FPGA_MASK 0x1000
/*实时之中*/
#define DRO_RTC_MASK 0x0040

#define IO_READ32(addr) ((uint32_t)(*(volatile uint32_t *)(addr)))
#define IO_WRITE32(addr, data) (*(volatile uint32_t *)(addr) = ((uint32_t)(data)))
#define IO_READ16(addr) ((uint16_t)(*(volatile uint16_t *)(addr)))
#define IO_WRITE16(addr, data) (*(volatile uint16_t *)(addr) = ((uint16_t)(data)))
#define IO_READ8(addr) ((uint8_t)(*(volatile uint8_t *)(addr)))
#define IO_WRITE8(addr, data) (*(volatile uint8_t *)(addr) = ((uint8_t)(data)))
typedef void (*VOIDFUNC_INTPTR)(uint32_t);
extern void AppInit(void) __attribute__((constructor));
/*NorFlash接口，MorS是主备，0主，1备*/
/*NorFlash全擦*/
extern int NorFlash_ChipErase(uint8_t MorS);
/*NorFlash扇区擦除，每个扇区32KB，0-63是程序，64-127空闲*/
extern int NorFlash_SectorErase(uint32_t sector_num, uint8_t MorS);
/*NorFlash写，需先擦除*/
extern int NorFlash_Write(uint32_t offset_addr, const char *buf, uint32_t len, uint8_t MorS);
/*NorFlash读*/
extern uint32_t NorFlash_Read(char *buf, uint32_t offset_addr, uint32_t len, uint8_t MorS);
/*当前NorFlash主备状态*/
extern int NorFlash_JudgeMaster(void);
/*烧写BOOT，最大120KB*/
extern uint32_t BurnBoot(uint8_t byaCodeBuf[], uint32_t dwLength, uint8_t MorS);
/*烧写BOOT操作系统，最大1MB*/
extern uint32_t BurnOSToNorFlash(uint8_t byaCodeBuf[], uint32_t dwLength, uint8_t MorS);
extern uint32_t BurnOSToNandFlash(uint8_t byaCodeBuf[], uint32_t dwLength, uint8_t Num);
/*烧写应用，最大510KB*/
extern uint32_t BurnAppToNorFlash(uint8_t byaCodeBuf[], uint32_t dwLength, uint8_t MorS);
extern uint32_t BurnAppToNandFlash(uint8_t byaCodeBuf[], uint32_t dwLength, uint8_t Num);
/*复制程序，用法见《》*/
extern uint8_t CopyCode(uint8_t bySrc, uint8_t byDes);
/*停止喂狗，复位*/
extern void Reset(uint8_t byType);
extern int nand_erase_nand(uint32_t block);
extern int nand_write_page(uint8_t buf[], uint32_t page);
extern int nand_read_page(uint8_t buffer[], uint32_t page);
#if 0
extern void LS1F2OCTransmit(uint32_t dwOCMask, uint16_t wMsgNum);
extern uint32_t CodeRead(uint8_t byaCodeBuf[], uint8_t CodeType);
//extern void LS1FIntInit(void);
//extern void LS1FIntConnect(uint8_t u8IntVector, VOIDFUNC_INTPTR func);
//extern uint8_t GKIntLock(void);
//extern uint8_t GKIntUnlock(void);
extern void ISR_EN_SET_ENABLE(uint32_t dwIsrPin);
extern void ISR_EN_SET_DISABLE(uint32_t dwIsrPin);
#define ISA_READ8(addr) (IO_READ8((((addr)&0x7FFF) << 1) | ((addr)&0xFFFF0000)))
#define ISA_WRITE8(addr, data) (IO_READ8((((addr)&0x7FFF) << 1) | ((addr)&0xFFFF0000)))
#define ISA_READ16(addr) (IO_READ16(addr))
#define ISA_WRITE16(addr, data) (IO_WRITE16(addr, (data)))
//#define ISA_READ32(addr)         (IO_READ32(addr))
//#define ISA_WRITE32(addr,data)         (IO_WRITE32(addr, (data)))
//#define ISA_READ8(addr)         (IO_READ8((((addr) & 0xFFFF)<<2) | ((addr)&0xFFFF0000)))
//#define ISA_WRITE8(addr,data)         (IO_WRITE8((((addr) & 0xFFFF)<<2) | ((addr)&0xFFFF0000), (data)))
//#define ISA_READ16(addr)         (IO_READ16((((addr) & 0xFFFF)<<1) | ((addr)&0xFFFF0000)))
//#define ISA_WRITE16(addr,data)         (IO_WRITE16((((addr) & 0xFFFF)<<1) | ((addr)&0xFFFF0000), (data)))
extern uint32_t ISA_READ32(uint32_t dwAddr);
extern void ISA_WRITE32(uint32_t dwAddr, uint32_t dwData);
extern void LS1FISARepair(uint32_t u32Addr);
#endif
#endif
