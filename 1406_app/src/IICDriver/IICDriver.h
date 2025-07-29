/********************************************************************
ï¿½ï¿½ï¿½Æ£ï¿½DRO IICï¿½ï¿½ï¿½ï¿½
ï¿½ï¿½ï¿½Ü£ï¿½DRO IICï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ã»ï¿½Ö»ï¿½ï¿½ÒªÊ¹ï¿½ï¿½ï¿½ï¿½ï¿?4ï¿½ï¿½ï¿½Ó¿Úºï¿½ï¿½ï¿½
IICInit(uint32_t u32Port);
IICConfig(uint32_t u32Port, uint8_t u8Div);
IICMasterSend(uint32_t u32Port, uint8_t u8Addr, uint8_t u8aDataBuf[], uint32_t u32Len);
IICMasterReceive(uint32_t u32Port, uint8_t u8Addr, uint8_t u8aDataBuf[], uint32_t u32Len);
IICPrint(void);
ï¿½ï¿½ï¿½ß£ï¿½yifei.zhang
*********************************************************************/
#ifndef __IICDRIVER_H__
#define __IICDRIVER_H__
#include "Device.h"
#define IIC_PORT_NUM 4

//#define IIC_BASE_ADDR ADDR_BASE
#define IIC_PORT_ADDR_LEN 0x1000
#define IIC_CTRL_REG_OFFSET 0
#define IIC_STAT_REG_OFFSET 0x04
#define IIC_DATA_REG_OFFSET 0x08
#define IIC_ADDR0_REG_OFFSET 0x0c
#define IIC_SMB_REG_OFFSET 0x10
#define IIC_ADDR1_REG_OFFSET 0x1c
// #define IIC_STAT_REG_OFFSET 0x08
// #define IIC_DATA_REG_OFFSET 0x10
// #define IIC_ADDR0_REG_OFFSET 0x18
// #define IIC_SMB_REG_OFFSET 0x20
// #define IIC_ADDR1_REG_OFFSET 0x38
#define IIC_CTRL_MASK_CR0 0x01  // div 0
#define IIC_CTRL_MASK_CR1 0x02  // div 1
#define IIC_CTRL_MASK_AA 0x04   // assert acknow
#define IIC_CTRL_MASK_SI 0x08   // serial interrupt
#define IIC_CTRL_MASK_STO 0x10  // stop
#define IIC_CTRL_MASK_STA 0x20  // start
#define IIC_CTRL_MASK_ENS1 0x40 // enable
#define IIC_CTRL_MASK_CR2 0x80  // div2
#define IIC_STAT_START 0x08
#define IIC_STAT_REPEAT 0x10
#define IIC_STAT_MSTOP 0xE0
#define IIC_STAT_MTCMD_ACK 0x18
#define IIC_STAT_MTCMD_NACK 0x20
#define IIC_STAT_MTDATA_ACK 0x28
#define IIC_STAT_MTDATA_NACK 0x30
#define IIC_STAT_MARBIT_LOST 0x38
#define IIC_STAT_MRCMD_ACK 0x40
#define IIC_STAT_MRCMD_NACK 0x48
#define IIC_STAT_MRDATA_ACK 0x50
#define IIC_STAT_MRDATA_NACK 0x58
#define IIC_STAT_SRCMD_ACK 0x60
#define IIC_STAT_SRCMD_NACK 0x68
#define IIC_STAT_SBCMD_ACK 0x70
#define IIC_STAT_SARBIT_LOST 0x78
#define IIC_STAT_SRDATA_ACK 0x80
#define IIC_STAT_SRDATA_NACK 0x88
#define IIC_STAT_SBDATA_ACK 0x90
#define IIC_STAT_SBDATA_NACK 0x98
#define IIC_STAT_SSTASTO 0xA0
#define IIC_STAT_STCMD_ACK 0xA8
#define IIC_STAT_STCMD_NACK 0xB0
#define IIC_STAT_STDATA_ACK 0xB8
#define IIC_STAT_STDATA_NACK 0xC0
#define IIC_STAT_IDLE 0xF8
#define IIC_TBUFF_SIZE 0x1000
#define IIC_RBUFF_SIZE 0x1000
#define IIC_MASTER 0x1
#define IIC_SLAVE 0x2
#define IIC_ENABLE_VALUE 0x1A
#define IIC_PORT0 0
#define IIC_PORT1 1
#define IIC_PORT2 2
#define IIC_PORT3 3
#define IIC_DIV_256 0 // 50M/256
#define IIC_DIV_224 1 // 50M/224
#define IIC_DIV_192 2 // 50M/192
#define IIC_DIV_160 3 // 50M/160
#define IIC_DIV_960 4 // 50M/960
#define IIC_DIV_120 5 // 50M/120
#define IIC_DIV_60 6  // 50M/60
#define IIC_DIV_8 7   // 800K/8
typedef struct IIC_CB
{
    uint32_t Port;
    uint32_t MorS;
    uint32_t Enable;
    uint32_t CAddr;
    uint32_t SAddr;
    uint32_t DAddr;
    uint32_t AAddr;
    uint8_t Ctrl;
    //SEM_ID sem;
} IIC_CB;
typedef struct IIC_CB_S
{
    uint8_t TBuf[IIC_TBUFF_SIZE];
    uint8_t RBuf[IIC_RBUFF_SIZE];
    uint32_t TBufWrite;
    uint32_t TBufRead;
    uint32_t RBufWrite;
    uint32_t RBufRead;
} IIC_CB_S;
/********************************************************************
ï¿½ï¿½ï¿½Æ£ï¿½IICï¿½ï¿½Ê¼ï¿½ï¿½
ï¿½ï¿½ï¿½Ü£ï¿½ï¿½ï¿½Î»IICï¿½Ë¿Ú£ï¿½ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½Ä´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½FIFOï¿½ï¿½Ä¬ï¿½Ï²ï¿½ï¿½ï¿½ï¿½ï¿½100kbps
ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½u32Portï¿½Ë¿Úºï¿½0-2
ï¿½ï¿½ï¿½Ø£ï¿½1ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½É¹ï¿½ï¿½ï¿½0ï¿½ï¿½Ê¼ï¿½ï¿½Ê§ï¿½ï¿½
*********************************************************************/
extern uint32_t IICInit(uint32_t u32Port);
/********************************************************************
ï¿½ï¿½ï¿½Æ£ï¿½IICï¿½ï¿½ï¿½ï¿½
ï¿½ï¿½ï¿½Ü£ï¿½ï¿½ï¿½ï¿½ï¿½IICï¿½ï¿½ï¿½ï¿½ï¿½Ê£ï¿½ï¿½ï¿½ï¿½ï¿½IICInit(uint32_t u32Port)ï¿½ï¿½ï¿½ï¿½ï¿½Ê»á±»ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÎªIIC_DIV_8
ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½u32Portï¿½Ë¿Úºï¿½0-2ï¿½ï¿½ u8Divï¿½ï¿½Æµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ä¬ï¿½ï¿½ÎªIIC_DIV_8ï¿½ï¿½ï¿½ï¿½100Kï¿½ï¿½ï¿½ï¿½
IIC_DIV_256 0   //50M/256
IIC_DIV_224 1   //50M/224
IIC_DIV_192 2   //50M/192
IIC_DIV_160 3   //50M/160
IIC_DIV_960 4   //50M/960
IIC_DIV_120 5   //50M/120
IIC_DIV_60 6   //50M/60
IIC_DIV_8 7   //800K/8
ï¿½ï¿½ï¿½Ø£ï¿½1ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½É¹ï¿½ï¿½ï¿½0ï¿½ï¿½Ê¼ï¿½ï¿½Ê§ï¿½ï¿½
*********************************************************************/
extern uint32_t IICConfig(uint32_t u32Port, uint8_t u8Div);
/********************************************************************
ï¿½ï¿½ï¿½Æ£ï¿½IICï¿½ï¿½Ä£Ê½ï¿½ï¿½ï¿½ï¿½
ï¿½ï¿½ï¿½Ü£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ù¶ï¿½Ã¿2ï¿½ï¿½ï¿½ï¿½1ï¿½ï¿½ï¿½Ö½Ú£ï¿½ï¿½ï¿½ï¿½ï¿½Ú¼ä²»Õ¼ï¿½ï¿½CPU
ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½u32Portï¿½Ë¿Úºï¿½0-2ï¿½ï¿½u8AddrÄ¿ï¿½ï¿½ï¿½Ö·ï¿½ï¿½u8aDataBufï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ýµï¿½Ö·ï¿½ï¿½u32Lenï¿½ï¿½ï¿½Ý³ï¿½ï¿½ï¿½
ï¿½ï¿½ï¿½Ø£ï¿½Êµï¿½Ê³É¹ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö½Ú¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ô¤ï¿½Ú²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¹ï¿½ï¿½IICInitï¿½ï¿½Î»ï¿½Ã¶Ë¿ï¿½
*********************************************************************/
extern uint32_t IICMasterSend(uint32_t u32Port, uint8_t u8Addr, uint8_t u8aDataBuf[], uint32_t u32Len);

/********************************************************************
ï¿½ï¿½ï¿½Æ£ï¿½IICï¿½ï¿½Ä£Ê½ï¿½ï¿½ï¿½ï¿½
ï¿½ï¿½ï¿½Ü£ï¿½ï¿½ï¿½È¡ï¿½Ù¶ï¿½Ã¿2ï¿½ï¿½ï¿½ï¿½1ï¿½ï¿½ï¿½Ö½Ú£ï¿½ï¿½ï¿½ï¿½ï¿½Ú¼ä²»Õ¼ï¿½ï¿½CPU
ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½u32Portï¿½Ë¿Úºï¿½0-2ï¿½ï¿½u8AddrÄ¿ï¿½ï¿½ï¿½Ö·ï¿½ï¿½u8aDataBufï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ýµï¿½Ö·ï¿½ï¿½u32Lenï¿½ï¿½ï¿½Ý³ï¿½ï¿½ï¿½
ï¿½ï¿½ï¿½Ø£ï¿½Êµï¿½Ê³É¹ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö½Ú¸ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ô¤ï¿½Ú²ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê¹ï¿½ï¿½IICInitï¿½ï¿½Î»ï¿½Ã¶Ë¿ï¿½
*********************************************************************/
extern uint32_t IICMasterReceive(uint32_t u32Port, uint8_t u8Addr, uint8_t u8aDataBuf[], uint32_t u32Len);
extern uint32_t IICMasterSendNoStop(uint32_t u32Port, uint8_t u8Addr, uint8_t u8aDataBuf[], uint32_t u32Len);
/********************************************************************
ï¿½ï¿½ï¿½Æ£ï¿½ï¿½ï¿½Ó¡IIC×´Ì¬
ï¿½ï¿½ï¿½Ü£ï¿½ï¿½ï¿½Ó¡ï¿½Ñ³ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½IICï¿½Ë¿Ú¼Ä´ï¿½ï¿½ï¿½×´Ì¬
ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
ï¿½ï¿½ï¿½Ø£ï¿½ï¿½Ñ³ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½ï¿½Ä¶Ë¿ï¿½ï¿½ï¿½Ä¿
*********************************************************************/
extern int IICSemTake(uint32_t u32Port);
extern int IICSemGive(uint32_t u32Port);
extern uint32_t IICPrint(void);

//add
void reset_i2c(int port);
void test111(uint32_t u32Port);
#endif
