/******************************************************************************
* COPYRIGHT Beijing UCAS Space Technology Co.,Ltd
*******************************************************************************

*******************************************************************************
* 文件名称: i2c_access.h
* 功能描述: i2c接口访问头文件
* 使用说明:
* 文件作者:
* 编写日期: 2022/1/24
* 修改历史:
* 修改版本  修改日期     修改人        修改内容
* -----------------------------------------------------------------------------
* 01a      2022/1/24    zhanghao     创建基础版本
*******************************************************************************/
/******************************** 头文件保护开头 ******************************/
#ifndef __I2C_ACCESS_H__ /* 头文件保护 */
#define __I2C_ACCESS_H__ /* 头文件保护 */

#define I2C_0_CHIP "/dev/i2c-0"
#define I2C_1_CHIP "/dev/i2c-1"
#define I2C_2_CHIP "/dev/i2c-2"
#define I2C_3_CHIP "/dev/i2c-3"
#define I2C_4_CHIP "/dev/i2c-4"
#define I2C_5_CHIP "/dev/i2c-5"
#define I2C_6_CHIP "/dev/i2c-6"

// 3种器件I2C地址
#define I2C_PCA9548_ADDR 0x70 // i2c通道切换芯片 	PCA9548
#define I2C_LTC2992_ADDR 0x6A // 功率监控芯片 		LTC2992
#define I2C_TCA6424_ADDR 0x22 // I2C转GPIO扩展芯片	TCA6424

// i2c通道切换芯片PCA9548切换通道                       i2c-0 or i2c-1
#define PCA9548_CH_DIS 0x00 // close ALL switch
#define PCA9548_CH0_EN 0x01 // Switch to LTC2992(U1   or U6)
#define PCA9548_CH1_EN 0x02 // Switch to LTC2992(U2   or U7)
#define PCA9548_CH2_EN 0x04 // Switch to LTC2992(U3   or U8)
#define PCA9548_CH3_EN 0x08 // Switch to LTC2992(U4   or U9)
#define PCA9548_CH4_EN 0x10 // Switch to LTC2992(U5   or U10)
#define PCA9548_CH5_EN 0x20 // Switch to TCA6424(U190 or U191)
#define PCA9548_CH6_EN 0x40
#define PCA9548_CH7_EN 0x80

// LTC2992寄存器地址
#define LTC2992_REG_CTRLA 0x00 // 配置
#define LTC2992_REG_CTRLB 0x01 // 配置
#define LTC2992_REG_NADC 0x04  // 配置AD采样的位数，8bit或12bit，默认12bit
#define LTC2992_RET_ADCST 0x32 // ADC采集状态
#define LTC2992_REG_I1 0x14    // sensor1 current(2bytes)，大端序
#define LTC2992_REG_I2 0x46    // sensor2 current(2bytes)
#define LTC2992_REG_G1 0x28    // GPIO1 voltage(2bytes)
#define LTC2992_REG_G2 0x5A    // GPIO2 voltage(2bytes)
#define LTC2992_REG_G3 0x64    // GPIO3 voltage(2bytes)
#define LTC2992_REG_G4 0x6E    // GPIO4 voltage(2bytes)

// TCA6424 12个寄存器地址
#define TCA6462_REG_INP0 0x00
#define TCA6462_REG_INP1 0x01
#define TCA6462_REG_INP2 0x02
#define TCA6462_REG_OUTP0 0x04
#define TCA6462_REG_OUTP1 0x05
#define TCA6462_REG_OUTP2 0x06
#define TCA6462_REG_INVP0 0x08
#define TCA6462_REG_INVP1 0x09
#define TCA6462_REG_INVP2 0x0A
#define TCA6462_REG_CFGP0 0x0C
#define TCA6462_REG_CFGP1 0x0D
#define TCA6462_REG_CFGP2 0x0E

extern int i2c_open(unsigned char id);
extern void i2c_close(void);
extern void i2c_fd_close(int fd);

extern int i2c_Write(unsigned short slave_addr, unsigned char reg, unsigned char value);
extern int i2c_Read(unsigned short slave_addr, unsigned char reg, int buf_len, unsigned char *outbuf);
extern int i2c_wt_pca9548(unsigned short slave_addr, unsigned char value);
extern int i2c_rd_pca9548(unsigned short slave_addr, unsigned char *outbuf);
extern int i2cIPMB_Write(unsigned short slave_addr, unsigned char len, unsigned char *dat);
extern int i2cIPMB_Read(unsigned short slave_addr, unsigned char len, unsigned char *outbuf);
extern int i2c_eeprom_Write(unsigned short slave_addr, unsigned char addr, unsigned char value);
extern int i2c_eeprom_Read(unsigned short slave_addr, unsigned char reg, int buf_len, unsigned char *outbuf);

#if 0
/*ipmb内部工参*/
typedef struct
{
	  /*IPMB数字量工参*/
	  union
	  {
		  u8 ver;
		  struct
		  {
			  u8 reserved6		       : 1;  /*bin0 预留*/
			  u8 rotatorchmccode_ver   : 7;  /*bit1-7 转子1CHMC软件版本号和心跳*/
		  }st;
	  }__attribute__((__packed__))rotatorchmccode; /**/

	  u8 rotatorchmcrevcodeturecnt; 		  /*CHMC接收主控正确指令计数*/
	  u8 rotatorchmcrevcodeerrcnt;			  /*CHMC接收主控错误指令计数*/
	  union
	  {
		  u8 num;
		  struct
		  {
			  u8 rotatorchmcipmbcomerrcnt  :4;		/*bit0-3  CHMC与IPMB通信异常计数*/
			  u8 rotatorchmczkcomerrcnt    :4;		/*bit4-7  CHMC与主控通信异常计数*/
		  }st;
	  }__attribute__((__packed__))rotatorchmccomerrcnt ;/*定子通信异常计数*/
	   union
	  {
		  u8 num;
		  struct
		  {
			  u8 rotatorchmcwindogrstcnt  :4;	   /*bit0-3 CHMC看门狗复位计数*/
			  u8 rotatoripmbalarmcnt      :4;		  /*bit4-7	IPMB一级报警次数*/
		  }st;
	  }__attribute__((__packed__))rotatoripmb_chmc_alarmcnt; /*1209 */
	   union
	  {
		  u8 state;
		  struct
		  {
			  u8 rotatoripmbreserved  : 5;		/*bit0-4 ipmb通信状态预留*/
			  u8 rotatoripmc2comstate : 1;		/*bit5-5 IPMC2通信状态 0正常,1异常*/
			  u8 rotatoripmc1comstate : 1;		/*bit6-6 IPMC1通信状态 0正常,1异常*/
			  u8 rotatorchmccomstate  : 1;		/*bit7-7 CHMC通信状态*/
		  }st;
	  }__attribute__((__packed__))rotatoripmb_chmc_comstate ;/*1210 */
	   union
	  {
		  u8 state;
		  struct
		  {
			  u8 rotatoripmcpowerreserved  : 5; 	 /*bit0-5IPMC加断电状态预留*/
			  u8 rotatoripmc2powerstate    : 1; 	 /*bit6-6 IPMC2加断电状态 0正常,1异常*/
			  u8 rotatoripmc1powerstate    : 1; 	 /*bit6-6 IPMC1加断电状态 0正常,1异常*/
			  u8 rotatorchmcpowerstate	   : 1; 	 /*bit7-7 CHMC加断电状态*/
		  }st;
	  }__attribute__((__packed__))rotatoripmb_chmc_powerstate; /*1211 */
	  u8 rotatoripmb1nergc [16];  /*IPMB1 AD0-8工程参数*/
	  u8 rotatoripmb2nergc [16];  /*IPMB2 AD0-8工程参数*/

}__attribute__((__packed__))Ipmbgcdef;

#else

#define IpmbDigLen 14 /*数字量帧总长度 6+7+1*/
#define N 1
#define IpmbGcLen (6 + 16 * N + 1) /*工参帧总长度 6+16*NUM+1*/

typedef struct
{
    unsigned short FrameHead;
    unsigned char FrameType;
    unsigned char DevIdentifier;
    unsigned short PacketNumber;
    unsigned int TimeCode;
    unsigned short DataLen;
    unsigned char DigitalVar[5];
    unsigned char GC_Var[16 * 16];
    unsigned char SumByte;
} IpmbGCdef;

#endif

#endif /* __I2C_ACCESS_H__ */
/******************************头文件结束*************************************/
