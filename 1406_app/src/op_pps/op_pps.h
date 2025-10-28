
#ifndef __PPS__
#define __PPS__
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


/* PPS功能操作步骤
1	地面校时
	1)	对reg1,reg2,reg5,reg6进行赋值
	2)	在不改变其他位情况下，将reg0[0]进行赋值即可完成地面校时（仅启动一次）
2	集中校时
	1)	对reg3,reg4,reg7,reg8进行赋值
	2)	在不改变其他位情况下，将reg0[4]，reg0[1]进行赋值，即规定校时方向和开启校时
3	PPS校时
	1) 对reg0[3]进行赋值即可开启外部PPS对该模块进行校时PPS校时使能可随时开关，
	2) 对reg0[6、7、8、9]赋值可选择开启PPS输出
4	均匀校时
	1)	对reg9,reg10,reg11,reg12进行赋值
	2)	对reg0[5],reg0[2]进行赋值即可完成
5	时间片中断
	1)	对reg15赋值,即给出时间片中断的时间间隔
	2)	对reg13[0]赋值
	3)	等待中断响应，并读取reg16，判断reg16[0]是否被置起
	4)	对reg14[0]赋值，即清除中断响应寄存器reg16
6	PPS中断
	1)	对reg13[1]赋值
	2)	等待中断响应，并读取reg16，判断reg16[0]是否被置起
	3)	对reg14[1]赋值，即清除中断响应寄存器reg16
7	读取本地时间
	1)	读取reg17,reg18,reg19,reg20
*/

//gpiochip1007： PL出的GPIO，外部有8路GPI， 需要访问0x41210000地址的寄存器来访问GPI，
//gpiochip1015： PL出的GPIO，外部有1路GPO， 需要访问0x41210000地址的寄存器来访问GPO，
//               访问第0bit即可，用于控制GPIO_OUT_06
//gpiochip1016： PL出的GPIO，外部有8路GPIO，访问0x41200000地址寄存器先使能相应bit，
//               然后再访问GPIO，用于LED灯、看门狗GPIO
//gpiochip889：  PS出的GPIO，直接访问GPIO，用于双boot切换
#define PL_GPI_O_EN_BASE		0x41210000	// 使能寄存器，逻辑出的GPI和GPO，GPI对应gpiochip1007，GPO对应1015
#define PL_GPI_O_LEN			0x1000 		// 交互区长度

// 基地址和偏移地址
#define PPS_RTC_EN_BASE			0x41200000	// 逻辑RTC功能模块使能寄存器基地址
#define PPS_MEM_BASE1			0x62000000
// #define PPS_MEM_BASE2			0x64000000
#define PPS_MEM_LEN				0x1000 		// PPS PS-PL交互区长度

// PPS控制寄存器各字段掩码
#define PPS_CTRL_TIM_EN_MASK 	0xfff0		// bit3~0, 校时使能控制字段掩码
#define PPS_CTRL_CENT_TIM_MASK	0xffef		// bit4,   集中校时方向
#define PPS_CTRL_AVG_TIM_MASK	0xffdf		// bit5,   均匀校时方向
// bit9~6, PPSn输出使能，bit9->pps3, ..., bit6->pps0
#define PPS_CTRL_PPS_OEN_MASK	0xfc3f		

// 4种校时方式使能, bit0~bit3
#define PPS_CTRL_ETH_TIM_EN		0x0001		// 地面校时使能   已废弃
#define PPS_CTRL_CEN_TIM_EN		0x0002		// 集中校时使能   已废弃
#define PPS_CTRL_AVG_TIM_EN		0x0004		// 均匀校时使能   已废弃
#define PPS_CTRL_PPS_TIM_EN		0x0008		// PPS 校时使能  

// 校时方向, bit4,bit5
#define PPS_CTRL_CEN_TIM_FORW	0x0010		// 集中校时正向，参与或运算
#define PPS_CTRL_CEN_TIM_REVS	(~0x0010)	// 集中校时逆向，参与与运算
#define PPS_CTRL_AVG_TIM_FORW	0x0020		// 均匀校时正向
#define PPS_CTRL_AVG_TIM_REVS	(~0x0020)	// 均匀校时逆向

// PPS输出使能, bit6~bit9
#define PPS_OUT_EN0				0x0040		// PPS0输出使能（0：禁止 1：开启）    已废弃 修改成外部PPS信号触发边沿（0：默认下降沿 1：上升沿）
#define PPS_OUT_DIS0			(~0x0040)
#define PPS_OUT_EN1				0x0080		// PPS1输出使能（0：禁止 1：开启）   已废弃  修改成切换外部PPS信号（0：默认使用PPS1 1：使用PPS2）
#define PPS_OUT_DIS1			(~0x0080)
#define PPS_OUT_EN2				0x0100		// PPS2输出使能（0：禁止 1：开启）   已废弃
#define PPS_OUT_DIS2			(~0x0100)
#define PPS_OUT_EN3				0x0200		// PPS3输出使能（0：禁止 1：开启）   已废弃
#define PPS_OUT_DIS3			(~0x0200)

// 中断相关寄存器控制
#define INT_EN_PPS				0x2			// PPS中断使能
#define INT_EN_TSLOT			0x1			// 时间片中断使能
#define INT_CLR_PPS				0x2			// PPS中断清除，回读始终为0
#define INT_CLR_TSLOT			0x1			// 时间片中断清除，回读始终为0
#define INT_RSP_PPS				0x2			// PPS中断响应，只读
#define INT_RSP_TSLOT			0x1			// 时间片中断响应，只读
#pragma (1)
typedef struct
{
	/* Ctrl_R0字段定义，复位后为0
	[15:10]	保留
	[9]		PPS3输出使能（0：禁止 1：开启）
	[8]		PPS2输出使能（0：禁止 1：开启）
	[7]		PPS1输出使能（0：禁止 1：开启）
	[6]		PPS0输出使能（0：禁止 1：开启）
	[5]		均匀校时方向
	[4]		集中校时校时方向 
			1：正向
			0：逆向
	[3]		PPS校时使能
	[2]		均匀校时使能
	[1]		集中校时使能，回读值始终为0
	[0]		地面校时使能，回读值始终为0
	*/
	unsigned short Ctrl_R0;				// 控制寄存器
	
	unsigned short SecL_R1;				// 秒计数值低16位，用于地面校时
	unsigned short SecH_R2;				// 秒计数值高16位，用于地面校时
	unsigned short SecDifL_R3;			// 秒差值计数值低16位，用于集中校时
	unsigned short SecDifH_R4;			// 秒差值计数值高16位，用于集中校时
	
	unsigned short UsL_R5;				// 微秒计数值低16位，用于地面校时
	unsigned short UsH_R6;				// 微秒计数值高16位，用于地面校时
	unsigned short UsDifL_R7;			// 微秒差值计数值低16位，用于集中校时
	unsigned short UsDifH_R8;			// 微秒差值计数值高16位，用于集中校时

	unsigned short AvgTimInt_R9;		// 均匀校时间隔寄存器，均匀校时间隔 （秒）最大65535
	unsigned short AvgTimSecDif_R10;	// 均匀校时秒计数差值，即校时一次要进行补正的秒值
	unsigned short AvgTimUsDifL_R11;	// 均匀校时微秒计数差值低16位，即校时一次要进行补正的微秒值
	unsigned short AvgTimUsDifH_R12;	// 均匀校时微秒计数差值高16位，即校时一次要进行补正的微秒值

	/* 中断使能寄存器
	[15:2]	保留
	[1]		PPS中断使能
	[0]		时间片中断使能
	*/
	unsigned short IntEn_R13;

	/* 中断清除寄存器
	[15:2]	保留
	[1]		PPS中断清除，回读值始终为0
	[0]		时间片中断清除，回读值始终为0
	*/
	unsigned short IntClr_R14;

	/* 时间片中断间隔寄存器
	[15:0]	时间片中断间隔可调，间隔为5ms*（该寄存器值+1）*/
	unsigned short IntIntv_R15;
	
	/* 中断响应寄存器
	[15:2]	保留 
	[1]		PPS中断响应位
	[0]		时间片中断响应位
	*/
	unsigned short IntRsp_R16;

	unsigned short LocSecL_R17;			// 本地时钟秒计数值低16位
	unsigned short LocSecH_R18;			// 本地时钟秒计数值高16位
	unsigned short LocUsL_R19;			// 本地时钟微秒计数值低16位
	unsigned short LocUsH_R20;			// 本地时钟微秒计数值高16位

    //独立时钟会自动计数，  如果设置的比较寄存器值和独立时钟相同时，会发送一次脉冲。
    unsigned short LocUsH_R21;          //FPGA独立时钟寄存器[15:0]
    unsigned short LocUsH_R22;          //FPGA独立时钟寄存器[31:16]
    unsigned short LocUsH_R23;          //FPGA独立时钟寄存器[47：32]
    unsigned short LocUsH_R24;          //FPGA独立时钟寄存器[63：48]
    
    unsigned short LocUsH_R25;          //龙芯配置的比较寄存器[15:0]
    unsigned short LocUsH_R26;          //龙芯配置的比较寄存器[31:16]
    unsigned short LocUsH_R27;          //龙芯配置的比较寄存器[47：32]
    unsigned short LocUsH_R28;          //龙芯配置的比较寄存器[63：48]

    unsigned short LocUsH_R29;          //连续比较时，配置的步长[15:0]
    unsigned short LocUsH_R30;          //连续比较时，配置的步长[31:16]
    unsigned short LocUsH_R31;          //连续比较时，配置的步长[47：32]
    unsigned short LocUsH_R32;          //连续比较时，配置的步长[63：48]
} pps_t;
#pragma (0)

void rtc_irq_proc();
void *rtc_irq_read_thrd(void *arg);
void pps_open();
void pps_close();
void pps_dump();
void pps_test(unsigned char type, unsigned char id, unsigned int data);
#endif