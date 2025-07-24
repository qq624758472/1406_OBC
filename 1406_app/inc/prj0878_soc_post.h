/******************************************************************************
* COPYRIGHT Beijing UCAS Space Technology Co.,Ltd
*******************************************************************************

*******************************************************************************
* 文件名称: prj0878_soc_post.h
* 功能描述: 开机自检程序(Power-on Self Test Program)头文件
* 使用说明:
* 文件作者:
* 编写日期: 2022/1/22 
* 修改历史:
* 修改版本  修改日期     修改人        修改内容
* -----------------------------------------------------------------------------
* 01a      2022/1/22    zhanghao    创建基础版本
*******************************************************************************/
/******************************** 头文件保护开头 ******************************/

#ifndef _PRJ0878_SOC_POST_H_
#define _PRJ0878_SOC_POST_H_

#include <linux/types.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/time.h>

#define STR_MAX_LEN			128

typedef struct st_opt
{
	char select[STR_MAX_LEN];
	char fval[STR_MAX_LEN];
	char sval[STR_MAX_LEN];
	char tval[STR_MAX_LEN];
	char endrval[STR_MAX_LEN];
} st_opt_t;

typedef struct
{
	int id;
	int sockid;

	struct can_frame RxFm;
	struct can_frame TxFm;
} can_mng;

#define RESETR_BASE_ADDR 0x43c00000
#define RESETR_SIZE       0x1000
#define VER_DATE_ADDR  (0x43c00000-RESETR_BASE_ADDR)
#define VER_TIME_ADDR  (0x43c00004-RESETR_BASE_ADDR)
#define VER_WDTCNT_ADDR  (0x43c00038-RESETR_BASE_ADDR)
#define VER_SYSCNT_ADDR  (0x43c0003C-RESETR_BASE_ADDR)
// MRAM地址和容量
#define MRAM_BASE_ADDR  0x60000000    /* MRAM基地址 */
#define MRAM_SIZE       0x200000      /* MRAM共2M字节 */


#define CAN_ID_Z7_BD_ADDR		0x41A 	//z7板can id字段地址
#define CAN_ID_OCA_BD_ADDR		0x42A	//OCA板can id字段地址
#define CAN_ID_OCB_BD_ADDR		0x42B	//OCA板can id字段地址
#define CAN_MSG_TYPE_SW_EN		0x1a	//使能或禁止主备切换
#define CAN_MSG_TYPE_CTRL		0x2a	//OC板通道控制
#define CAN_MSG_TYPE_SMP_REQ	0x30  	// 采集请求，低4bit用于选择哪一组数据，共5组电压和1组杂项
#define CAN_MSG_TYPE_SMP_RSP	0x40  	// 采集相应

typedef struct
{
	unsigned char MsgType;
	unsigned char FrameId;
	unsigned char Data[6];
}can_msg;

// 能源板下位机与z7板CAN协议通信定义
typedef struct
{
	unsigned int Rsv:21;
	unsigned int BusPos:1;   //总线占用标记
	unsigned int FramType:2; //帧类型
	unsigned int NodAddr:5;  //节点地址
	unsigned int BusId:1;    //A/B总线
	unsigned int SglOrMul:1; //单复帧信息
}stCanId;

//PCM同步串口
#define PCM_MEM_BASE			0x83eb0000	// PCM PS-PL交互区基地址
#define PCM_MEM_LEN				0x10000 	// PCM PS-PL交互区长度

#define PCM_PORT_NUM 1  //PCM目前只有一路
#define PCM_BASE_ADDR PCM_MEM_BASE
#define PCM_PORT_ADDR_LEN 0x1000//每个端口占用地址空间
#define PCM_CTRL_REG 0//控制寄存器偏移
#define PCM_TDATA_REG 4 //数据发送寄存器偏移，寄存器16位，只有低8位有效
#define PCM_STAT_REG 8//状态寄存器偏移
#define PCM_TBUFF_USE_COUNT_REG 12//发送FIFO使用字节数，0-1024
#define PCM_RDES_BLOCK_POINT_REG 16//描述符写指针，每个描述符4个字，8个字节，这里的指针是以16bit为单位的
#define PCM_RDES_BLOCK_RAM_ADDR 0x400//描述符队列起始地址
#define PCM_RBUFF_RAM_ADDR 0x1000//接收数据RAM起始地址偏移，低8位有效
#define PCM_TBUFF_LEN 1024//发送数据FIFO长度
#define PCM_RDES_BLOCK_RAM_LEN 256//描述符队列长度
#define PCM_RBUFF_RAM_LEN 2048//接收数据RAM地址空间，接收数据RAM大小1024B，低8位有效，占用地址空间2048
#define PCM_CTRL_T_START_BIT 0x0001//控制寄存器，发送开始
#define PCM_CTRL_T_RESET_BIT 0x0002//控制寄存器发送模块复位，写1复位
#define PCM_CTRL_R_RESET_BIT 0x0004//控制寄存器接收模块复位，写1复位
#define PCM_STAT_BUSY_BIT 0x0001//无效
#define PCM_DES_HEAD1_WORD 0//描述符第一个字，目前固定0x5AA5
#define PCM_DES_HEAD2_WORD 1//描述符第二个字，目前固定0xABCD
#define PCM_DES_COR_LEN_WORD 2//描述符第三个字，错误标志和帧长度
#define PCM_DES_COR_BIT 0x8000//描述符第三个字，最高位为错误标识，1有错误
#define PCM_DES_LEN_BIT 0x01FF//描述符第三个字，低9位为帧有效数据长度，0-256
#define PCM_DES_DATA_ADDR_WORD 3//描述符第四个字，数据帧在接收数据RAM的起始地址
#define PCM_BAG_SIZE 128
#define PCM_RECEIVE_BUFF_SIZE 0x2000
#define PCM_ENABLE_VALUE 0x1A

typedef struct PCM_CB
{
    uint32_t Port;			//端口号
    uint32_t Enable;		//控制块是否初始化过
    uint32_t InitOK;		//端口是否初始化完成

	/***************** PCM寄存器begin ***************/
	// 控制
    uint32_t CTAddr;	//控制寄存器地址:     0x000
    // 发送相关
    uint32_t TAddr;		//发送数据寄存器地址: 0x001<<1, [bit7:0]: 为发送的数据
    uint32_t STAddr;	//发送状态寄存器地址: 0x002<<1, [bit1]=1：发送出现帧错误; 
                        //                                    =0：发送正常。
						//                              [bit0]=1：busy
						//                                    =0：free
						//注意: 对该寄存器写访问可对该bit清零
    uint32_t TUCAddr;	//发送区占用字节数寄存器地址: 0x003<<1, 总大小1KB
	// 接收相关
    uint32_t DesPAddr;	//描述符指针地址
    uint32_t DesAddr;	//描述符RAM地址
    uint32_t RAddr;		//接收数据RAM地址
    /***************** PCM寄存器end ***************/

    uint16_t DesPRec;//前一次软件服务完成时描述符指针，其实就是描述符读指针
    uint8_t RBuf[PCM_RECEIVE_BUFF_SIZE][PCM_BAG_SIZE];//接收软件缓冲 
    uint32_t RBufWrite;//接收软件缓冲写指针
    uint32_t RBufRead;//接收软件缓冲读指针
    uint32_t RBufLock;//
    uint32_t initCnt; //调试用
    //SEM_ID semT;//发送互斥信号量
    //SEM_ID semR;//接收互斥信号量
    //SEM_ID semIntT;//中断信号量 无用
    pthread_t rxThrd;
} PCM_CB;//PCM控制块

/* ----------------------- 软件重构功能 ---------------------- */
// 重构管理信息文件路径
#define UPDATE_INFO_FILE		"/run/media/mmcblk0p1/update_info.dat"

// update_info结构体 UpgradeSt 成员定义
#define UPDATE_NONE				0
#define UPDATE_FSHA_OK			1
#define UPDATE_FSHB_OK			2
#define UPDATE_FSHAB_OK			3

//#define UPGRD_NONE				0		// 未升级
//#define UPGRD_FIN					1		// 升级完成
//#define UPGRD_FAIL				2		// 升级失败

// 巡检/重构功能工作状态机，update_ctrl结构体 StMach 成员定义
#define STATE_MACH_NONE			0
#define STATE_MACH_DET			1
#define STATE_MACH_RECOV		2
#define STATE_MACH_UPDATE		3

// 巡检flash模式
#define FLSH_DET_MD_WK		0		// 巡检当前启动的flash
#define FLSH_DET_MD_NWK		1		// 巡检未启动的flash

#define FLASH_A   0
#define FLASH_B   1

// eMMC三个重构区版本路径
#define VERLIBX_PATH			"/run/media/mmcblk0p"
#define VERLIB1_PATH			"/run/media/mmcblk0p1"
#define VERLIB2_PATH			"/run/media/mmcblk0p2"
#define VERLIB3_PATH			"/run/media/mmcblk0p3"
#define VERLIB_CURA				"/aur/"
#define VERLIB_CURB				"/bur/"
#define VERLIB_NEW				"/new/"
#define DEFAULT                 "/default/"

// 版本文件名称和类型编号
#define MAX_VER_TYPE			20
#define VER_NAME_BOOT			"BOOT.BIN"
#define VER_NAME_ENV			"bootenv"
#define VER_NAME_KERN			"image.ub"
#define VER_NAME_APP			"app-test"
#define VER_TYPE_NONE			0
#define VER_TYPE_BOOT			1
#define VER_TYPE_ENV			2
#define VER_TYPE_KERN			3
#define VER_TYPE_APP			4

// ModuTable 定义
#define VER_NAME_MODU1			"modu1"
#define VER_NAME_MODU2			"modu2"
#define VER_NAME_MODU3			"modu3"
#define VER_NAME_MODU4			"modu4"
#define VER_NAME_MODU5			"modu5"
#define VER_NAME_MODU6			"modu6"
#define VER_NAME_MODU7			"modu7"
#define VER_NAME_MODU8			"modu8"
#define VER_NAME_MODU9			"modu9"
#define VER_NAME_MODU10			"modu10"
#define VER_NAME_MODU11			"modu11"
#define VER_NAME_MODU12			"modu12"
#define VER_NAME_MODU13			"modu13"
#define VER_NAME_MODU14			"modu14"
#define VER_NAME_MODU15			"modu15"
#define VER_TYPE_MODU1			5
#define VER_TYPE_MODU2			6
#define VER_TYPE_MODU3			7
#define VER_TYPE_MODU4			8
#define VER_TYPE_MODU5			9
#define VER_TYPE_MODU6			10
#define VER_TYPE_MODU7			11
#define VER_TYPE_MODU8			12
#define VER_TYPE_MODU9			13
#define VER_TYPE_MODU10			14
#define VER_TYPE_MODU11			15
#define VER_TYPE_MODU12			16
#define VER_TYPE_MODU13			17
#define VER_TYPE_MODU14			18
#define VER_TYPE_MODU15			19


// 巡检时间间隔默认值，分钟
#define DET_TIME_INTV_M			1  //15  // 测试用1分钟，上星用15分钟
#define DET_MAX_CNT				2
#define UPDATE_SLEEP_US			500000  // 巡检/重构等待时间

// md5sum码长度，32字节内容+1字节'\0'
#define MD5SUM_MAX_SIZE			33
#define MD5SUM_FILE_PATH		"/home/root/"

// 星务app可执行程序运行目录
#define APP_EXEC_PATH			"/home/root/"
#define APP_HEAD_UP_FILE		"/home/root/app-head-up"
#define APP_HEAD_read_FILE		"/home/root/app-head-read"
#define APP_HEAD_LD_FILE		"/home/root/app-head-load"
#define FSH_ERASESIZE			0x80000 	// flash可擦除块最小单位,512K

// 各版本区修复统计计数，占用20字节
typedef union 
{
	uint8_t cnt[MAX_VER_TYPE];	// 数组表达
	struct st_cnt				// 结构体表达
	{
		uint8_t BootCnt;		// BOOT.BIN修复统计
		uint8_t EnvCnt;			// bootenv修复统计
		uint8_t KernCnt;		// image.ub修复统计
		uint8_t AppCnt;			// 星务app修复统计
		uint8_t ModuCnt[MAX_VER_TYPE-4];	// 星务module修复统计
	}stCnt; 					// eMMC重构分区1~3
}repair_cnt;
	
/* 
* 重构管理信息，用于历史信息存储，以二进制文件的形式存储于EMMC
* 存储路径: /run/media/mmcblk0p1/updata.dat
* 存储端序: Host Endian(Little Endian)
*/
typedef struct
{
	//offset 0
	uint8_t HisStartFsh;	// 历史启动FLASH区，bit0为最近一次启动flash区，bit1次之，
							// 以此类推，最多可记录8次启动情况
	uint8_t UpgradeSt;		// 当前升级状态，参见定义 UPDATE_NONE 等
	uint8_t ALoadCnt;		// flash A加载次数统计
	uint8_t BLoadCnt;		// flash B加载次数统计

	//offset 4
	uint8_t Fs1RecovCnt;	// eMMC重构分区1文件系统修复统计
	uint8_t Fs2RecovCnt;	// eMMC重构分区2文件系统修复统计
	uint8_t Fs3RecovCnt;	// eMMC重构分区3文件系统修复统计
	uint8_t LenRecovCnt;	// mtd3 app len字段修复统计


	uint8_t BootFlash;		// 当前启动的flash区，0:flash A,  1:flash B
	uint8_t DealFlashShow;		//设置完操作当前flash或者非当前启动flash，后显示此状态 
	uint8_t rcv_yuliu;
    /*OC control state*/
    union 
   {
        uint8_t Stateshow;
        struct
        {
            uint8_t DetSTATE :2;       //巡检状态  0:表示空闲状态  1:表示开始 2表示巡检结束
            uint8_t UpdateSTATE :2;    // NON表示空闲；1:开始  2 :升级成功 3 :升级失败
			uint8_t RepairSTATE:2; //0 :空闲  1 :开始   2:结束  3 :ERR
			uint8_t NewLoad:2;
        }st;
    }State_info;	

	//offset 8
	uint32_t UpgradeMap;	// 升级版本类型位图，第几个bit对应VER_TYPE_XXXX    升级结果
							// 用于支持一次更新多种版本

	//offset 12
	uint32_t VlibDamMap;	// EMMC 3个版本库中某种类型的版本均坏掉的位图标记，     <巡检结果>
							// 第几个bit对应VER_TYPE_XXXX
	uint32_t DetErrMap;		// 巡检有错误的版本类型位图
	//offset 16，emmc各类版本修复次数
	repair_cnt unEmmcRecov[3];	// 0:mmcblk0p1, 1:mmcblk0p2, 2:mmcblk0p3
	
	//offset 16+60=76，flash巡检修复各分区统计次数
	repair_cnt unFshRecov[2];	// 0:flash A,  1:flash B

	//offset 16+100=116，flash升级各分区统计次数
	repair_cnt unFshUpdate[2];	// 0:flash A,  1:flash B
	
} update_info; // 总计156 bytes

// 巡检或重构的版本信息，VerName为不含路径的文件名
typedef struct
{
	uint8_t VerType;
	char	VerName[31];
}ver_info;

// 重构控制字段，作为全局变量使用，重启后恢复为初始值
// 注意：此控制结构体既有控制信息，也有实时状态检测信息
typedef struct
{
	int     FileId;			// 重构管理信息文件ID
	char    verMDfive[MAX_VER_TYPE];
	uint8_t StMach;			// 工作状态机，初始化为空状态。
							// 0:空状态； 1:巡检状态； 2:修复状态； 3:重构状态
							// 状态机转换：
							// (1) 0->1: 升级状态完整，且星务下达巡检参数
							// (2) 1->0: 巡检未发现错误，且巡检次数达到最大值，自动切换
							// (3) 1->2: 巡检发现版本错误后自动切换到修复状态等待
							// (4) 2->1: 星务下达忽略或修复指令，(若要求修复，则修复完成后)自动切换
							// (5) 0->3: 新版本存在，且星务下达升级指令
							// (6) 3->0: 升级完成自动切换
							//
	uint8_t CurDetVerType;	// 当前正在巡检的版本类型，由重构模块自己更新
	uint8_t Rsv;			// 4字节对齐
	
	uint8_t CtrlDetMode;	// 控制巡检模式，默认巡检的是当前启动的FLASH，
							// 0: 巡检当前启动的FLASH
							// 1: 巡检的是非启动的FLASH
	uint8_t RepairEn;		// 控制是否修复巡检发现错误的flash分区
	uint8_t RsvArr[2];		// 保留4字节对齐
	
	uint32_t CtrlUpgrdMap;	// 当前要求升级的版本类型位图，第几个bit对应VER_TYPE_XXXX
							// 用于支持一次更新多种版本
	
	
	ver_info stVer[MAX_VER_TYPE];	// 4+16种类型的版本，作为查询表使用，初始化后不再变化
	time_t   DetIntv;		// 巡检时间间隔，初始化为15分钟
	uint32_t DetCnt;		// 本次开机已经巡检的次数，初始化为0
	uint32_t MaxDetCnt;		// 最大巡检次数，初始化为0，根据星务配置再执行巡检
	
}update_ctrl;

// 星务应用程序文件长度存放在mtd3分区中前64个字节，
// app版本从64字节偏移的地方开始
#define APP_MODU_REGION		(1024*1024)
typedef struct
{
	uint32_t fileLen;		// 星务应用程序二进制文件长度
	char rsv[12];			// 保留字段，12字节
	char name[48];			// 文件名
}app_head;

#define MTD_RD_TMP_FILE		"/home/root/mtd-rd-tmp-file"  // 测试用
#define QSPI_Ctrler_ADDR	0xe000d000
#define QSPI_Ctrler_MLEN	64				// 32bit寄存器，主要的寄存器都在前16个以内
#define MEMRESUME           0xc8			// 在内核中自定义宏值，用于复位qspi控制器
#define MEMRESUME_ZHH       0xc9

#if 0
// 在/include/uapi/mtd/mtd-abi.h
struct erase_info_user {
	__u32 start;
	__u32 length;
};

// 在/include/uapi/mtd/mtd-abi.h
struct mtd_info_user {
	__u8 type;
	__u32 flags;
	__u32 size;	/* Total size of the MTD */
	__u32 erasesize;
	__u32 writesize;
	__u32 oobsize;	/* Amount of OOB data per block (e.g. 16) */
	__u64 padding;	/* Old obsolete field; do not use */
};
#endif

#endif  /* _PRJ0878_SOC_POST_H_ */
/******************************头文件结束*************************************/
