

#ifndef _DRV_COMMON_H_
#define _DRV_COMMON_H_


// //LOG  control
// #define LOG_ENABLE  
// #ifdef LOG_ENABLE
//     #define LOG(fmt, ...) LOG("[%s:%d] " fmt "\n",__FUNCTION__, __LINE__, ##__VA_ARGS__)  
// #else
//     #define LOG(fmt, ...)
// #endif

// #define LOG_ERROR_ENABLE  
// #ifdef LOG_ERROR_ENABLE
//     #define LOGERR(fmt, ...) LOG("[%s:%d] " fmt "\n",__FUNCTION__, __LINE__, ##__VA_ARGS__)  
// #else
//     #define LOGERR(fmt, ...)
// #endif

#ifndef YES
#define YES		1
#endif
#ifndef NO
#define NO 		0
#endif

#ifndef OK
#define OK		0
#endif
#ifndef ERROR
#define ERROR	(-1)
#endif
#ifndef ERR
#define ERR (-1)
#endif

typedef unsigned char	uint8_t;
typedef unsigned short	uint16_t;
typedef unsigned int	uint32_t;

#include <semaphore.h> 
#define TCA9548_CHIP    "/dev/i2c-0"
#define TCA9548_ADDRESS     0x70
#define I2C_M_WT    0x0000 /* write data, from master to slave */
#define I2C_M_RD    0x0001 /* read data, from slave to master */
#define I2C_RETRIES         0x0701   
#define I2C_TIMEOUT         0x0702  
#define I2C_SLAVE           0x0703 
#define I2C_SLAVE_FORCE     0x0706  
#define I2C_TENBIT          0x0704  
#define I2C_FUNCS           0x0705  
#define I2C_RDWR            0x0707  
#define I2C_PEC             0x0708   /* != 0 to use PEC with SMBus            */ 
#define I2C_SMBUS           0x0720  /*SMBus transfer                         */

//GPIO
#define GPIO_DIR_OUT		1   
#define GPIO_DIR_IN			0   
#define PL_GPO_GNSS_BASE	1014 
#define PL_GPO_BASE			1016

#define PL_GPO_TTL			(PL_GPO_BASE+16) 
#define PL_GPO_WDI			(PL_GPO_BASE+0)
#define  PL_GPO_RTC                     (PL_GPO_BASE+3)


#define GPIO_LED_TEST		(PL_GPO_BASE)
#define GPIO_DOG_EN			(PL_GPO_BASE+1) 
#define GPIO_DOG_FEED		(PL_GPO_BASE+0) 
#define PS_GPIO_BASE		859//876
#define PS_GPO_BASE         859//876
#define GPIOINOUT_BASE_ADDR 0x43cc0000

#define GPIO_BOOT_SW		(PS_GPIO_BASE+29) 
#define GPIO_BOOT_STATE		(PS_GPIO_BASE+35) 




#define MSGLEN  4*1024           
#define MAXMSGLEN  (MSGLEN + 1)  
//#define MAXQSIZE  500           
#define MAXWAINT  50            

//#define AXIDMA_USE_QUEUE

typedef char MSGTYPE;
typedef struct qelem
{
    MSGTYPE msg[MAXMSGLEN];
    int len;
}QElemType;

typedef struct
{
    QElemType *base;
    int front;
    int rear;
    int queuesize;
    unsigned int queFullCnt;
    unsigned int queEmptyCnt;
    unsigned int recvCnt;
    unsigned int sendCnt;
    sem_t queue_sem;
}SqQueue;  /*???��???*/
extern void devm_unmap(void *virt_addr, int len, int *phandle);
extern void *devm_map(unsigned long addr, int len, int *phandle);
extern void xil_memcpy(void* dst, const void* src, unsigned int cnt);
extern int switch_channel_enable(unsigned char channel);
extern int InitQueue(SqQueue *Q, unsigned int maxqsize);
extern int EnQueue(SqQueue *Q, MSGTYPE *pMsg, int len);
extern int DeQueue(SqQueue *Q, MSGTYPE *pMsg, int *pLen);
extern int DestroyQueue(SqQueue *Q);
extern void ClearQueue(SqQueue *Q);
extern int mySleep(int sec, int msec);
extern void myUsleep(int usec);
extern void print_data(char *data, int len);
extern void print_data_16bit(unsigned short *data, int len);
extern int system_cmd(const char *command);
#endif  /* _DRV_COMMON_H_ */
/******************************????????*************************************/
