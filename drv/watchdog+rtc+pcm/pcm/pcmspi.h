// pcmspi.h
#ifndef PCMSPI_H
#define PCMSPI_H

#include <stdint.h>


// Error Code
#define PCMSPI_EC_OK        0
#define PCMSPI_EC_DEVOPEN   -1        // failed to open device
#define PCMSPI_EC_INVALID_PARAMS -2   // Invalid parameter(s)
#define PCMSPI_EC_IOCTL     -3        // ioctl failed
#define PCMSPI_EC_CFG       -4        // config failed



// Function declarations

// 初始化函数，程序启动时运行一次，后续读写过程无需再次运行。
int pcmspi_init(void);
// 清理函数，所有读写操作完成后，程序退出前运行一次。
int pcmspi_exit(void);

// 读函数，buf为调用者申请并维护的缓冲区，需要保证长度不低于len字节；
// uiStartAddr为读起始地址；len为读字节数。注意检查地址段，不要超出PCM芯片的地址范围。
// 返回值参见PCMSPI_EC_XX错误码
int pcmspi_read(uint8_t *buf, uint32_t uiStartAddr, int len);
uint8_t pcmspi_readb(uint32_t uiAddr); //读8bit数据
uint16_t pcmspi_readw(uint32_t uiAddr); //读16bit数据
uint32_t pcmspi_readd(uint32_t uiAddr); //读32bit数据

// 写函数，buf为调用者申请并维护的缓冲区，并填充需要写入的数据，需要保证长度不低于len字节；
// uiStartAddr为写起始地址，len为写字节数，注意检查地址段，不要超出PCM芯片的地址范围。
// 返回值参见PCMSPI_EC_XX错误码
int pcmspi_write(uint8_t *buf, uint32_t uiStartAddr, int len);
int pcmspi_writeb(uint32_t uiAddr, uint8_t data); //写8bit数据
int pcmspi_writew(uint32_t uiAddr, uint16_t data);//写16bit数据
int pcmspi_writed(uint32_t uiAddr, uint32_t data);//写32bit数据


#endif // PCMSPI_H