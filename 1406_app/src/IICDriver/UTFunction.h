// deflate<DRO_Demo.out> DRO_Demo.zbin
#ifndef __UTFUNCTION_H__
#define __UTFUNCTION_H__
#include "Device.h"
#define DRIVER_TEST_FLAG 1
extern void UTMemPrint(uint8_t u8aAddr[], uint32_t u32Len);
extern void UTMakeData(uint8_t u8aDataBuf[], uint32_t u32Size, uint8_t u8Start, uint8_t u8Increment);
extern uint32_t UTDataCompare(uint8_t u8aDataBufA[], uint8_t u8aDataBufB[], uint32_t u32Len);
extern void UTMakeData32(uint32_t u32aDataBuf[], uint32_t u32Size, uint32_t u32Start, uint32_t u32Increment);
extern uint32_t UTDataCompare32(uint32_t u32aDataBufA[], uint32_t u32aDataBufB[], uint32_t u32Len);
#endif
