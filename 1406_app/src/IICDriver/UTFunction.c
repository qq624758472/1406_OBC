#include "UTFunction.h"
#include "../op_common/op_common.h"

void UTMemPrint(uint8_t u8aAddr[], uint32_t u32Len);
void UTMakeData(uint8_t u8aDataBuf[], uint32_t u32Size, uint8_t u8Start, uint8_t u8Increment);
uint32_t UTDataCompare(uint8_t u8aDataBufA[], uint8_t u8aDataBufB[], uint32_t u32Len);
void UTMakeData32(uint32_t u32aDataBuf[], uint32_t u32Size, uint32_t u32Start, uint32_t u32Increment);
uint32_t UTDataCompare32(uint32_t u32aDataBufA[], uint32_t u32aDataBufB[], uint32_t u32Len);
void UTMemPrint(uint8_t u8aAddr[], uint32_t u32Len)
{
    uint32_t i = 0;
    for (i = 0; i < u32Len; i++)
    {
        LOG("D[%02X]=%02X ", i, u8aAddr[i]);
        if (i % 8 == 7)
        {
            LOG("\n");
        }
    }
    if (i % 8 != 0)
    {
        LOG("\n");
    }
}
void UTMakeData(uint8_t u8aDataBuf[], uint32_t u32Size, uint8_t u8Start, uint8_t u8Increment)
{
    uint32_t i = 0;
    for (i = 0; i < u32Size; i++)
    {
        u8aDataBuf[i] = u8Start + u8Increment * i;
    }
    return;
}
uint32_t UTDataCompare(uint8_t u8aDataBufA[], uint8_t u8aDataBufB[], uint32_t u32Len)
{
    uint32_t i = 0;
    for (i = 0; i < u32Len; i++)
    {
        if (u8aDataBufA[i] != u8aDataBufB[i])
        {
            return i + 1;
        }
    }
    return 0;
}
void UTMakeData32(uint32_t u32aDataBuf[], uint32_t u32Size, uint32_t u32Start, uint32_t u32Increment)
{
    uint32_t i = 0;
    for (i = 0; i < u32Size; i++)
    {
        u32aDataBuf[i] = u32Start + u32Increment * i;
    }
    return;
}
uint32_t UTDataCompare32(uint32_t u32aDataBufA[], uint32_t u32aDataBufB[], uint32_t u32Len)
{
    uint32_t i = 0;
    for (i = 0; i < u32Len; i++)
    {
        if (u32aDataBufA[i] != u32aDataBufB[i])
        {
            return i + 1;
        }
    }
    return 0;
}
uint16_t IO_WRITE16_T(uint32_t Addr, uint16_t Data)
{
    return IO_WRITE16(Addr, Data);
}
uint16_t IO_READ16_T(uint32_t Addr)
{
    return IO_READ16(Addr);
}
uint8_t IO_WRITE8_T(uint32_t Addr, uint8_t Data)
{
    return IO_WRITE8(Addr, Data);
}
uint8_t IO_READ8_T(uint32_t Addr)
{
    return IO_READ8(Addr);
}
