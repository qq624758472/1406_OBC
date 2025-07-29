#include <stdlib.h>
#include "IICDriver.h"
#include "UTFunction.h"
#include <time.h>
#include "../inc/drv_common.h"
#include "../op_common/op_common.h"
static IIC_CB IICCB[IIC_PORT_NUM] = {{0}};
void IICCBCreat(uint32_t u32Port);
uint32_t IICInit(uint32_t u32Port);
uint32_t IICConfig(uint32_t u32Port, uint8_t u8Div);
uint32_t IICMasterSend(uint32_t u32Port, uint8_t u8Addr, uint8_t u8aDataBuf[], uint32_t u32Len);
uint32_t IICMasterSendNoStop(uint32_t u32Port, uint8_t u8Addr, uint8_t u8aDataBuf[], uint32_t u32Len);
uint32_t IICMasterReceive(uint32_t u32Port, uint8_t u8Addr, uint8_t u8aDataBuf[], uint32_t u32Len);
int IICSemTake(uint32_t u32Port);
int IICSemGive(uint32_t u32Port);
uint32_t IICWait(uint32_t u32Port);
uint32_t IICPrint(void);
void taskIICSlave(uint32_t u32Port, uint8_t u8Addr, IIC_CB_S *IICCBS);

void *IIC_BASE_ADDR = NULL;
// void *HPI_ADDR_BASE = NULL;
int mramHdlIICDriver = -1;
void IICCBCreat(uint32_t u32Port)
{
    if (IIC_BASE_ADDR != NULL)
    {
        devm_unmap(IIC_BASE_ADDR, 4 * 1024, &mramHdlIICDriver);
        mramHdlIICDriver = -1;
        IIC_BASE_ADDR = NULL;
    }
    switch (u32Port)
    {
    case 0:
        IIC_BASE_ADDR = devm_map(ADDR_BASE, 4 * 1024, &mramHdlIICDriver); // 32M
        break;
    case 1:
        IIC_BASE_ADDR = devm_map(ADDR_BASE1, 4 * 1024, &mramHdlIICDriver); // 32M
        break;
    case 2:
        IIC_BASE_ADDR = devm_map(ADDR_BASE2, 4 * 1024, &mramHdlIICDriver); // 32M
        break;
    case 3:
        IIC_BASE_ADDR = devm_map(ADDR_BASE3, 4 * 1024, &mramHdlIICDriver); // 32M
        break;
    default:
        break;
    }
    if (IIC_BASE_ADDR == NULL)
    {
        LOG("devm_map ERROR!");
        exit(1);
    }

    // taskLock();
    if (IICCB[u32Port].Enable != IIC_ENABLE_VALUE)
    {
        // // IICCB[u32Port].sem = semMCreate(EM_Q_PRIORITY);
        // IICCB[u32Port].Port = u32Port;
        // IICCB[u32Port].CAddr = IIC_BASE_ADDR + u32Port * IIC_PORT_ADDR_LEN + IIC_CTRL_REG_OFFSET;
        // IICCB[u32Port].SAddr = IIC_BASE_ADDR + u32Port * IIC_PORT_ADDR_LEN + IIC_STAT_REG_OFFSET;
        // IICCB[u32Port].DAddr = IIC_BASE_ADDR + u32Port * IIC_PORT_ADDR_LEN + IIC_DATA_REG_OFFSET;
        // IICCB[u32Port].AAddr = IIC_BASE_ADDR + u32Port * IIC_PORT_ADDR_LEN + IIC_ADDR0_REG_OFFSET;
        // IICCB[u32Port].Enable = IIC_ENABLE_VALUE;

        // IICCB[u32Port].sem = semMCreate(EM_Q_PRIORITY);
        IICCB[u32Port].Port = u32Port;
        IICCB[u32Port].CAddr = IIC_BASE_ADDR + IIC_CTRL_REG_OFFSET;
        IICCB[u32Port].SAddr = IIC_BASE_ADDR + IIC_STAT_REG_OFFSET;
        IICCB[u32Port].DAddr = IIC_BASE_ADDR + IIC_DATA_REG_OFFSET;
        IICCB[u32Port].AAddr = IIC_BASE_ADDR + IIC_ADDR0_REG_OFFSET;
        IICCB[u32Port].Enable = IIC_ENABLE_VALUE;
    }
    // taskUnlock();
}
uint32_t IICInit(uint32_t u32Port)
{
    if (u32Port >= IIC_PORT_NUM)
    {
        LOG("PortMax = %d", IIC_PORT_NUM - 1);
        return 0;
    }
    LOG();
    IICCBCreat(u32Port);
    IICConfig(u32Port, IIC_DIV_8);
    IO_WRITE16(DRO_FPGA_RESET0_RGE_ADDR, DRO_IIC0_MASK << u32Port);
    IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl);
    // taskDelay(2);
    usleep(20);
    if (IO_READ8(IICCB[u32Port].CAddr) == IICCB[u32Port].Ctrl)
    {
        // semGive(IICCB[u32Port].sem);
        return 1;
    }
    else
    {
        // semGive(IICCB[u32Port].sem);
        return 0;
    }
}
uint32_t IICConfig(uint32_t u32Port, uint8_t u8Div)
{
    if (u32Port >= IIC_PORT_NUM)
    {
        LOG("PortMax = %d", IIC_PORT_NUM - 1);
        return 0;
    }
    if (IICCB[u32Port].Enable != IIC_ENABLE_VALUE)
    {
        IICInit(u32Port);
    }
    // semTake(IICCB[u32Port].sem, 2000);
    IICCB[u32Port].Ctrl = (u8Div & 0x03) | ((u8Div & 0x04) << 5) | IIC_CTRL_MASK_ENS1;
    // semGive(IICCB[u32Port].sem);
    return 1;
}

void test111(uint32_t u32Port)
{
    IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STO);
}

void reset_i2c(int port)
{
    switch (port)
    {
    case 0:
        /* code */
        enable(24, 0);
        usleep(5);
        enable(24, 1);
        break;

    case 1:
        enable(25, 0);
        usleep(5);
        enable(25, 1);
        /* code */
        break;
    case 2:
        enable(26, 0);
        usleep(5);
        enable(26, 1);
        /* code */
        break;
    case 3:
        enable(27, 0);
        usleep(5);
        enable(27, 1);
        /* code */
        break;
    default:
        break;
    }
}

uint32_t IICMasterSend(uint32_t u32Port, uint8_t u8Addr, uint8_t u8aDataBuf[], uint32_t u32Len)
{
    uint32_t i = 0;
    if (u32Port >= IIC_PORT_NUM)
    {
        LOG("PortMax = %d", IIC_PORT_NUM - 1);
        return 0;
    }
    if (IICCB[u32Port].Enable != IIC_ENABLE_VALUE)
    {
        IICInit(u32Port);
    }
    // semTake(IICCB[u32Port].sem, 2000);
    IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STA);
    IICWait(u32Port);
    // IICBDelay(10);
    IO_WRITE8(IICCB[u32Port].DAddr, u8Addr << 1);
    IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl);

    // IICBDelay(10);
    IICWait(u32Port);
    if (IO_READ8(IICCB[u32Port].SAddr) != IIC_STAT_MTCMD_ACK)
    {
        reset_i2c(u32Port);
        usleep(10);
        reset_i2c(u32Port);
        LOG("%02X !=IIC_STAT_MTCMD_ACK\n", IO_READ8(IICCB[u32Port].SAddr));
        IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STO);
        // semGive(IICCB[u32Port].sem);
        IICInit(u32Port);
        return 0;
    }
    for (i = 0; i < u32Len; i++)
    {
        IO_WRITE8(IICCB[u32Port].DAddr, u8aDataBuf[i]);
        IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl);
        IICWait(u32Port);
        // IICBDelay(10);
        if (IO_READ8(IICCB[u32Port].SAddr) != IIC_STAT_MTDATA_ACK)
        {
            LOG("%02X !=IIC_STAT_MTDATA_ACK\n", IO_READ8(IICCB[u32Port].SAddr));
            IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STO);
            // semGive(IICCB[u32Port].sem);
            IICInit(u32Port);
            return i;
        }
    }
    IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STO);
    IICWait(u32Port);
    // semGive(IICCB[u32Port].sem);
    return u32Len;
}
uint32_t IICMasterSend_NOACK(uint32_t u32Port, uint8_t u8Addr, uint8_t u8aDataBuf[], uint32_t u32Len)
{
    uint32_t i = 0;
    if (u32Port >= IIC_PORT_NUM)
    {
        LOG("PortMax = %d", IIC_PORT_NUM - 1);
        return 0;
    }
    if (IICCB[u32Port].Enable != IIC_ENABLE_VALUE)
    {
        IICInit(u32Port);
    }
    // semTake(IICCB[u32Port].sem, 2000);
    IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STA);
    IICWait(u32Port);
    // IICBDelay(10);
    IO_WRITE8(IICCB[u32Port].DAddr, u8Addr << 1);
    IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl);
    // IICBDelay(10);
    IICWait(u32Port);
    if (IO_READ8(IICCB[u32Port].SAddr) != IIC_STAT_MTCMD_ACK)
    {
        LOG("%02X !=IIC_STAT_MTCMD_ACK\n", IO_READ8(IICCB[u32Port].SAddr));
        // IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STO);
        // semGive(IICCB[u32Port].sem);
        // IICInit(u32Port);
        // return 0;
    }
    for (i = 0; i < u32Len; i++)
    {
        IO_WRITE8(IICCB[u32Port].DAddr, u8aDataBuf[i]);
        IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl);
        IICWait(u32Port);
        // IICBDelay(10);
        if (IO_READ8(IICCB[u32Port].SAddr) != IIC_STAT_MTDATA_ACK)
        {
            // LOG("%02X !=IIC_STAT_MTDATA_ACK\n", IO_READ8(IICCB[u32Port].SAddr));
            // IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STO);
            // semGive(IICCB[u32Port].sem);
            // IICInit(u32Port);
            // return i;
        }
    }
    IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STO);
    IICWait(u32Port);
    // semGive(IICCB[u32Port].sem);
    return u32Len;
}
uint32_t IICMasterSendNoStop(uint32_t u32Port, uint8_t u8Addr, uint8_t u8aDataBuf[], uint32_t u32Len)
{
    uint32_t i = 0;
    if (u32Port >= IIC_PORT_NUM)
    {
        LOG("PortMax = %d", IIC_PORT_NUM - 1);
        return 0;
    }
    if (IICCB[u32Port].Enable != IIC_ENABLE_VALUE)
    {
        IICInit(u32Port);
    }
    // semTake(IICCB[u32Port].sem, 2000);
    IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STA);
    IICWait(u32Port);
    // IICBDelay(10);
    IO_WRITE8(IICCB[u32Port].DAddr, u8Addr << 1);
    IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl);
    // IICBDelay(10);
    IICWait(u32Port);
    if (IO_READ8(IICCB[u32Port].SAddr) != IIC_STAT_MTCMD_ACK)
    {
        LOG("%02X !=IIC_STAT_MTCMD_ACK\n", IO_READ8(IICCB[u32Port].SAddr));
        IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STO);
        // semGive(IICCB[u32Port].sem);
        IICInit(u32Port);
        return 0;
    }
    for (i = 0; i < u32Len; i++)
    {
        IO_WRITE8(IICCB[u32Port].DAddr, u8aDataBuf[i]);
        IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl);
        IICWait(u32Port);
        // IICBDelay(10);
        if (IO_READ8(IICCB[u32Port].SAddr) != IIC_STAT_MTDATA_ACK)
        {
            LOG("%02X !=IIC_STAT_MTDATA_ACK\n", IO_READ8(IICCB[u32Port].SAddr));
            IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STO);
            // semGive(IICCB[u32Port].sem);
            IICInit(u32Port);
            return i;
        }
    }
    // IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STO);
    IICWait(u32Port);
    // semGive(IICCB[u32Port].sem);
    return u32Len;
}
uint32_t IICMasterReceive(uint32_t u32Port, uint8_t u8Addr, uint8_t u8aDataBuf[], uint32_t u32Len)
{
    int ret = 0;
    uint32_t Wait = 0;
    uint32_t i = 0;
    if (u32Port >= IIC_PORT_NUM)
    {
        LOG("PortMax = %d", IIC_PORT_NUM - 1);
        return 0;
    }
    if (IICCB[u32Port].Enable != IIC_ENABLE_VALUE)
    {
        IICInit(u32Port);
    }
    // LOG("");
    //  semTake(IICCB[u32Port].sem, 2000);
    IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STA);
    IICWait(u32Port);
    IO_WRITE8(IICCB[u32Port].DAddr, (u8Addr << 1) | 0x01);
    IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl);
    IICWait(u32Port);
    if (IO_READ8(IICCB[u32Port].SAddr) != IIC_STAT_MRCMD_ACK)
    {
        LOG("0x%02X !=IIC_STAT_MRCMD_ACK\n", IO_READ8(IICCB[u32Port].SAddr));
        IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STO);
        // semGive(IICCB[u32Port].sem);
        IICInit(u32Port);
        return ret;
    }
    // LOG("");
    // clock_t start, end;
    // double cpu_time_used;
    // start = clock(); // ??????
    for (i = 0; i < u32Len - 1; i++)
    {
        IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_AA);
        IICWait(u32Port);
        if (IO_READ8(IICCB[u32Port].SAddr) != IIC_STAT_MRDATA_ACK)
        {
            LOG("%02X !=IIC_STAT_MRDATA_ACK\n", IO_READ8(IICCB[u32Port].SAddr));
            IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STO);
            // semGive(IICCB[u32Port].sem);
            IICInit(u32Port);
            return ret;
        }
        u8aDataBuf[i] = IO_READ8(IICCB[u32Port].DAddr);
    }
    // end = clock();
    // cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC; // ??????
    // LOG("recv1 %f s\n", cpu_time_used);
    // start = clock(); // ??????
    // LOG("");
    IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl);
    IICWait(u32Port);
    if (IO_READ8(IICCB[u32Port].SAddr) != IIC_STAT_MRDATA_NACK)
    {
        LOG("%02X !=IIC_STAT_MRDATA_NACK\n", IO_READ8(IICCB[u32Port].SAddr));
        IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STO);
        // semGive(IICCB[u32Port].sem);
        IICInit(u32Port);
        return ret;
    }
    // LOG("");
    u8aDataBuf[i] = IO_READ8(IICCB[u32Port].DAddr);
    IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_STO);
    IICWait(u32Port);
    // semGive(IICCB[u32Port].sem);
    // end = clock();
    // cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC; // ??????
    // LOG("recv2 %f s\n", cpu_time_used);
    return u32Len;
}
int IICSemTake(uint32_t u32Port)
{
    // return semTake(IICCB[u32Port].sem, 2000);
}
int IICSemGive(uint32_t u32Port)
{
    // return semGive(IICCB[u32Port].sem);
}
uint32_t IICWait(uint32_t u32Port)
{
#if 1
    uint32_t u32TimeOut = 10000; // DATA��NACK 40-42 START��STOP 1-2
    while (((IO_READ8(IICCB[u32Port].CAddr) & IIC_CTRL_MASK_SI) == 0) && (u32TimeOut > 0))
    {
        u32TimeOut--;
    }
    // LOG("u32TimeOut:%d\n", u32TimeOut);
    return u32TimeOut;
#else
    // taskDelay(2);
    usleep(8);
    return 1;
#endif
}
/*??????????????????*/
#if (DRIVER_TEST_FLAG == 1)
uint32_t IICPrint(void)
{
    uint32_t u32EnablePortNum = 0;
    uint32_t u32Port = 0;
    for (u32Port = 0; u32Port < IIC_PORT_NUM; u32Port++)
    {
        if (IICCB[u32Port].Enable == IIC_ENABLE_VALUE)
        {
            LOG("IIC%d Cmd=%02X\n", u32Port, IO_READ8(IICCB[u32Port].CAddr));
            LOG("IIC%d State=%02X\n", u32Port, IO_READ8(IICCB[u32Port].SAddr));
            LOG("IIC%d Data=%02X\n", u32Port, IO_READ8(IICCB[u32Port].DAddr));
            LOG("IIC%d Addr=%02X\n", u32Port, IO_READ8(IICCB[u32Port].AAddr));
            u32EnablePortNum++;
        }
    }
    return u32EnablePortNum;
}
void taskIICSlave(uint32_t u32Port, uint8_t u8Addr, IIC_CB_S *IICCBS)
{
    uint8_t u8State = 0;
    if (u32Port >= IIC_PORT_NUM)
    {
        LOG("PortMax = %d", IIC_PORT_NUM - 1);
        // taskDelete(0);
    }
    if (IICCB[u32Port].Enable != IIC_ENABLE_VALUE)
    {
        IICInit(u32Port);
    }
    IO_WRITE8(IICCB[u32Port].AAddr, u8Addr << 1);
    IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_AA);

    while (1)
    {
        u8State = IO_READ8(IICCB[u32Port].SAddr);
        switch (u8State)
        {
        case IIC_STAT_SRCMD_ACK:
        {
            // LOG("Port%d IIC_STAT_SRCMD_ACK\n", u32Port);
            IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_AA);
            break;
        }
        case IIC_STAT_SRDATA_ACK:
        {
            IICCBS->RBuf[IICCBS->RBufWrite] = IO_READ8(IICCB[u32Port].DAddr);
            IICCBS->RBufWrite = (IICCBS->RBufWrite + 1) % IIC_RBUFF_SIZE;
            // LOG("Port%d D:%02X\n", u32Port, IO_READ8(IICCB[u32Port].DAddr));
            IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_AA);
            break;
        }
        case IIC_STAT_STCMD_ACK:
        case IIC_STAT_STCMD_NACK:
        case IIC_STAT_STDATA_ACK:
        {
            // IO_WRITE8(IICCB[u32Port].DAddr, u8Data++);
            IO_WRITE8(IICCB[u32Port].DAddr, IICCBS->TBuf[IICCBS->TBufRead]);
            IICCBS->TBufRead = (IICCBS->TBufRead + 1) % IIC_TBUFF_SIZE;
            // LOG("Port%d State:%X\n", u32Port, u8State);
            IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_AA);
            break;
        }
        case IIC_STAT_STDATA_NACK:
        {
            // LOG("Port%d State:%X\n", u32Port, u8State);
            IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_AA);
            break;
        }
        case IIC_STAT_IDLE:
        {
            // taskDelay(1);
            usleep(10);
            break;
        }
        case IIC_STAT_SSTASTO:
        {
            // LOG("Port%d IIC_STAT_SSTASTO\n", u32Port);
            IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl | IIC_CTRL_MASK_AA);
            break;
        }
        default:
        {
            // LOG("Port%d State:%X\n", u32Port, u8State);
            IO_WRITE8(IICCB[u32Port].CAddr, IICCB[u32Port].Ctrl);
            // taskDelay(500);
            usleep(5000);
            break;
        }
        }
    }
}
void IICUserTest(uint32_t u32Port, uint8_t u8Addr, uint8_t u8Data)
{
    uint8_t u8aDataBuf[10];
    u8aDataBuf[0] = u8Addr;
    u8aDataBuf[1] = u8Data;
    IICMasterSend(u32Port, 0x50, u8aDataBuf, 2);
    // taskDelay(10);
    usleep(20);
    u8aDataBuf[0] = 0;
    IICMasterSend(u32Port, 0x50, u8aDataBuf, 1);
    IICMasterReceive(u32Port, 0x50, u8aDataBuf, 10);
    UTMemPrint(u8aDataBuf, 10);
}
/*IIC?????u32MPort???u32SPort???????????????????????????????????????????*/
void IICTest(uint32_t u32MPort, uint32_t u32SPort, uint32_t u32Len)
{
#if 0
    static uint8_t u8Addr = 0x1;
    uint8_t u8aDataBuf[IIC_TBUFF_SIZE];
    IIC_CB_S IICBCS;
    uint32_t u32Ret = 0;
    static uint8_t u8DataStart = 0;
    int taskID;
    IICBCS.TBufWrite = 0;
    IICBCS.TBufRead = 0;
    IICBCS.RBufWrite = 0;
    IICBCS.RBufRead = 0;
    if (u32Len > IIC_TBUFF_SIZE)
    {
        u32Len = IIC_TBUFF_SIZE;
    }
    if (u32Len > IIC_RBUFF_SIZE)
    {
        u32Len = IIC_RBUFF_SIZE;
    }
    taskID = taskSpawn("taskIICSlave", 250, 0, 0x400, (FUNCPTR)taskIICSlave, u32SPort, u8Addr, &IICBCS, 0, 0, 0, 0, 0, 0, 0);

    UTMakeData(u8aDataBuf, u32Len, u8DataStart++, 1);
    IICMasterSend(u32MPort, u8Addr, u8aDataBuf, u32Len);
    //taskDelay(10);
    usleep(20);
    u32Ret = UTDataCompare(u8aDataBuf, IICBCS.RBuf, u32Len);
    if (u32Ret > 0)
    {
        u32Ret--;
        LOG("IIC%d MTLoop Err! S[%d]=%02X, R[%d]=%02X\n", u32MPort, u32Ret, u8aDataBuf[u32Ret], u32Ret, IICBCS.RBuf[u32Ret]);
        //taskDelete(taskID);
    }
    UTMakeData(IICBCS.TBuf, u32Len, u8DataStart++, 1);
    IICMasterReceive(u32MPort, u8Addr, u8aDataBuf, u32Len);
    //taskDelay(10);
    usleep(20);
    u32Ret = UTDataCompare(u8aDataBuf, IICBCS.TBuf, u32Len);
    if (u32Ret > 0)
    {
        u32Ret--;
        LOG("IIC%d MRLoop Err! S[%d]=%02X, R[%d]=%02X\n", u32MPort, u32Ret, IICBCS.TBuf[u32Ret], u32Ret, u8aDataBuf[u32Ret]);
        //taskDelete(taskID);
    }
    //taskDelete(taskID);
    u8Addr = (u8Addr % 0x7F) + 1;
#endif
}
void IICBDelay(uint32_t u32Delay)
{
    u32Delay = u32Delay << 10;
    while (u32Delay--)
        ;
    return;
}
#endif
#if 0
IO_WRITE8_T 0xBE014000,0xC3
IO_WRITE8_T 0xBE014018,0x03
IO_WRITE8_T 0xBE014000,0xC7
IO_WRITE8_T 0xBE012000,0xC3
IO_WRITE8_T 0xBE012000,0xE3
IO_WRITE8_T 0xBE012010,0x02
IO_WRITE8_T 0xBE012000,0xC3
IO_READ8_T 0xBE012008 
IO_WRITE8_T 0xBE014000,0xC7
IO_WRITE8_T 0xBE012010,0x12
IO_WRITE8_T 0xBE012000,0xC3
IO_READ8_T 0xBE014010
IO_WRITE16_T 0BE01A018,0x0100
#endif
