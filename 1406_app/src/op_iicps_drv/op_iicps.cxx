/*
 * @Description:
 * @Version: 2.0
 * @Autor: ruog__
 * @Date: 2025-04-01 13:47:03
 * @LastEditors: ruog__
 * @LastEditTime: 2025-04-01 15:43:06
 */
#include "op_iicps.h"
#include "xil_types.h"
#include "xil_io.h"
#include "../op_common/op_common.h"
// #include "xiicps.h"
/*
 * The slave address to send to and receive from.
 */
#define IIC_SLAVE_ADDR 0x01
#define IIC_SCLK_RATE 100000

/*
 * The following constant controls the length of the buffers to be sent
 * and received with the IIC.
 */
#define TEST_BUFFER_SIZE 250
#define NUMBER_OF_SIZES 7
XIicPs Iic; /* Instance of the IIC Device */

static OptionsMap OptionsTable[] = {
    {XIICPS_7_BIT_ADDR_OPTION, XIICPS_CR_NEA_MASK},
    {XIICPS_10_BIT_ADDR_OPTION, XIICPS_CR_NEA_MASK},
    {XIICPS_SLAVE_MON_OPTION, XIICPS_CR_SLVMON_MASK},
    {XIICPS_REP_START_OPTION, XIICPS_CR_HOLD_MASK},
};

#define XIICPS_NUM_OPTIONS (sizeof(OptionsTable) / sizeof(OptionsMap))

void XIicPs_Reset(XIicPs *InstancePtr)
{

    // Xil_AssertVoid(InstancePtr != NULL);
    // Xil_AssertVoid(InstancePtr->IsReady == (u32)XIL_COMPONENT_IS_READY);

    // /*
    //  * Abort any transfer that is in progress.
    //  */
    // XIicPs_Abort(InstancePtr);

    /*
     * Reset any values so the software state matches the hardware device.
     */
    XIicPs_WriteReg(InstancePtr->Config.BaseAddress, XIICPS_CR_OFFSET,
                    XIICPS_CR_RESET_VALUE);
    XIicPs_WriteReg(InstancePtr->Config.BaseAddress,
                    XIICPS_TIME_OUT_OFFSET, XIICPS_TO_RESET_VALUE);
    XIicPs_WriteReg(InstancePtr->Config.BaseAddress, XIICPS_IDR_OFFSET,
                    XIICPS_IXR_ALL_INTR_MASK);
}

u32 XIicPs_GetOptions(XIicPs *InstancePtr)
{
    u32 OptionsFlag = 0U;
    u32 ControlReg;
    u32 Index;

    /*
     * Read control register to find which options are currently set.
     */
    ControlReg = XIicPs_ReadReg(InstancePtr->Config.BaseAddress,
                                XIICPS_CR_OFFSET);

    /*
     * Loop through the options table to determine which options are set.
     */
    for (Index = 0U; Index < XIICPS_NUM_OPTIONS; Index++)
    {
        if ((ControlReg & OptionsTable[Index].Mask) != (u32)0x0U)
        {
            OptionsFlag |= OptionsTable[Index].Option;
        }
        if ((ControlReg & XIICPS_CR_NEA_MASK) == (u32)0x0U)
        {
            OptionsFlag |= XIICPS_10_BIT_ADDR_OPTION;
        }
    }

    if (InstancePtr->IsRepeatedStart != 0)
    {
        OptionsFlag |= XIICPS_REP_START_OPTION;
    }
    return OptionsFlag;
}

s32 XIicPs_CfgInitialize(XIicPs *InstancePtr, XIicPs_Config *ConfigPtr, u32 EffectiveAddr)
{
    /*
     * Assert validates the input arguments.
     */
    assert(InstancePtr != NULL);
    assert(ConfigPtr != NULL);

    /*
     * Set some default values.
     */
    InstancePtr->Config.DeviceId = ConfigPtr->DeviceId;
    InstancePtr->Config.BaseAddress = EffectiveAddr;
    InstancePtr->Config.InputClockHz = ConfigPtr->InputClockHz;
    // InstancePtr->StatusHandler = StubHandler;
    InstancePtr->CallBackRef = NULL;

    InstancePtr->IsReady = (u32)XIL_COMPONENT_IS_READY;

    /*
     * Reset the IIC device to get it into its initial state. It is expected
     * that device configuration will take place after this initialization
     * is done, but before the device is started.
     */
    XIicPs_Reset(InstancePtr);

    /*
     * Keep a copy of what options this instance has.
     */
    InstancePtr->Options = XIicPs_GetOptions(InstancePtr);

    /* Initialize repeated start flag to 0 */
    InstancePtr->IsRepeatedStart = 0;

    return (s32)XST_SUCCESS;
}

s32 XIicPs_SelfTest(XIicPs *InstancePtr)
{

    // Xil_AssertNonvoid(InstancePtr != NULL);
    // Xil_AssertNonvoid(InstancePtr->IsReady == (u32)XIL_COMPONENT_IS_READY);

    /*
     * All the IIC registers should be in their default state right now.
     */
    if ((XIICPS_CR_RESET_VALUE !=
         XIicPs_ReadReg(InstancePtr->Config.BaseAddress,
                        XIICPS_CR_OFFSET)) ||
        (XIICPS_IXR_ALL_INTR_MASK !=
         XIicPs_ReadReg(InstancePtr->Config.BaseAddress,
                        XIICPS_IMR_OFFSET)))
    {
        return (s32)XST_FAILURE;
    }

    XIicPs_Reset(InstancePtr);

    /*
     * Write, Read then write a register
     */
    XIicPs_WriteReg(InstancePtr->Config.BaseAddress,
                    XIICPS_SLV_PAUSE_OFFSET, REG_TEST_VALUE);

    if (REG_TEST_VALUE != XIicPs_ReadReg(InstancePtr->Config.BaseAddress,
                                         XIICPS_SLV_PAUSE_OFFSET))
    {
        return (s32)XST_FAILURE;
    }

    XIicPs_WriteReg(InstancePtr->Config.BaseAddress,
                    XIICPS_SLV_PAUSE_OFFSET, 0U);

    XIicPs_Reset(InstancePtr);

    return (s32)XST_SUCCESS;
}

s32 XIicPs_SetSClk(XIicPs *InstancePtr, u32 FsclHz)
{
    u32 Div_a;
    u32 Div_b;
    u32 ActualFscl;
    u32 Temp;
    u32 TempLimit;
    u32 LastError;
    u32 BestError;
    u32 CurrentError;
    u32 ControlReg;
    u32 CalcDivA;
    u32 CalcDivB;
    u32 BestDivA;
    u32 BestDivB;
    u32 FsclHzVar = FsclHz;

    // Xil_AssertNonvoid(InstancePtr != NULL);
    // Xil_AssertNonvoid(InstancePtr->IsReady == (u32)XIL_COMPONENT_IS_READY);
    // Xil_AssertNonvoid(FsclHzVar > 0U);

    if (0U != XIicPs_In32((InstancePtr->Config.BaseAddress) + XIICPS_TRANS_SIZE_OFFSET))
    {
        return (s32)XST_DEVICE_IS_STARTED;
    }

    /*
     * Assume Div_a is 0 and calculate (divisor_a+1) x (divisor_b+1).
     */
    Temp = (InstancePtr->Config.InputClockHz) / ((u32)22U * FsclHzVar);

    /*
     * If the answer is negative or 0, the Fscl input is out of range.
     */
    if ((u32)(0U) == Temp)
    {
        return (s32)XST_FAILURE;
    }

    /*
     * If frequency 400KHz is selected, 384.6KHz should be set.
     * If frequency 100KHz is selected, 90KHz should be set.
     * This is due to a hardware limitation.
     */
    if (FsclHzVar > (u32)384600U)
    {
        FsclHzVar = (u32)384600U;
    }

    if ((FsclHzVar <= (u32)100000U) && (FsclHzVar > (u32)90000U))
    {
        FsclHzVar = (u32)90000U;
    }

    /*
     * TempLimit helps in iterating over the consecutive value of Temp to
     * find the closest clock rate achievable with divisors.
     * Iterate over the next value only if fractional part is involved.
     */
    TempLimit = (((InstancePtr->Config.InputClockHz) %
                  ((u32)22 * FsclHzVar)) != (u32)0x0U)
                    ? Temp + (u32)1U
                    : Temp;
    BestError = FsclHzVar;

    BestDivA = 0U;
    BestDivB = 0U;
    for (; Temp <= TempLimit; Temp++)
    {
        LastError = FsclHzVar;
        CalcDivA = 0U;
        CalcDivB = 0U;

        for (Div_b = 0U; Div_b < 64U; Div_b++)
        {

            Div_a = Temp / (Div_b + 1U);

            if (Div_a != 0U)
            {
                Div_a = Div_a - (u32)1U;
            }
            if (Div_a > 3U)
            {
                continue;
            }
            ActualFscl = (InstancePtr->Config.InputClockHz) /
                         (22U * (Div_a + 1U) * (Div_b + 1U));

            if (ActualFscl > FsclHzVar)
            {
                CurrentError = (ActualFscl - FsclHzVar);
            }
            else
            {
                CurrentError = (FsclHzVar - ActualFscl);
            }

            if (LastError > CurrentError)
            {
                CalcDivA = Div_a;
                CalcDivB = Div_b;
                LastError = CurrentError;
            }
        }

        /*
         * Used to capture the best divisors.
         */
        if (LastError < BestError)
        {
            BestError = LastError;
            BestDivA = CalcDivA;
            BestDivB = CalcDivB;
        }
    }

    /*
     * Read the control register and mask the Divisors.
     */
    ControlReg = XIicPs_ReadReg(InstancePtr->Config.BaseAddress,
                                (u32)XIICPS_CR_OFFSET);
    ControlReg &= ~((u32)XIICPS_CR_DIV_A_MASK | (u32)XIICPS_CR_DIV_B_MASK);
    ControlReg |= (BestDivA << XIICPS_CR_DIV_A_SHIFT) |
                  (BestDivB << XIICPS_CR_DIV_B_SHIFT);

    XIicPs_WriteReg(InstancePtr->Config.BaseAddress, (u32)XIICPS_CR_OFFSET,
                    ControlReg);

    return (s32)XST_SUCCESS;
}

u8 XIicPs_send_complete(XIicPs *InstancePtr)
{
    u16 StatusReg = XIicPs_ReadReg(InstancePtr->Config.BaseAddress, XIICPS_IER_OFFSET);
    // if ((StatusReg & ))
    // {
    //     return (s32)XST_FAILURE;
    // }
}

int IicPsMasterIntrExample(u16 DeviceId)
{
    volatile u32 SendComplete;
    volatile u32 RecvComplete;
    volatile u32 TotalErrorCount;
    int Status;
    XIicPs_Config *Config;
    int Index;
    int tmp;
    int BufferSizes[NUMBER_OF_SIZES] = {0xeb, 0x90, 0x00, 0xe3, 0x00, 0x00, 0xe3};
    u8 SendBuffer[7] = {0xeb, 0x90, 0x00, 0xe3, 0x00, 0x00, 0xe3}; /* Buffer for Transmitting Data */
    u8 IpmbMaster[7] = {0xeb, 0x90, 0x01, 0xec, 0x00, 0x00, 0xec};
    u8 RecvBuffer[TEST_BUFFER_SIZE]; /* Buffer for Receiving Data */
    /*
     * Initialize the IIC driver so that it's ready to use
     * Look up the configuration in the config table, then initialize it.
     */
    Config->BaseAddress = 0xE0004000; // ps7_i2c_0
    Config->DeviceId = 0;
    Config->InputClockHz = 112500000;

    Status = XIicPs_CfgInitialize(&Iic, Config, Config->BaseAddress);
    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    /*
     * Perform a self-test to ensure that the hardware was built correctly.
     */
    Status = XIicPs_SelfTest(&Iic);
    if (Status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    /*
     * Connect the IIC to the interrupt subsystem such that interrupts can
     * occur. This function is application specific.
     */
    // Status = SetupInterruptSystem(&Iic);
    // if (Status != XST_SUCCESS)
    // {
    //     return XST_FAILURE;
    // }

    /*
     * Setup the handlers for the IIC that will be called from the
     * interrupt context when data has been sent and received, specify a
     * pointer to the IIC driver instance as the callback reference so
     * the handlers are able to access the instance data.
     */
    // XIicPs_SetStatusHandler(&Iic, (void *)&Iic, Handler);

    /*
     * Set the IIC serial clock rate.
     */
    XIicPs_SetSClk(&Iic, IIC_SCLK_RATE);

    /*
     * Initialize the send buffer bytes with a pattern to send and the
     * the receive buffer bytes to zero to allow the receive data to be
     * verified.
     */
    for (Index = 0; Index < TEST_BUFFER_SIZE; Index++)
    {
        // SendBuffer[Index] = (Index % TEST_BUFFER_SIZE);
        RecvBuffer[Index] = 0;
    }

#if 1
    // 发送设置ipmb模块为master模式
    while (XIicPs_BusIsBusy(&Iic))
    {
        /* NOP */
    }

    TotalErrorCount = 0;

    /*
     * Send the buffer, errors are reported by TotalErrorCount.
     */
    XIicPs_MasterSend(&Iic, IpmbMaster, 7, IIC_SLAVE_ADDR);

    /*
     * Wait for the entire buffer to be sent, letting the interrupt
     * processing work in the background, this function may get
     * locked up in this loop if the interrupts are not working
     * correctly.
     */
    while (!SendComplete)
    {
        // /*
        //  * All of the data transfer has been finished.
        //  */
        // if (0 != (Event & XIICPS_EVENT_COMPLETE_RECV))
        // {
        //     RecvComplete = TRUE;
        // }
        // else if (0 != (Event & XIICPS_EVENT_COMPLETE_SEND))
        // {
        //     SendComplete = TRUE;
        // }
        // else if (0 == (Event & XIICPS_EVENT_SLAVE_RDY))
        // {
        //     /*
        //      * If it is other interrupt but not slave ready interrupt, it is
        //      * an error.
        //      * Data was received with an error.
        //      */
        //     TotalErrorCount++;
        // }
    }
#endif

#if 1
    // 发送E3采集指令
    /* Wait for bus to become idle
     */
    while (XIicPs_BusIsBusy(&Iic))
    {
        /* NOP */
    }
    usleep(100);
    SendComplete = FALSE;
    TotalErrorCount = 0;
    /*
     * Send the buffer, errors are reported by TotalErrorCount.  SendBuffer
     */
    XIicPs_MasterSend(&Iic, SendBuffer, 7, IIC_SLAVE_ADDR);

    /*
     * Wait for the entire buffer to be sent, letting the interrupt
     * processing work in the background, this function may get
     * locked up in this loop if the interrupts are not working
     * correctly.
     */
    while (!SendComplete)
    {
        if (0 != TotalErrorCount)
        {
            return XST_FAILURE;
        }
    }
#endif
    /*
     * Wait bus activities to finish.
     */
    while (XIicPs_BusIsBusy(&Iic))
    {
        /* NOP */
    }

    /*
     * Receive data from slave, errors are reported through
     * TotalErrorCount.
     */
    //		memset(RecvBuffer,0,250);
    for (Index = 0; Index < TEST_BUFFER_SIZE; Index++)
    {
        RecvBuffer[Index] = 0;
    }
    RecvComplete = FALSE;
    XIicPs_MasterRecv(&Iic, RecvBuffer, 250, IIC_SLAVE_ADDR);

    while (!RecvComplete)
    {
        if (0 != TotalErrorCount)
        {
            return XST_FAILURE;
        }
    }

    /* Check for received data.
     */
    for (tmp = 0; tmp < 250; tmp++)
    {

        /*
         * Aardvark as slave can only set up to 64 bytes for
         * output.
         */
        xil_printf(" 0x%02x", RecvBuffer[tmp]);
        if (tmp % 16 == 0)
            xil_printf("\r\n");
    }
    xil_printf("\r\n");
    return XST_SUCCESS;
}
