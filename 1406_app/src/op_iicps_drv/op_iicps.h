#ifndef IICPS_H
#define IICPS_H

#include "xil_types.h"
#include "xstatus.h"
#include "xil_io.h"
typedef struct
{
    u16 DeviceId;     /**< Unique ID  of device */
    u32 BaseAddress;  /**< Base address of the device */
    u32 InputClockHz; /**< Input clock frequency */
} XIicPs_Config;

typedef struct
{
    XIicPs_Config Config; /* Configuration structure */
    u32 IsReady;          /* Device is initialized and ready */
    u32 Options;          /* Options set in the device */

    u8 *SendBufferPtr; /* Pointer to send buffer */
    u8 *RecvBufferPtr; /* Pointer to recv buffer */
    s32 SendByteCount; /* Number of bytes still expected to send */
    s32 RecvByteCount; /* Number of bytes still expected to receive */
    s32 CurrByteCount; /* No. of bytes expected in current transfer */

    s32 UpdateTxSize;    /* If tx size register has to be updated */
    s32 IsSend;          /* Whether master is sending or receiving */
    s32 IsRepeatedStart; /* Indicates if user set repeated start */

    // XIicPs_IntrHandler StatusHandler;  /* Event handler function */
    void *CallBackRef; /* Callback reference for event handler */
} XIicPs;

#define XIICPS_IXR_ARB_LOST_MASK  0x00000200U	 /**< Arbitration Lost Interrupt
													mask */
#define XIICPS_IXR_RX_UNF_MASK    0x00000080U	 /**< FIFO Recieve Underflow
													Interrupt mask */
#define XIICPS_IXR_TX_OVR_MASK    0x00000040U	 /**< Transmit Overflow
													Interrupt mask */
#define XIICPS_IXR_RX_OVR_MASK    0x00000020U	 /**< Receive Overflow Interrupt
													mask */
#define XIICPS_IXR_SLV_RDY_MASK   0x00000010U	 /**< Monitored Slave Ready
													Interrupt mask */
#define XIICPS_IXR_TO_MASK        0x00000008U	 /**< Transfer Time Out
													Interrupt mask */
#define XIICPS_IXR_NACK_MASK      0x00000004U	 /**< NACK Interrupt mask */
#define XIICPS_IXR_DATA_MASK      0x00000002U	 /**< Data Interrupt mask */
#define XIICPS_IXR_COMP_MASK      0x00000001U	 /**< Transfer Complete
													Interrupt mask */
#define XIICPS_IXR_DEFAULT_MASK   0x000002FFU	 /**< Default ISR Mask */
#define XIICPS_IXR_ALL_INTR_MASK  0x000002FFU	 /**< All ISR Mask */

#define XIICPS_TIME_OUT_MASK    0x000000FFU    /**< IIC Time Out mask */
#define XIICPS_TO_RESET_VALUE   0x000000FFU    /**< IIC Time Out reset value */


#define XIICPS_CR_OFFSET			0x00U  /**< 32-bit Control */
#define XIICPS_SR_OFFSET			0x04U  /**< Status */
#define XIICPS_ADDR_OFFSET			0x08U  /**< IIC Address */
#define XIICPS_DATA_OFFSET			0x0CU  /**< IIC FIFO Data */
#define XIICPS_ISR_OFFSET			0x10U  /**< Interrupt Status */
#define XIICPS_TRANS_SIZE_OFFSET	0x14U  /**< Transfer Size */
#define XIICPS_SLV_PAUSE_OFFSET		0x18U  /**< Slave monitor pause */
#define XIICPS_TIME_OUT_OFFSET		0x1CU  /**< Time Out */
#define XIICPS_IMR_OFFSET			0x20U  /**< Interrupt Enabled Mask */
#define XIICPS_IER_OFFSET			0x24U  /**< Interrupt Enable */
#define XIICPS_IDR_OFFSET			0x28U  /**< Interrupt Disable */

#define XIICPS_EVENT_COMPLETE_SEND	0x0001U  /**< Transmit Complete Event*/
#define XIICPS_EVENT_COMPLETE_RECV	0x0002U  /**< Receive Complete Event*/
#define XIICPS_EVENT_TIME_OUT		0x0004U  /**< Transfer timed out */
#define XIICPS_EVENT_ERROR			0x0008U  /**< Receive error */
#define XIICPS_EVENT_ARB_LOST		0x0010U  /**< Arbitration lost */
#define XIICPS_EVENT_NACK			0x0020U  /**< NACK Received */
#define XIICPS_EVENT_SLAVE_RDY		0x0040U  /**< Slave ready */
#define XIICPS_EVENT_RX_OVR			0x0080U  /**< RX overflow */
#define XIICPS_EVENT_TX_OVR			0x0100U  /**< TX overflow */
#define XIICPS_EVENT_RX_UNF			0x0200U  /**< RX underflow */

#define XIICPS_CR_DIV_A_MASK	0x0000C000U /**< Clock Divisor A */
#define XIICPS_CR_DIV_A_SHIFT			14U /**< Clock Divisor A shift */
#define XIICPS_DIV_A_MAX				 4U /**< Maximum value of Divisor A */
#define XIICPS_CR_DIV_B_MASK	0x00003F00U /**< Clock Divisor B */
#define XIICPS_CR_DIV_B_SHIFT			 8U /**< Clock Divisor B shift */
#define XIICPS_CR_CLR_FIFO_MASK	0x00000040U /**< Clear FIFO, auto clears*/
#define XIICPS_CR_SLVMON_MASK	0x00000020U /**< Slave monitor mode */
#define XIICPS_CR_HOLD_MASK		0x00000010U /**<  Hold bus 1=Hold scl,
												0=terminate transfer */
#define XIICPS_CR_ACKEN_MASK	0x00000008U /**< Enable TX of ACK when
												Master receiver*/
#define XIICPS_CR_NEA_MASK		0x00000004U /**< Addressing Mode 1=7 bit,
												0=10 bit */
#define XIICPS_CR_MS_MASK		0x00000002U /**< Master mode bit 1=Master,
												0=Slave */
#define XIICPS_CR_RD_WR_MASK	0x00000001U /**< Read or Write Master
												transfer  0=Transmitter,
												1=Receiver*/
#define XIICPS_CR_RESET_VALUE			 0U /**< Reset value of the Control
												register */
#define XIICPS_7_BIT_ADDR_OPTION	0x01U  /**< 7-bit address mode */
#define XIICPS_10_BIT_ADDR_OPTION	0x02U  /**< 10-bit address mode */
#define XIICPS_SLAVE_MON_OPTION		0x04U  /**< Slave monitor mode */
#define XIICPS_REP_START_OPTION		0x08U  /**< Repeated Start */

#define REG_TEST_VALUE    0x00000005U
#define XIicPs_ReadIER(BaseAddress) \
    XIicPs_ReadReg((BaseAddress), XIICPS_IER_OFFSET)
#define XIicPs_WriteReg(BaseAddress, RegOffset, RegisterValue) \
    XIicPs_Out32((BaseAddress) + (u32)(RegOffset), (u32)(RegisterValue))



    typedef struct {
		u32 Option;
		u32 Mask;
} OptionsMap;



#endif