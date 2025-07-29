/*
 * @Description: /
 * @Version: 2.0
 * @Autor: ruog__
 * @Date: 2024-09-06 17:26:17
 * @LastEditors: ruog__
 * @LastEditTime: 2024-10-24 10:19:27
 */

#ifndef __OP_DS__
#define __OP_DS__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h> //文件操作
#include <termios.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <linux/limits.h>
#include <mtd/mtd-abi.h>

#include "../op_common/xil_io.h"
#include "../op_common/xil_types.h"


//64'h2893_3daf_0f00_00f2      64'h28e1_6ab0_0f00_0062
//18B20新id : 64'h283817b00f0000b1    64'h2812daaf0f0000a4

#define DS18B20_TEST_ID_0 0x28a71b760d00007a
#define DS18B20_TEST_ID_1 0x2812fa750d000009
#define DS18B20_TEST_ID_2 0x2803c1760d000019 
#define DS18B20_TEST_ID_3 0x28aba6760d0000a4
#define DS18B20_TEST_ID_4 0x28393f760d00003f
#define DS18B20_TEST_ID_5 0x288aae760d00001b
#define DS18B20_TEST_ID_6 0x2895a3760d000088
#define DS18B20_TEST_ID_7 0x28ffb9760d000045
#define DS18B20_TEST_ID_8 0x28e0b4760d0000d6
#define DS18B20_TEST_ID_9 0x28bb45760d00006b

#define DS18B20_0_high_byte 0x283817b0
#define DS18B20_0_low__byte 0x0f0000b1
#define DS18B20_1_high_byte 0x2812daaf
#define DS18B20_1_low__byte 0x0f0000a4

void ds_test();

/**
 * @brief  给chan路-WIRE总线的id通道写入id
 *
 * @param chan 取值 1-8
 * @param id  取值 0-7
 * @param idData
 */
void ds_set_id(u8 chan, u8 id, u64 idData);

void ds_test_read(int chan);
#endif