
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include "op_common.h"
#include "xil_io.h"
#include "xil_types.h"
#include "xstatus.h"
#include "op_peidian.h"

u8 *i2c_test_t(unsigned char type, unsigned char id, unsigned char addr, unsigned char reg, unsigned char data);

void peidian_get(u8 enid, u8 enAddr, u8 adcId, u8 adcAddr)
{
    u8 *result = NULL;
    u8 i2cAddr = 0;
    u8 i2cReg = 0;
    u8 i2cADCchannel = 1; // 1表示ADC通道1，2表示ADC通道2，，4表示ADC通道4
    // 选通IO口
    i2c_test_t(0x01, enid, enAddr, 0x00, 0x01); // i2c选通后续的地址, 让0x22,0x23可见。
    // 先使能,打开全部使能
    i2c_test_t(0x01, enid, 0x22, 0x0c, 0x00); // 0x22上的IO口全部使能。

    //===>通道1<====
    // 读数据  给0x48 0x49 0x4a 0x4b发送对应的参数获取数据
    // 参数是从0x8c 0x9c 0xac 0xbc 0xcc 0xdc 0xec 0xfc循环发送
    i2c_test_t(0x01, adcId, adcAddr, 0x00, i2cADCchannel); // i2c选通通道1, 让 0x48 0x49 0x4a 0x4b可见。
    //common_save_file("adc_data.txt", &i2cADCchannel, 1);

    i2cAddr = 0x48;
    i2cReg = 0x8c;
    common_save_file("adc_data.txt", &i2cAddr, 1);
    for (i2cReg = 0x8c; i2cReg != 0x0c; i2cReg += 0x10)
    {
        result = i2c_test_t(0x00, adcId, i2cAddr, i2cReg, 0x00); // 从0x48 0x49 0x4a 0x4b读值
        common_save_file("adc_data.txt", &i2cReg, 1);
        common_save_file("adc_data.txt", result, 2);
    }
    i2cAddr = 0x49;
    i2cReg = 0x8c;
    common_save_file("adc_data.txt", &i2cAddr, 1);
    for (i2cReg = 0x8c; i2cReg != 0x0c; i2cReg += 0x10)
    {
        // if (i2cReg == 0xbc || i2cReg == 0xfc)
        // {
        //     result[0] = 0;
        //     result[1] = 0;
        //     common_save_file("adc_data.txt", &i2cReg, 1);
        //     common_save_file("adc_data.txt", result, 2);
        //     continue;
        // }
        result = i2c_test_t(0x00, adcId, i2cAddr, i2cReg, 0x00); // 从0x48 0x49 0x4a 0x4b读值
        common_save_file("adc_data.txt", &i2cReg, 1);
        common_save_file("adc_data.txt", result, 2);
    }

    i2c_test_t(0x01, enid, 0x22, 0x0d, 0x00);     // 0x22上的IO口全部使能。
    i2c_test_t(0x01, adcId, adcAddr, 0x00, 0x01); // i2c选通通道1, 让 0x48 0x49 0x4a 0x4b可见。
    i2cAddr = 0x4a;
    i2cReg = 0x8c;
    common_save_file("adc_data.txt", &i2cAddr, 1);
    for (i2cReg = 0x8c; i2cReg != 0x0c; i2cReg += 0x10)
    {
        result = i2c_test_t(0x00, adcId, i2cAddr, i2cReg, 0x00); // 从0x48 0x49 0x4a 0x4b读值
        common_save_file("adc_data.txt", &i2cReg, 1);
        common_save_file("adc_data.txt", result, 2);
    }
    i2cAddr = 0x4b;
    i2cReg = 0x8c;
    result[0] = 0;
    result[1] = 0;
    common_save_file("adc_data.txt", &i2cAddr, 1);
    for (i2cReg = 0x8c; i2cReg != 0x0c; i2cReg += 0x10)
    {
        result = i2c_test_t(0x00, adcId, i2cAddr, i2cReg, 0x00); // 从0x48 0x49 0x4a 0x4b读值
        common_save_file("adc_data.txt", &i2cReg, 1);
        common_save_file("adc_data.txt", result, 2);
    }

    //===>通道2<====
    // 从 90 92 94 96 --->读数据  给0x48 0x49 0x4a 0x4b发送对应的参数获取数据
    i2cADCchannel = 0x02;
    i2c_test_t(0x01, enid, 0x22, 0x0e, 0x00);              // 0x22上的IO口全部使能。
    i2c_test_t(0x01, adcId, adcAddr, 0x00, i2cADCchannel); // i2c选通通道2, 让 0x48 0x49 0x4a 0x4b可见。
    //common_save_file("adc_data.txt", &i2cADCchannel, 1);
    i2cAddr = 0x48;
    i2cReg = 0x8c;
    common_save_file("adc_data.txt", &i2cAddr, 1);
    for (i2cReg = 0x8c; i2cReg != 0x0c; i2cReg += 0x10)
    {
        result = i2c_test_t(0x00, adcId, i2cAddr, i2cReg, 0x00); // 从0x48 0x49 0x4a 0x4b读值
        common_save_file("adc_data.txt", &i2cReg, 1);
        common_save_file("adc_data.txt", result, 2);
    }
    i2cAddr = 0x49;
    i2cReg = 0x8c;
    result[0] = 0;
    result[1] = 0;
    common_save_file("adc_data.txt", &i2cAddr, 1);
    for (i2cReg = 0x8c; i2cReg != 0x0c; i2cReg += 0x10)
    {
        result = i2c_test_t(0x00, adcId, i2cAddr, i2cReg, 0x00); // 从0x48 0x49 0x4a 0x4b读值
        common_save_file("adc_data.txt", &i2cReg, 1);
        common_save_file("adc_data.txt", result, 2);
    }

    i2c_test_t(0x01, enid, 0x23, 0x0c, 0x00);              // 0x23上的IO口全部使能。
    i2c_test_t(0x01, adcId, adcAddr, 0x00, i2cADCchannel); // i2c选通通道2, 让 0x48 0x49 0x4a 0x4b可见。
    i2cAddr = 0x4a;
    i2cReg = 0x8c;
    common_save_file("adc_data.txt", &i2cAddr, 1);
    for (i2cReg = 0x8c; i2cReg != 0x0c; i2cReg += 0x10)
    {
        result = i2c_test_t(0x00, adcId, i2cAddr, i2cReg, 0x00); // 从0x48 0x49 0x4a 0x4b读值
        common_save_file("adc_data.txt", &i2cReg, 1);
        common_save_file("adc_data.txt", result, 2);
    }
    i2cAddr = 0x4b;
    i2cReg = 0x8c;
    result[0] = 0;
    result[1] = 0;
    common_save_file("adc_data.txt", &i2cAddr, 1);
    for (i2cReg = 0x8c; i2cReg != 0x0c; i2cReg += 0x10)
    {
        result = i2c_test_t(0x00, adcId, i2cAddr, i2cReg, 0x00); // 从0x48 0x49 0x4a 0x4b读值
        common_save_file("adc_data.txt", &i2cReg, 1);
        common_save_file("adc_data.txt", result, 2);
    }

    //===>通道4<====只有0x48
    // 从 90 92 94 96 --->读数据  给0x48 0x49 0x4a 0x4b发送对应的参数获取数据
    i2cADCchannel = 0x04;
    i2c_test_t(0x01, enid, 0x23, 0x0d, 0x00);              // 0x22上的IO口全部使能。
    i2c_test_t(0x01, adcId, adcAddr, 0x00, i2cADCchannel); // i2c选通通道2, 让 0x48 0x49 0x4a 0x4b可见。
    //common_save_file("adc_data.txt", &i2cADCchannel, 1);
    i2cAddr = 0x48;
    i2cReg = 0x8c;
    common_save_file("adc_data.txt", &i2cAddr, 1);
    for (i2cReg = 0x8c; i2cReg != 0x0c; i2cReg += 0x10)
    {
        result = i2c_test_t(0x00, adcId, i2cAddr, i2cReg, 0x00); // 从0x48 0x49 0x4a 0x4b读值
        common_save_file("adc_data.txt", &i2cReg, 1);
        common_save_file("adc_data.txt", result, 2);
    }
}