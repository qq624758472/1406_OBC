/*
 * @Description: v
 * @Version: 2.0
 * @Autor: ruog__
 * @Date: 2024-09-06 17:19:58
 * @LastEditors: ruog__
 * @LastEditTime: 2024-10-24 10:27:14
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#include "xil_io.h"
#include "xil_types.h"
#include "xstatus.h"

#include "ds.h"

#include "../op_common/xil_io.h"
#include "../op_common/xil_types.h"
#include "../op_common/op_common.h"

extern void *ctrl_all;

/**
 * @brief
 *
 * @param chan 取值 1-8
 * @param id  取值 0-9
 * @param idData
 */
void ds_set_id(u8 chan, u8 id, u64 idData)
{
    u32 height = (u32)(idData >> 32);
    u32 low = (u32)(idData & 0xFFFFFFFF);

    u32 startOffset = 0;
    switch (chan)
    {
    case 1:
        startOffset = 0x0 + id * 8;
        break;
    case 2:
        startOffset = 0x50 + id * 8;
        break;
    case 3:
        startOffset = 0xa0 + id * 8;
        break;
    case 4:
        startOffset = 0xf0 + id * 8;
        break;
    case 5:
        startOffset = 0x140 + id * 8;
        break;
    case 6:
        startOffset = 0x190 + id * 8;
        break;
    case 7:
        startOffset = 0x1e0 + id * 8;
        break;
    case 8:
        startOffset = 0x230 + id * 8;
        break;
    default:
        break;
    }

    COMMO_WriteReg(0, startOffset, ctrl_all, height);
    COMMO_WriteReg(0, startOffset + 4, ctrl_all, low);
    LOG("startOffset: 0x%x, height:0x%x, low:0x%x", startOffset, height, low);
}

void ds_test_t()
{
    common_mmap_ctr(); // 使能ctrl_all
    int count = 0;
    int num = 0;
    int resetrHdl0 = -1;
    u64 idSet[10] = {DS18B20_TEST_ID_0, DS18B20_TEST_ID_1, DS18B20_TEST_ID_2, DS18B20_TEST_ID_3,
                     DS18B20_TEST_ID_4, DS18B20_TEST_ID_5, DS18B20_TEST_ID_6,
                     DS18B20_TEST_ID_7, DS18B20_TEST_ID_8, DS18B20_TEST_ID_9};

    void *snCtl = devm_map(0x43cc0000, 0xff, &resetrHdl0);
    if (snCtl == NULL)
    {
        LOG("snCtl: mmap fail!\n");
        return -1;
    }
    int dsCtrlData = COMMO_ReadReg(0, 0x08, snCtl);
    dsCtrlData |= 1 << 9;
    COMMO_WriteReg(0, 0x08, snCtl, dsCtrlData);

    for (int j = 1; j < 9; j++)
    {
        // 写第0路id
        for (int i = 0; i < 10; i++)
        {
            ds_set_id(j, i, idSet[i]);
        }
    }
}

void ds_test_read(int chan)
{
    // 使能
    int count = 0;
    void *enable = devm_map(0x41280000, 0xf, &count);
    if (enable == NULL)
    {
        LOG("enable: mmap fail!\n");
        exit(-1);
    }
    COMMO_SEND_MAICHONG(enable, 0X0, chan);

    sleep(2);

    int ctl_fd = -1;
    void *wd = devm_map(0x43cd0000, 0xffffff, &ctl_fd);
    if (wd == NULL)
    {
        LOG("devm_map: mmap fail!\n");
        return -1;
    }
    u32 bound = 0x28 * chan + 0x24;
    for (int i = 0; i <= bound; i += 4)
    {
        int tmp = COMMO_ReadReg(0, i, wd);
        LOG("offset:%d, data:0x%x\n", i, tmp);
    }

    devm_unmap(enable, 0xf, &count);
    devm_unmap(wd, 0xffffff, &ctl_fd);
    return;
}

void ds_test()
{
    common_mmap_ctr(); // 使能ctrl_all
    int count = 0;
    int num = 0;
    int resetrHdl0 = -1;

    // o_slv_reg2_9	 0x0008 第9bit 	1	0	EN_DS18B20	DS18B20使能控制	1：使能  0：不使能      默认0
    void *snCtl = devm_map(0x43cc0000, 0xff, &resetrHdl0);
    if (snCtl == NULL)
    {
        LOG("snCtl: mmap fail!\n");
        return -1;
    }
    int dsCtrlData = COMMO_ReadReg(0, 0x08, snCtl);
    dsCtrlData |= 1 << 9;
    COMMO_WriteReg(0, 0x08, snCtl, dsCtrlData);

    // 写第0路id
    for (int i = 0; i <= 0x3c; i += 4)
    {
        LOG("count:%d,num:%d, i:0x%x\n", count, num, i);
        if (count % 2 == 0 && num % 2 == 0)
            COMMO_WriteReg(0, i, ctrl_all, DS18B20_0_high_byte);
        else if (count % 2 != 0 && num % 2 == 0)
        {
            COMMO_WriteReg(0, i, ctrl_all, DS18B20_0_low__byte);
            num++;
        }
        else if (count % 2 == 0 && num % 2 == 1)
            COMMO_WriteReg(0, i, ctrl_all, DS18B20_1_high_byte);
        else if (count % 2 != 0 && num % 2 == 1)
        {
            COMMO_WriteReg(0, i, ctrl_all, DS18B20_1_low__byte);
            num++;
        }
        else
            LOG("error\n");
        count++;
    }

    // 使能
    system("devmem 0x41280000 32 0xf");
    system("devmem 0x41280000 32 0x0");

    sleep(2);

    int ctl_fd = -1;
    void *wd = devm_map(0x43cd0000, 0xff, &ctl_fd);
    if (wd == NULL)
    {
        LOG("devm_map: mmap fail!\n");
        return -1;
    }
    for (int i = 0; i <= 0x1c; i += 4)
    {
        int tmp = COMMO_ReadReg(0, i, wd);
        LOG("chan:%d, data:0x%x\n", i, tmp);
    }
}