/*
 * @Description: 
 * @Version: 2.0
 * @Autor: ruog__
 * @Date: 2024-12-25 16:00:24
 * @LastEditors: ruog__
 * @LastEditTime: 2024-12-25 16:00:25
 */
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

#include "op_dmalib.h"
#include "../op_common/op_common.h"


#if 0
void main()
{

}


#endif