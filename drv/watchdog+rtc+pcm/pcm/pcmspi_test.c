#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "pcmspi.h"
#include <time.h>

#define ADDRESS 0x1000

// 用于计算时间差的宏
#define DIFF_NS(start, end) \
    (((end).tv_sec - (start).tv_sec) * 1000000000LL + ((end).tv_nsec - (start).tv_nsec))

    

int str2int(char *str)
{
    int i=0, len=0, rv=0;
    if(str == NULL) return 0;
    len = strlen(str);
    if(strncmp(str, "0x", 2) == 0)
    {
        sscanf(str, "%x", &rv);
        return rv;
    }
    sscanf(str, "%d", &rv);
    return rv;
}

#define PRNT_LINE 16
#define PRNT_HEAD 8
#define PRNT_TAIL 8
#define PRNT_ADDRMASK (~((uint32_t)(PRNT_LINE-1)))
void printdata(uint8_t *data, uint32_t startaddr, int len)
{
    int i,j,flag=1;
    uint32_t lineaddr, lastaddr=0;
    printf("\nAddress   ");
    for(i=0; i<PRNT_LINE; i++) printf("%3d", i);
    printf("\n");
    j=0;
    for(i=0; i<len; )
    {
        lineaddr = (startaddr+i)&PRNT_ADDRMASK;
        if(j==0)
        {
            if(((lineaddr-(startaddr&PRNT_ADDRMASK))>(PRNT_HEAD*PRNT_LINE)) && \
                (((startaddr+len)&PRNT_ADDRMASK)-lineaddr)>(PRNT_TAIL*PRNT_LINE)) 
            {
                i += PRNT_LINE;
                if(flag)
                {
                    flag=0;
                    printf("\n...   ...");
                }
                continue;
            }
            printf("\n%08x: ", lineaddr);
        }
        if(lineaddr+j<startaddr) printf("   ");
        else { printf(" %02x", data[i]); i++;}
        j=(j+1)%PRNT_LINE;
    }
    printf("\n");
}

int main(int argc, char **argv)
{
    int rv;
    uint8_t *writebuf, *readbuf;
    int datalen, startaddr;
    int i, errcnt;

    struct timespec start, end;
    int64_t elapsed_ns;    
    float time_sec, rate;

    if(argc <3) 
    {
        printf("Usage: pcmspi_test [start addr] [length].\n");
        return 0;
    }
    startaddr = str2int(argv[1]);
    datalen = str2int(argv[2]);
    printf("startaddr = %#x, datalen = %d\n", startaddr, datalen);

    printf("\nInit PCM\n");
    rv = pcmspi_init();
    if(rv == PCMSPI_EC_OK) printf("init ok!\n");
    else 
    {
        printf("init failed (%d)\n", rv);
        goto out;
    }
 // =============== 新增阶段：先写入并读取 0xFF ===============
// 分配缓冲区
    writebuf = (uint8_t *)malloc(datalen);
    readbuf = (uint8_t *)malloc(datalen);
   // verifybuf = (uint8_t *)malloc(datalen); // 用于验证 0xFF
    if(!writebuf || !readbuf )
    {
        printf("allocate data buf failed.\n");
        goto out;
    }
 printf("\n=== PHASE 1: Write and Read 0xFF ===\n");
// 准备 0xFF 数据
    memset(writebuf, 0xFF, datalen);

    printf("Writing 0xFF to address %#x, length %d\n", startaddr, datalen);
    printdata(writebuf, startaddr, datalen);    

    clock_gettime(CLOCK_MONOTONIC, &start);
    rv = pcmspi_write(writebuf, startaddr, datalen);
    clock_gettime(CLOCK_MONOTONIC, &end);
 if(rv <= 0) {
        printf("Write 0xFF failed (%d)\n", rv);
        goto out;
    }

    time_sec = (float)(DIFF_NS(start, end)) / 1000000000.0;
    rate = (float)datalen / time_sec;
    printf("%f second elapsed, write rate %f bytes/second.\n", time_sec, rate);
// 读取验证
    memset(readbuf, 0, datalen); // 清空读缓冲
    printf("Reading back for 0xFF verification...\n");
    clock_gettime(CLOCK_MONOTONIC, &start);
    rv = pcmspi_read(readbuf, startaddr, datalen);
    clock_gettime(CLOCK_MONOTONIC, &end);

    if(rv <= 0) {
        printf("Read 0xFF failed (%d)\n", rv);
        goto out;
    }

    time_sec = (float)(DIFF_NS(start, end)) / 1000000000.0;
    rate = (float)datalen / time_sec;
    printf("%f second elapsed, read rate %f bytes/second.\n", time_sec, rate);
    printdata(readbuf, startaddr, datalen);
    // 验证是否全为 0xFF
    errcnt = 0;
    for(i = 0; i < datalen; i++) {
        if(readbuf[i] != 0xFF) {
            if(errcnt < 10) // 只打印前10个错误
                printf("Mismatch at offset %d: expected 0xFF, got 0x%02x\n", i, readbuf[i]);
            errcnt++;
        }
    }
    if(errcnt == 0)
        printf(" All data verified as 0xFF. Proceeding...\n");
    else {
        printf(" Verification failed with %d errors. Aborting.\n", errcnt);
        goto out;
    }
    if(readbuf) free(readbuf);
    if(writebuf) free(writebuf);



    printf("\nPrepare write buffer.\n");
    writebuf = (uint8_t *)malloc(datalen);
    if(writebuf == NULL)
    {
        printf("allocate data buf failed.\n");
        goto out;
    }
    for(i=0; i<datalen/2; i++)
    {
        ((uint16_t *)writebuf)[i] = (uint16_t)i;
    }
    printdata(writebuf, startaddr, datalen);

    printf("\nWrite data\n");

    clock_gettime(CLOCK_MONOTONIC, &start);
    rv = pcmspi_write(writebuf, startaddr, datalen);
    clock_gettime(CLOCK_MONOTONIC, &end);

    if(rv>0) printf("%d bytes written.\n", rv);
    else
    {
        printf("write failed (%d)\n", rv);
        goto out;
    }

    time_sec = (float)(DIFF_NS(start, end))/1000000000.0;
    rate = (float)datalen / time_sec;
    printf("%f second elapsed, write rate %f bytes/second.\n", time_sec, rate);

    printf("\nread data\n");
    readbuf = (uint8_t *)malloc(datalen);
    if(readbuf == NULL)
    {
        printf("allocate data buf failed.\n");
        goto out;
    }
    memset(readbuf, 0, datalen);

    clock_gettime(CLOCK_MONOTONIC, &start);
    rv = pcmspi_read(readbuf, startaddr, datalen);
    clock_gettime(CLOCK_MONOTONIC, &end);

    if(rv>0) printf("%d bytes.\n", rv);
    else
    {
        printf("read failed (%d)\n", rv);
        goto out;
    }

    time_sec = (float)(DIFF_NS(start, end))/1000000000.0;
    rate = (float)datalen / time_sec;
    printf("%f second elapsed, read rate %f bytes/second.\n", time_sec, rate);

    printdata(readbuf, startaddr, datalen);


    printf("\nverify data\n");
    errcnt = 0;
    for(i=0; i<datalen; i++)
    {
        if(readbuf[i] != writebuf[i]) 
        {
            printf("mismatch %d: [%#x] %#x should be %#x\n", errcnt, i, readbuf[i], writebuf[i]);
            errcnt ++;
        }
    }
    printf("total mismatch: %d\n", errcnt);
out:
    if(readbuf) free(readbuf);
    if(writebuf) free(writebuf);
    pcmspi_exit();
}