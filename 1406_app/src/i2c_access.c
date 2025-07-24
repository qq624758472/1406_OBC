/******************************************************************************
* COPYRIGHT Beijing UCAS Space Technology Co.,Ltd
*******************************************************************************

*******************************************************************************
* �ļ�����: i2c_access.c
* ��������: i2c�ӿڷ���
* ʹ��˵��:
* �ļ�����:
* ��д����: 2022/1/24 
* �޸���ʷ:
* �޸İ汾  �޸�����     �޸���        �޸�����
* -----------------------------------------------------------------------------
* 01a      2022/1/24    zhanghao     ���������汾 
*******************************************************************************/

/******************************* �����ļ����� ********************************/
#include <stdio.h>   
#include <linux/types.h>   
#include <fcntl.h>   
#include <unistd.h>   
#include <stdlib.h>   
#include <sys/types.h>   
#include <sys/ioctl.h>   
#include <errno.h>   
#include <assert.h>   
#include <string.h>   
#include <linux/i2c.h>   
#include <linux/i2c-dev.h>   
#include "drv_common.h"
#include "i2c_access.h"
#include "op_common/op_common.h"
/******************************* �ֲ��궨�� **********************************/
int hmc_fd;
/******************************* �ֲ�����ԭ������ ****************************/

/******************************* ȫ�ֱ�������/��ʼ�� *************************/ 
/******************************* ����ʵ�� ************************************/
/**************************ʵ�ֺ���********************************************
*����ԭ��:	i2c_open(void)
*��������:	i2c�ӿڴ�
*����ֵ:	�ɹ� Ϊ 0 
*         ʧ�� Ϊ-1
*******************************************************************************/ 
int i2c_open(unsigned char id)
{
	if(id == 0)
	{
    	hmc_fd = open(I2C_0_CHIP, O_RDWR, 0666);
    }
	else if(id == 1)
	{
    	hmc_fd = open(I2C_1_CHIP, O_RDWR, 0666);
    }
	else if(id == 2)
	{
    	hmc_fd = open(I2C_2_CHIP, O_RDWR, 0666);
    }
	else if(id ==3)
	{
    	hmc_fd = open(I2C_3_CHIP, O_RDWR, 0666);
    }
	else if(id ==4)
	{
    	hmc_fd = open(I2C_4_CHIP, O_RDWR, 0666);
    }
	else if(id ==5)
	{
    	hmc_fd = open(I2C_5_CHIP, O_RDWR, 0666);
    }
	else if(id ==6)
	{
    	hmc_fd = open(I2C_6_CHIP, O_RDWR, 0666);
    }	

	if(hmc_fd < 0)
    {
        LOG("open I2C failed !\n");
        return -1;		
    }

	return 0;

}


void i2c_fd_close(int fd)
{
    if(fd > 0)
    {
		close(fd);
		fd = -1;
		LOG("close I2C OK!\n");
    }
}


void i2c_close(void)
{
    if(hmc_fd > 0)
    {
		close(hmc_fd);
		hmc_fd = -1;
		LOG("close I2C OK!\n");
    }
}

int i2cIPMB_Write(unsigned short slave_addr, unsigned char len, unsigned char *dat)
{
    struct i2c_rdwr_ioctl_data packets;  
    struct i2c_msg messages[1];  

//    switch_channel_enable(3);

    ioctl(hmc_fd,I2C_TIMEOUT,1);/*��ʱʱ��*/
    ioctl(hmc_fd,I2C_RETRIES,2);/*�ظ�����*/
  
    messages[0].addr  = slave_addr;  
    messages[0].flags = I2C_M_WT;  
    messages[0].len   = len;    
    messages[0].buf   = dat;
  
    /* Transfer the i2c packets to the kernel and verify it worked */  
    packets.msgs  = messages;  
    packets.nmsgs = 1;  
    if(ioctl(hmc_fd, I2C_RDWR, &packets) < 0)
    {  
        LOG("");
        perror("Unable to  123 send data");  

        return -1;  
    }  
    LOG("");
    return 0;  
}

int i2cIPMB_Read(unsigned short slave_addr, unsigned char len, unsigned char *outbuf)
{
    struct i2c_rdwr_ioctl_data packets;  
    struct i2c_msg messages[1]; 

//    switch_channel_enable(3);

    ioctl(hmc_fd,I2C_TIMEOUT,1000);/*��ʱʱ��*/
    ioctl(hmc_fd,I2C_RETRIES,2);/*�ظ�����*/

    messages[0].addr  = slave_addr;  
    messages[0].flags = I2C_M_RD;  
    messages[0].len   = len; //sizeof(outbuf);
    messages[0].buf   = outbuf;  

    /* Send the request to the kernel and get the result back */  
    packets.msgs      = messages;  
    packets.nmsgs     = 1;  
	if(ioctl(hmc_fd, I2C_RDWR, &packets) < 0)
	{
        perror("Unable to  123 recv data");  
        return -1;  
    }  
  
    return 0;    	
}

int i2c_eeprom_Write(unsigned short slave_addr, unsigned char addr, unsigned char value)
{
    unsigned char outbuf[3]={0};
    struct i2c_rdwr_ioctl_data packets;  
    struct i2c_msg messages[1];  

//    switch_channel_enable(3);

    ioctl(hmc_fd,I2C_TIMEOUT,1);/*��ʱʱ��*/
    ioctl(hmc_fd,I2C_RETRIES,2);/*�ظ�����*/
  
    messages[0].addr  = slave_addr;  
    messages[0].flags = I2C_M_WT;  
    messages[0].len   = 3;    
    messages[0].buf   = outbuf;
      
  
    /* The first byte indicates which register we'll write */  
    outbuf[0] = 0;  
	outbuf[1] = addr;  

    /* The second byte indicates the value we'll write */  
    outbuf[2] = value;
  
    /* Transfer the i2c packets to the kernel and verify it worked */  
    packets.msgs  = messages;  
    packets.nmsgs = 1;  
    if(ioctl(hmc_fd, I2C_RDWR, &packets) < 0)
    {  
        perror("Unable to  123 send data");  
        return -1;  
    }  
  
    return 0;  
}
int i2c_eeprom_Read(unsigned short slave_addr, unsigned char reg, int buf_len, unsigned char *outbuf)
{
	unsigned char inbuf[2]={0};
    struct i2c_rdwr_ioctl_data packets;  
    struct i2c_msg messages[2]; 

//    switch_channel_enable(3);

    ioctl(hmc_fd,I2C_TIMEOUT,1);/*��ʱʱ��*/
    ioctl(hmc_fd,I2C_RETRIES,2);/*�ظ�����*/

	inbuf[0] = 0x0;
	inbuf[1] = reg;

    messages[0].addr  = slave_addr;  
    messages[0].flags = I2C_M_WT;  
    messages[0].len   = 2; //sizeof(outbuf);
    messages[0].buf   = inbuf;  

    /* The data will get returned in this structure */  
    messages[1].addr  = slave_addr;  
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;  
    messages[1].len   = buf_len;  
    messages[1].buf   = outbuf;  

    /* Send the request to the kernel and get the result back */  
    packets.msgs      = messages;  
    packets.nmsgs     = 2;  
	if(ioctl(hmc_fd, I2C_RDWR, &packets) < 0)
	{
        perror("Unable to  123 send data");  
        return -1;  
    }  
  
    return 0;    	
}



int i2c_Write(unsigned short slave_addr, unsigned char reg, unsigned char value)
{
    unsigned char outbuf[2]={0};
    struct i2c_rdwr_ioctl_data packets;  
    struct i2c_msg messages[1];  

//    switch_channel_enable(3);

    ioctl(hmc_fd,I2C_TIMEOUT,1);/*��ʱʱ��*/
    ioctl(hmc_fd,I2C_RETRIES,2);/*�ظ�����*/
  
    messages[0].addr  = slave_addr;  
    messages[0].flags = I2C_M_WT;  
    messages[0].len   = 2;    
    messages[0].buf   = outbuf;
      
  
    /* The first byte indicates which register we'll write */  
    outbuf[0] = reg;  

    /* The second byte indicates the value we'll write */  
    outbuf[1] = value;
  
    /* Transfer the i2c packets to the kernel and verify it worked */  
    packets.msgs  = messages;  
    packets.nmsgs = 1;  
    if(ioctl(hmc_fd, I2C_RDWR, &packets) < 0)
    {  
        perror("Unable to  123 send data");  
        return -1;  
    }  
  
    return 0;  
}

int i2c_Read(unsigned short slave_addr, unsigned char reg, int buf_len, unsigned char *outbuf)
{
    struct i2c_rdwr_ioctl_data packets;  
    struct i2c_msg messages[2]; 

//    switch_channel_enable(3);

    ioctl(hmc_fd,I2C_TIMEOUT,1);/*��ʱʱ��*/
    ioctl(hmc_fd,I2C_RETRIES,2);/*�ظ�����*/

    messages[0].addr  = slave_addr;  
    messages[0].flags = I2C_M_WT;  
    messages[0].len   = 1; //sizeof(outbuf);
    messages[0].buf   = &reg;  

    /* The data will get returned in this structure */  
    messages[1].addr  = slave_addr;  
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;  
    messages[1].len   = buf_len;  
    messages[1].buf   = outbuf;  

    /* Send the request to the kernel and get the result back */  
    packets.msgs      = messages;  
    packets.nmsgs     = 2;  
	if(ioctl(hmc_fd, I2C_RDWR, &packets) < 0)
	{
        perror("Unable to  123 send data");  
        return -1;  
    }  
  
    return 0;    	
}

// ��PCA9548���ƼĴ���д����ֵ
int i2c_wt_pca9548(unsigned short slave_addr, unsigned char value)
{
    struct i2c_rdwr_ioctl_data packets;  
    struct i2c_msg messages[1];  

    ioctl(hmc_fd,I2C_TIMEOUT,5);/*��ʱʱ��*/
    ioctl(hmc_fd,I2C_RETRIES,2);/*�ظ�����*/
  
    messages[0].addr  = slave_addr;  
    messages[0].flags = I2C_M_WT;  
    messages[0].len   = 1;    
    messages[0].buf   = &value;
      
    /* Transfer the i2c packets to the kernel and verify it worked */  
    packets.msgs  = messages;  
    packets.nmsgs = 1;  
    if(ioctl(hmc_fd, I2C_RDWR, &packets) < 0)
    {  
        perror("Unable to  123 send data");  
        return -1;  
    }  
  
    return 0;  
}

// ��PCA9548���ƼĴ���������ֵ
int i2c_rd_pca9548(unsigned short slave_addr, unsigned char *outbuf)
{
    struct i2c_rdwr_ioctl_data packets;  
    struct i2c_msg messages[1]; 

    ioctl(hmc_fd,I2C_TIMEOUT,1);/*��ʱʱ��*/
    ioctl(hmc_fd,I2C_RETRIES,2);/*�ظ�����*/

    messages[0].addr  = slave_addr;  
    messages[0].flags = I2C_M_RD;  
    messages[0].len   = 1; //sizeof(outbuf);
    messages[0].buf   = outbuf;  

    /* Send the request to the kernel and get the result back */  
    packets.msgs      = messages;  
    packets.nmsgs     = 1;  
	if(ioctl(hmc_fd, I2C_RDWR, &packets) < 0)
	{
        perror("Unable to  123 send data");  
        return -1;  
    }  
  
    return 0;    	
}



/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �е�1��λ
����	dev  Ŀ���豸��ַ
reg	   �Ĵ�����ַ
bitNum  Ҫ�޸�Ŀ���ֽڵ�bitNumλ
data  Ϊ0 ʱ��Ŀ��λ������0 ���򽫱���λ
����   �ɹ� Ϊ1 
ʧ��Ϊ0
*******************************************************************************/ 
void IICwriteBit(unsigned char dev, unsigned char reg, unsigned char bitNum, unsigned char data)
{
	unsigned char b;
	i2c_Read(dev, reg, 1, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	i2c_Write(dev, reg, b);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*��������:	    �� �޸� д ָ���豸 ָ���Ĵ���һ���ֽ� �еĶ��λ
����	dev  Ŀ���豸��ַ
reg	   �Ĵ�����ַ
bitStart  Ŀ���ֽڵ���ʼλ
length   λ����
data    ��Ÿı�Ŀ���ֽ�λ��ֵ
����   �ɹ� Ϊ1 
ʧ��Ϊ0
*******************************************************************************/ 
void IICwriteBits(unsigned char dev,unsigned char reg,unsigned char bitStart,unsigned char length,unsigned char data)
{
	
	unsigned char b;
	unsigned char mask;
	i2c_Read(dev, reg, 1, &b);
	mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
	data <<= (8 - length);
	data >>= (7 - bitStart);
	b &= mask;
	b |= data;
	i2c_Write(dev, reg, b);
}


/********************************************** Դ�ļ����� **********************************/
