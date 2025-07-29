/******************************************************************************
* COPYRIGHT Beijing UCAS Space Technology Co.,Ltd
*******************************************************************************

*******************************************************************************
* �ļ�����: mram.c
* ��������: �����ڴ�����ģ��
* ʹ��˵��:
* �ļ�����:
* ��д����: 2018/4/17 
* �޸���ʷ:
* �޸İ汾  �޸�����     �޸���        �޸�����
* -----------------------------------------------------------------------------
* 01a      2018/4/17    �µ»ԡ�     ���������汾 
*******************************************************************************/

/******************************* �����ļ����� ********************************/
#include <stdlib.h>  
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>   
#include <sys/mman.h>
#include <linux/i2c.h>   
#include <linux/i2c-dev.h>
#include <errno.h>
#include "drv_common.h"
#include "op_common/op_common.h"
/******************************* �ֲ��궨�� **********************************/
/******************************* �ֲ�����ԭ������ ****************************/
/******************************* ȫ�ֱ�������/��ʼ�� *************************/
void xil_memcpy(void* dst, const void* src, unsigned int cnt)
{
	char *d = (char*)(void *)dst;
	const char *s = src;

	while (cnt >= sizeof (int)) 
	{
		*(int*)d = *(int*)s;
		d += sizeof (int);
		s += sizeof (int);
		cnt -= sizeof (int);
	}
	
	while ((cnt) > 0U)
	{
		*d = *s;
		d += 1U;
		s += 1U;
		cnt -= 1U;
	}
}

void *devm_map(unsigned long addr, int len, int *phandle)
{
	off_t offset;
	void *map_base; 

	if ((*phandle = open("/dev/mem", O_RDWR | O_SYNC, 0666)) == -1) 
	{
		printf("cannot open '/dev/mem'\n");
		goto err_open;
	}
	//printf("/dev/mem opened.\n");

	/*
	 * Map it
	 */

	/* offset for mmap() must be page aligned */
	offset = addr & ~(sysconf(_SC_PAGE_SIZE) - 1);

	map_base = mmap(NULL, len + addr - offset, PROT_READ | PROT_WRITE,
			MAP_SHARED, *phandle, offset);
	if (map_base == MAP_FAILED) 
	{
		printf("mmap failed\n");
		goto err_mmap;
	}
	//printf("Memory mapped at address %p.\n", map_base); 

	return map_base + addr - offset;

err_mmap:
	close(*phandle);

err_open:
	return NULL;
}

void devm_unmap(void *virt_addr, int len, int *phandle)
{
	unsigned long addr;

	if (*phandle == -1)
	{
		printf("'/dev/mem' is closed\n");
		return;
	}

	/* page align */
	addr = (((unsigned long)virt_addr) & ~(sysconf(_SC_PAGE_SIZE) - 1));
	munmap((void *)addr, len + (unsigned long)virt_addr - addr);
	close(*phandle);
	*phandle = -1;
}

/***********************************************************************
* ��������: InitQueue  
* ��    ��: ��ʼ�����к���
* ��    ��: Q   ����     ���п�����
* ��������: 0   �ɹ�			-1 ʧ��                                
*************************************************************************/
int InitQueue(SqQueue *Q, unsigned int maxqsize)
{
	if(NULL == Q)
	{
		printf("InitQueue Err,The Entry parameter Is NULL\n");
		return -1;	
	}	
	/* int sem_init(sem_t *sem, int pshared, unsigned int value);  
	* pshared�����ź��������ͣ������ֵΪ0���ͱ�ʾ����ź����ǵ�ǰ���̵ľֲ��ź�����
	* �����ź����Ϳ����ڶ������֮�乲����valueΪsem�ĳ�ʼֵ��
	*/
	int res = sem_init(&(Q->queue_sem), 0, 1);
	int i;
	                                   
	if (res != 0)                                  
	{                                              
		fprintf(stderr, "Semaphore init failed %s\n", strerror(errno));
		return -1;                     
	}

	Q->queuesize = maxqsize;
	Q->front = Q->rear = 0;
	Q->queFullCnt = 0;
	Q->queEmptyCnt = 0;
	Q->recvCnt = 0;
	Q->sendCnt = 0;
	Q->base = (QElemType *)malloc(Q->queuesize * sizeof(QElemType));
	if(Q->base == NULL)
	{
		fprintf(stderr, "Malloc error %s\n", strerror(errno));
		return -1;
	}

	//�����ֶ�����
	for(i=0; i<maxqsize; i++)
	{
		Q->base[i].len = 0;
	}

	return 0;
}

/***********************************************************************
* ��������: EnQueue  
* ��    ��: ��Ӻ���
* ��    ��: Q      ���     ���п�����
						pMsg   ���			��������׵�ַ
						len    ���			������ݳ���
* ��������:  0 �ɹ�			1 ʧ��                                
*************************************************************************/
int EnQueue(SqQueue *Q, MSGTYPE *pMsg, int len)
{

	if((NULL == Q) || (NULL == pMsg) || (len > MSGLEN))
	{
		fprintf(stderr, "Enqueue Entry parameter Error %s\n", strerror(errno));
		return -1;			
	}	

	if(NULL == Q->base)
	{
        return -1;    	
	}	

	sem_wait(&(Q->queue_sem));

	/*
	�ж϶����Ƿ����������������˵��û����ȡ�����ݣ���ʱ����������������
	*/
	if(((Q->rear+1) % Q->queuesize) == Q->front)
	{
		Q->queFullCnt += 1;
		//Q->front = Q->rear = 0; //���������
		sem_post(&(Q->queue_sem));
		//fprintf(stderr, "Queue is full %s\n", strerror(errno));
		return -1;
	}

	memcpy(Q->base[Q->rear].msg, pMsg, len);
	Q->base[Q->rear].len = len;    
	Q->rear = (Q->rear+1) % Q->queuesize;

	Q->recvCnt += 1;
	sem_post(&(Q->queue_sem));

	return 0;
}

/***********************************************************************
* ��������: DeQueue  
* ��    ��: ���Ӻ���
* ��    ��: Q      ���     ���п�����
						pMsg   ����			���������׵�ַ
						len    ����			�������ݳ���
* ��������:  0 �ɹ�			-1 ʧ��                                
*************************************************************************/
int DeQueue(SqQueue *Q, MSGTYPE *pMsg, int *pLen)
{

    if((NULL == Q) || (NULL == pMsg) || (NULL == pLen))
    {
    		fprintf(stderr, "DeQueue Entry parameter Is NULL %s\n", strerror(errno));
        return -1;				
    }
    	
    if(NULL == Q->base)
    {
        return -1;    	
    }	
    
    sem_wait(&(Q->queue_sem));    
    
    if(Q->front == Q->rear)
    {
        Q->queEmptyCnt += 1;
        sem_post(&(Q->queue_sem));
        //printf("Queue is empty\n");
        return -1;
    }
    
    memcpy(pMsg, Q->base[Q->front].msg, Q->base[Q->front].len);
    *pLen = Q->base[Q->front].len;
    Q->base[Q->front].len = 0;    
    Q->front = (Q->front+1) % Q->queuesize;

    Q->sendCnt += 1;

    sem_post(&(Q->queue_sem));

    return 0;
}

/***********************************************************************
* ��������: DestroyQueue  
* ��    ��: ���ٶ��к���
* ��    ��: Q      ���     ���п�����
* ��������:  OK �ɹ�			ERR ʧ��                                
*************************************************************************/
int DestroyQueue(SqQueue *Q)
{
	if(NULL == Q)
	{
		printf("DestroyQueue Err,The Entry parameter Is NULL\n");
		return -1;
	}	

	sem_wait(&(Q->queue_sem));

	if(NULL == Q->base)
	{
		sem_post(&(Q->queue_sem));
		return -1;
	}	
	free(Q->base);
	Q->base = NULL;
	sem_post(&(Q->queue_sem));

	sem_destroy(&(Q->queue_sem)); 
    
    return 0;
}

/***********************************************************************
* ��������: ClearQueue  
* ��    ��: ��ն��к���
* ��    ��: Q      ���     ���п�����
* ��������:  ��                               
*************************************************************************/
void ClearQueue(SqQueue *Q)
{
	if(NULL == Q)
	{
		printf("ClearQueue Err,The Entry parameter Is NULL\n");
		return;			
	}	
	sem_wait(&(Q->queue_sem));

	Q->front = Q->rear = 0; //��ն���

	sem_post(&(Q->queue_sem));

	return;
}

//select������ʱ����
int mySleep(int sec, int msec)
{
	struct timeval tv;
	tv.tv_sec = sec;
	tv.tv_usec = msec * 1000;

	select(0, NULL, NULL, NULL, &tv);

	return 0;
}

void myUsleep(int usec)
{
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = usec;

	select(0, NULL, NULL, NULL, &tv);

	return;	
}

void print_data(char *data, int len)
{
	//printf("---- print_data function ----");
	for(int i=0; i<len; i++)
	{
		/* ÿ16���ֽ�һ�У�������׵�ַ */
		if((i%16) == 0)
		{
			if(i>0)
			{
				printf("\n");
			}
			printf("<offset:%04x>  ", i);
		}

		/* ÿ8���ֽ�����һ���ո� */
		if((i%16) == 8)
		{
			printf("  ");
		}

		/* ���ÿ���ֽ� */
		printf("%02x ", data[i]);

		if(i>=1023)
		{
			break;
		}
	}

	printf("\n");
	
	return;
}

void print_data_16bit(unsigned short *data, int len)
{
	for(int i=0; i<len; i++)
	{
		/* ÿ8��16bit��һ�У�������׵�ַ */
		if((i%8) == 0)
		{
			if(i>0)
			{
				printf("\n");
			}
			printf("<offset:%04x>  ", i*2);
		}

		/* ÿ4��16bit������һ���ո� */
		if((i%8) == 4)
		{
			printf("  ");
		}

		/* ���ÿ��16bit�� */
		printf("%04x ", data[i]);

		if(i>=512)
		{
			break;
		}
	}

	printf("\n");
	
	return;
}

int system_cmd(const char *command)
{
	int status;
	status = system(command);

	if (-1 == status)
	{
		printf("system_cmd: system call=%s fail!\n", command);
		return ERROR;
	}
	else
	{
		if (WIFEXITED(status))
		{
			if (0 == WEXITSTATUS(status))
			{
				return OK;
			}
			else
			{
				printf("system_cmd: system call=%s fail, exit status: %d\n", 
					command, WEXITSTATUS(status));
				return ERROR;
			}
		}

		printf("system_cmd: system call=%s fail, exit status: %d\n", 
				command, WEXITSTATUS(status));
		return ERROR;
	}
}
/********************************************** Դ�ļ����� **********************************/

