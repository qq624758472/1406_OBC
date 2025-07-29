/*
 * @Description: 
 * @Version: 2.0
 * @Autor: ruog__
 * @Date: 2024-09-09 14:44:14
 * @LastEditors: ruog__
 * @LastEditTime: 2024-09-09 14:44:08
 */
/******************************************************************************
* COPYRIGHT BEIJING UCAS Space Technology Co.,Ltd
*******************************************************************************

*******************************************************************************
* 文件名称: net.h
* 功能描述: 网络通讯文件
* 使用说明:
* 文件作者:
* 编写日期: 2023/5/09 
* 修改历史:
* 修改版本  修改日期     修改人        修改内容
* -----------------------------------------------------------------------------
* 01a      2023/5/09         创建基础版本
*******************************************************************************/
/******************************** 头文件保护开头 ******************************/
#ifndef __NET_H__
#define __NET_H__

#define FRAME_HEAD_REQUEST 	0xCF1A
#define FRAME_HEAD_ACK  0x99bb
#define USER_ID_CHECK_OK 0x01
#define USER_ID_CHECK_ERR 0x0
#define ZH1_MSG 0x11
#define ZH2_MSG 0x22

#define JISUANJI1_IP "10.0.3.222"
#define JISUANJI2_IP "10.0.3.222"
#define SEND_PORT 2035
#define RECV_PORT 2036

#define QT_RECV_PORT 2002

#define UDP_INFO_MAX_CNT  8
#define TCP  1
#define UDP  2
#define LISTEN_MAX_NUM	10

#ifndef CONNECT_SIZE
#define CONNECT_SIZE 256
#endif

#define PORT 7777
#define MAX_LINE 2048
#define LISTENQ 20

extern IpmbGCdef IpmbGCtest;
int Trans_server_init(void);

#endif
