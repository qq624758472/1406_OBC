/*
 * @Description: 
 * @Version: 2.0
 * @Autor: ruog__
 * @Date: 2024-10-09 16:01:50
 * @LastEditors: ruog__
 * @LastEditTime: 2025-01-22 11:01:09
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <sys/ioctl.h>

#ifndef __ETH__
#define __ETH__

#define BUFFER_SIZE 1518 // 以太网帧的最大大小

ssize_t send_ethernet_frame(const unsigned char *dest_mac, const char *data, size_t data_len);

int eth_send_test();
int eth_recv_test();

void eth_set_index(u8 index);

signed int mac_rx_status(unsigned int *use_len);
signed int mac_tx_status(unsigned int *use_len);

//============对外接口定义=================
signed int mac_init(unsigned long this_mac, unsigned short this_type);


/**
 * @brief 
 * 
 * @param des_mac 目标设备的 mac 地址
 * @param sendbuf 发送数据缓存地址
 * @param len 表示需要发送的数据长度
 * @param act_len 实际发送的数据长度
 * @return signed int
 */
signed int mac_send(unsigned long des_mac, unsigned char *sendbuf, unsigned int len, unsigned int *act_len);


/**
 * @brief
 *
 * @param source_mac 接收源设备的 mac 地址
 * @param recvbuf 接收数据缓存地址
 * @param len 表示需要接收的数据长度
 * @param act_len 实际读取的数据长度
 * @return signed int
 */
signed int mac_recv(unsigned long source_mac, unsigned char *recvbuf, unsigned int len, unsigned int *act_len);


#endif