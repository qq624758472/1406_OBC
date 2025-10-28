
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

#include "xil_io.h"
#include "xil_types.h"
#include "../op_common/op_common.h"
#include "op_eth.h"

int sockfd = -1;
int ethIndex = -1;

void eth_set_index(u8 index)
{
    ethIndex = index;
}

signed int mac_init(unsigned long this_mac, unsigned short this_type)
{
    // 创建原始套接字
    sockfd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (sockfd < 0)
    {
        perror("Socket creation failed");
        return -1;
    }
    return 0;
}
/**
 * @brief 
 * 
 * @param des_mac 目标设备的 mac 地址
 * @param sendbuf 发送数据缓存地址
 * @param len 表示需要发送的数据长度
 * @param act_len 实际发送的数据长度
 * @return signed int
 */
signed int mac_send(unsigned long des_mac, unsigned char *sendbuf, unsigned int len, unsigned int *act_len)
{
    unsigned char dest_mac[6] = {0};
    unsigned char *p = des_mac;
    for (int i = 0; i < 6; i++)
    {
        dest_mac[i] = *p;
        p++;
    }
    
    *act_len = send_ethernet_frame(dest_mac, sendbuf, len);
    LOG2("dest_mac: %02x:%02x:%02x:%02x:%02x:%02x   actLen:%d", dest_mac[0], dest_mac[1], dest_mac[2], dest_mac[3], dest_mac[4], dest_mac[5],*act_len);
    return 0;
}
// 发送以太网帧
ssize_t send_ethernet_frame(const unsigned char *dest_mac, const char *data, size_t data_len)
{

    struct sockaddr_ll addr;
    unsigned char buffer[BUFFER_SIZE];

    // 设置以太网头部
    struct ethhdr *eth = (struct ethhdr *)buffer;
    memcpy(eth->h_dest, dest_mac, 6); // 设置目的 MAC 地址
    // 这里可以设置源 MAC 地址为接口的 MAC 地址
    //    根据索引获取接口名称
    char if_name[IFNAMSIZ];
    memset(&if_name, 0, IFNAMSIZ);
    if (if_indextoname(ethIndex, if_name) == NULL) // 默认索引0，第一个网卡
    {
        perror("if_indextoname");
        mac_close();
        exit(EXIT_FAILURE);
    }
    LOG2("IF_name :%s", if_name);
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, if_name, IFNAMSIZ - 1);
    if (ioctl(sockfd, SIOCGIFHWADDR, &ifr) < 0)
    {
        perror("SIOCGIFHWADDR");
        mac_close();
        return -1;
    }

    memcpy(eth->h_source, ifr.ifr_hwaddr.sa_data, 6); // 设置源 MAC 地址
    eth->h_proto = htons(ETH_P_ALL);                  // 设置以太网类型

    // 填充数据
    memcpy(buffer + sizeof(struct ethhdr), data, data_len);

    // 设置 sockaddr_ll 结构
    memset(&addr, 0, sizeof(addr));
    addr.sll_family = AF_PACKET;
    addr.sll_ifindex = if_nametoindex(if_name); // 获取接口索引
    addr.sll_halen = ETH_ALEN;                  // MAC 地址长度
    memcpy(addr.sll_addr, dest_mac, 6);         // 目的 MAC 地址

    // 发送数据包
    ssize_t sent_bytes = sendto(sockfd, buffer, sizeof(struct ethhdr) + data_len, 0, (struct sockaddr *)&addr, sizeof(addr));
    if (sent_bytes < 0)
    {
        perror("Send failed");
        mac_close();
        return -1;
    }

    LOG2("Ethernet frame sent successfully!\n");
    return sent_bytes;
}

/**
 * @brief
 *
 * @param source_mac 接收源设备的 mac 地址
 * @param recvbuf 接收数据缓存地址
 * @param len 表示需要接收的数据长度
 * @param act_len 实际读取的数据长度
 * @return signed int
 */
signed int mac_recv(unsigned long source_mac, unsigned char *recvbuf, unsigned int len, unsigned int *act_len)
{
    // int sockfd;
    struct sockaddr_ll addr;
    unsigned char buffer[BUFFER_SIZE];
    unsigned char *p = source_mac;

    //    根据索引获取接口名称
    char if_name[IFNAMSIZ];
    memset(&if_name, 0, IFNAMSIZ);
    if (if_indextoname(ethIndex, if_name) == NULL) // 默认索引0，第一个网卡
    {
        perror("if_indextoname");
        mac_close();
        exit(EXIT_FAILURE);
    }
    LOG2("IF_name :%s", if_name);
    // 绑定套接字到指定接口
    struct ifreq ifr;
    strncpy(ifr.ifr_name, if_name, IFNAMSIZ - 1);
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("SIOCGIFINDEX");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    memset(&addr, 0, sizeof(addr));
    addr.sll_family = AF_PACKET;
    addr.sll_ifindex = ifr.ifr_ifindex;

    while (1)
    {
        ssize_t recv_len = recvfrom(sockfd, recvbuf, BUFFER_SIZE, 0, NULL, NULL);
        if (recv_len < 0)
        {
            perror("Receive failed");
            break;
        }

        // 解析以太网帧
        struct ethhdr *eth = (struct ethhdr *)recvbuf;
        // LOG("eth->h_dest[0]:0x%02X ;p[0]:0x%02X; eth->h_source[0]:0x%02X", eth->h_dest[0], p[0], eth->h_source[0]);
        // if (eth->h_dest[0] == p[0] && eth->h_dest[1] == p[1] && eth->h_dest[2] == p[2] && eth->h_dest[3] == p[3] && eth->h_dest[4] == p[4] && eth->h_dest[5] == p[5])
        // if (eth->h_source[0] == 0x00 && eth->h_source[1] == 0x1b && eth->h_source[2] == 0x21)
        if (eth->h_source[0] == p[0] && eth->h_source[1] == p[1])
        {
#if 1
            printf("Received packet:\n");
            printf("Source MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", eth->h_source[0], eth->h_source[1], eth->h_source[2], eth->h_source[3], eth->h_source[4], eth->h_source[5]);
            printf("Destination MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", eth->h_dest[0], eth->h_dest[1], eth->h_dest[2], eth->h_dest[3], eth->h_dest[4], eth->h_dest[5]);
            printf("Protocol: 0x%04X\n", ntohs(eth->h_proto));

            // 输出数据部分
            printf("Data: ");
            for (size_t i = sizeof(struct ethhdr); i < recv_len; i++)
            {
                printf("%02X ", recvbuf[i]);
            }
            printf("\n\n");
#endif
        }
    }
    return 0;
}

signed int mac_close(void)
{
    close(sockfd);
    return 0;
}

/**
 * @brief 查询以太网接收 FIFO 的已经使用字节数
 * 
 * @param use_len 
 * @return signed int 
 */
signed int mac_rx_status (unsigned int* use_len)
{
    *use_len = 0;
    return 0;
}

/**
 * @brief 查询以太网发送 FIFO 的已经使用字节数
 * 
 * @param use_len 
 * @return signed int 
 */
signed int mac_tx_status (unsigned int* use_len)
{
    *use_len = 0;
    return 0;
}

int eth_send_test()
{
    int sendLen = 0;
    unsigned char dest_mac[6] = {0x00, 0x0a, 0x35, 0x00, 0xec, 0x48}; // 目标 MAC 地址，需替换 板卡的mac 00:0A:35:00:EC:48
    // unsigned char dest_mac[6] = {0x00, 0x0a, 0x35, 0x00, 0x1e, 0x53}; // 目标 MAC 地址，需替换
    const char *data = "Hello, this is a raw Ethernet frame!";        // 要发送的数据

    mac_init((unsigned long)dest_mac, 0x01FF);
    // 发送以太网帧
    if (mac_send((unsigned long)dest_mac, data, strlen(data), &sendLen) < 0)
    {
        fprintf(stderr, "Failed to send Ethernet frame.\n");
        return -1;
    }
    mac_close();
    return 0;
}

int eth_recv_test()
{
    unsigned char buffer[BUFFER_SIZE];
    memset(buffer, 0, BUFFER_SIZE);
    // unsigned char source_mac[6] = {0xaa, 0x22, 0xa3, 0x44, 0x55, 0xaa}; //D8-43-AE-96-6F-D6
    unsigned char source_mac[6] = {0x00, 0x1b, 0x21, 0x3b, 0x02, 0xac};
    unsigned int act_len = 0;
    mac_init((unsigned long)source_mac, 0x01FF);
    mac_recv((unsigned long)source_mac, buffer, BUFFER_SIZE, &act_len);
    LOG("act_len = %d", act_len);
    mac_close();
    return 0;
}