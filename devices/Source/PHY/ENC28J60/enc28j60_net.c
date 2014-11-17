/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#include "../../config.h"

#if (defined ENC28J60_PHY)

#include "enc28j60_hw.h"
#include "enc28j60_net.h"

#define phy_send            enc28j60PacketSend
#define phy_get             enc28j60_GetPacket
#define phy_skip            enc28j60_Skip

#define SUBNET_BROADCAST    (ip_addr | ~ip_mask)
#define NET_BROADCAST       0xFFFFFFFF

// defined in main.c
extern volatile uint32_t second_count;

// node MAC & IP addresses
static uint8_t          mac_addr[6];
uint32_t                ip_addr;
static uint32_t         ip_mask;
static uint32_t         ip_gateway;
// ARP record
static uint32_t         arp_ip_addr;
static uint8_t          arp_mac_addr[6];

#ifdef NET_WITH_DHCP
static dhcp_status_t    dhcp_status = DHCP_DISABLED;
static uint32_t         dhcp_retry_time;
static uint32_t         dhcp_transaction_id;
#endif

// Initialize network variables
void enc28j60_init_net(void)
{
    uint8_t Len;
    // Read Configuration data
    Len = sizeof(mac_addr);
    ReadOD(objMACAddr,    MQTTSN_FL_TOPICID_PREDEF, &Len, (uint8_t *)mac_addr);
    Len = sizeof(ip_addr);
    ReadOD(objIPAddr,     MQTTSN_FL_TOPICID_PREDEF, &Len, (uint8_t *)&ip_addr);
    ReadOD(objIPMask,     MQTTSN_FL_TOPICID_PREDEF, &Len, (uint8_t *)&ip_mask);
    ReadOD(objIPRouter,   MQTTSN_FL_TOPICID_PREDEF, &Len, (uint8_t *)&ip_gateway);

    //initialize enc28j60 hardware
    enc28j60Init(mac_addr);

    // Init Variables
    arp_ip_addr = 0;

#ifdef NET_WITH_DHCP
    if(ip_addr != 0xFFFFFFFF)
    {
        dhcp_status = DHCP_DISABLED;
    }
    else
    {
        ip_addr = 0;
        dhcp_status = DHCP_INIT;
        dhcp_retry_time = second_count + 2;
    }
#endif
}

//////////////////////////////////////////////////////////////////////
// Ethernet Section

// send Ethernet frame
// fields must be set:
//  - frame.target_mac
//  - frame.type
static void eth_send(uint16_t len, eth_frame_t * pFrame)
{
    memcpy(pFrame->sender_mac, mac_addr, 6);
    while(!enc28j60_CanSend(sizeof(eth_frame_t) + len));
    enc28j60_PutData(sizeof(eth_frame_t) + len, (uint8_t *)pFrame);
    enc28j60_Send();
}

// send Ethernet frame back
static void eth_reply(uint16_t len, eth_frame_t * pFrame)
{
    memcpy(pFrame->target_mac, pFrame->sender_mac, 6);
    eth_send(len, pFrame);
}

static void arp_filter(uint16_t len, eth_frame_t * pFrame);
static void ip_filter(uint16_t len, eth_frame_t * pFrame);

// process Ethernet packet
void eth_filter(uint16_t len, eth_frame_t * pFrame)
{
    if(pFrame->type == ETH_TYPE_ARP)
        arp_filter(len - sizeof(eth_frame_t), pFrame);
    else if(pFrame->type == ETH_TYPE_IP)
        ip_filter(len - sizeof(eth_frame_t), pFrame);
}
// End Ethernet Section
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// ARP Section

// resolve MAC address
static void arp_resolve_req(uint32_t node_ip_addr, eth_frame_t * pFrame)
{
    arp_message_t *arp = (void*)(pFrame->data);
    
    uint8_t pos;

    // send request
    for(pos = 0; pos < 6; pos++) pFrame->target_mac[pos] = 0xFF;
    pFrame->type = ETH_TYPE_ARP;
    arp->hw_type = ARP_HW_TYPE_ETH;
    arp->proto_type = ARP_PROTO_TYPE_IP;
    arp->hw_addr_len = 6;
    arp->proto_addr_len = 4;
    arp->opcode = ARP_TYPE_REQUEST;
    memcpy(arp->sender_mac, mac_addr, 6);
    memcpy(arp->sender_ip, &ip_addr, 4);
    for(pos = 0; pos < 6; pos++) arp->target_mac[pos] = 0;
    memcpy(arp->target_ip, &node_ip_addr, 4);

    eth_send(sizeof(arp_message_t), pFrame);
}

// process arp packet
static void arp_filter(uint16_t len, eth_frame_t * pFrame)
{
    if(len < sizeof(arp_message_t))
        return;

    arp_message_t *arp = (void *)pFrame->data;
  
    phy_get((void *)arp,  sizeof(arp_message_t));

    if((arp->hw_type != ARP_HW_TYPE_ETH) ||
       (arp->proto_type != ARP_PROTO_TYPE_IP) ||
       (memcmp(&arp->target_ip, &ip_addr, 4) != 0))
        return;

    if(arp->opcode == ARP_TYPE_REQUEST)
    {
        arp->opcode = ARP_TYPE_RESPONSE;
        memcpy(arp->target_mac, arp->sender_mac, 6);
        memcpy(arp->sender_mac, mac_addr, 6);
        memcpy(arp->target_ip, arp->sender_ip, 4);
        memcpy(arp->sender_ip, &ip_addr, 4);
        eth_reply(sizeof(arp_message_t), pFrame);
    }
    else if(arp->opcode == ARP_TYPE_RESPONSE)
    {
        memcpy(&arp_ip_addr, arp->sender_ip, 4);
        memcpy(arp_mac_addr, arp->sender_mac, 6);
    }
}
// End ARP Section
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// IP Section

// calculate IP checksum
static uint16_t ip_cksum(uint32_t sum, uint8_t *buf, size_t len)
{
    uint16_t uiTmp;

    while(len > 1)
    {
        uiTmp = ((uint16_t)*buf << 8) | *(buf+1);
        buf += 2;
        len -= 2;  

        sum += uiTmp;
    }

    if(len)
        sum += (uint16_t)*buf << 8;

    while(sum >> 16)
        sum = (sum & 0xffff) + (sum >> 16);

    uiTmp = sum & 0xFFFF;
    uiTmp = htons(uiTmp);
    uiTmp = ~uiTmp;

    return uiTmp;
}

// send IP packet
// fields must be set:
//  - ip.dst
//  - ip.proto
// len is IP packet payload length
static void ip_send(uint16_t len, eth_frame_t *pFrame)
{
    ip_packet_t *ip = (void*)(pFrame->data);
  
    uint32_t t_ip;
    memcpy(&t_ip, ip->target_ip, 4);

    // apply route
    if((t_ip == SUBNET_BROADCAST) || (t_ip == NET_BROADCAST))
    {
        uint8_t pos;
        for(pos = 0; pos < 6; pos++) pFrame->target_mac[pos] = 0xFF;
    }
    else
    {
        uint32_t route_ip;
    
        if(((t_ip ^ ip_addr) & ip_mask) == 0)
            route_ip = t_ip;
        else if(((ip_gateway ^ ip_addr) & ip_mask) == 0)
            route_ip = ip_gateway;
        else
            return;

        if(route_ip == arp_ip_addr)
            memcpy(pFrame->target_mac, arp_mac_addr, 6);
        else
        {
            arp_resolve_req(route_ip, pFrame);
            return;
        }
    }

    // send packet
    len += sizeof(ip_packet_t);
  
    pFrame->type = ETH_TYPE_IP;

    ip->ver_head_len = 0x45;
    ip->tos = 0;
    ip->total_len = htons(len);
    ip->fragment_id = 0;
    ip->flags_framgent_offset = 0;
    ip->ttl = IP_PACKET_TTL;
    ip->cksum = 0;
    memcpy(ip->sender_ip, &ip_addr, 4);
    ip->cksum = ip_cksum(0, (void*)ip, sizeof(ip_packet_t));

    eth_send(len, pFrame);
}

// send IP packet back
// len is IP packet payload length
static void ip_reply(uint16_t len, eth_frame_t * pFrame)
{
    ip_packet_t *ip = (void*)(pFrame->data);

    len += sizeof(ip_packet_t);

    ip->total_len = htons(len);
    ip->fragment_id = 0;
    ip->flags_framgent_offset = 0;
    ip->ttl = IP_PACKET_TTL;
    ip->cksum = 0;
    memcpy(ip->target_ip, ip->sender_ip, 4);
    memcpy(ip->sender_ip, &ip_addr, 4);
    ip->cksum = ip_cksum(0, (void*)ip, sizeof(ip_packet_t));

    eth_reply(len, pFrame);
}

static void icmp_filter(uint16_t len, eth_frame_t * pFrame);
static void udp_filter(uint16_t len, eth_frame_t * pFrame);

// process IP packet
static void ip_filter(uint16_t len, eth_frame_t * pFrame)
{
    if(len < sizeof(ip_packet_t))
        return;

    uint16_t hcs;
    ip_packet_t *ip = (void*)(pFrame->data);
  
    phy_get((void *)ip,  sizeof(ip_packet_t));

    hcs = ip->cksum;
    ip->cksum = 0;

    if((ip->ver_head_len != 0x45) ||    // Version 4, Header length 20 bytes
       (ip_cksum(0, (void*)ip, sizeof(ip_packet_t)) != hcs))
        return;

    len = htons(ip->total_len) - sizeof(ip_packet_t);
  
    uint32_t t_ip;
    memcpy(&t_ip, ip->target_ip, 4);

    switch(ip->protocol)
    {
#ifdef NET_WITH_ICMP
        case IP_PROTOCOL_ICMP:
            if(t_ip == ip_addr)
                icmp_filter(len, pFrame);
            break;
#endif  //  NET_WITH_ICMP
        case IP_PROTOCOL_UDP:
            if((t_ip == ip_addr) || (t_ip == SUBNET_BROADCAST) || (t_ip == NET_BROADCAST))
                udp_filter(len, pFrame);
            break;
    }
}
// End IP Section
//////////////////////////////////////////////////////////////////////

#ifdef NET_WITH_ICMP
//////////////////////////////////////////////////////////////////////
// ICMP Section
// process ICMP packet
static void icmp_filter(uint16_t len, eth_frame_t * pFrame)
{
    if((len < sizeof(icmp_echo_packet_t)) || (len > (MAX_FRAME_BUF - sizeof(ip_packet_t) - sizeof(eth_frame_t))))
        return;

    ip_packet_t *ip = (void*)pFrame->data;
    icmp_echo_packet_t *icmp = (void*)ip->data;

    phy_get((void *)icmp, sizeof(icmp_echo_packet_t));
  
    if(icmp->type == ICMP_TYPE_ECHO_RQ)
    {
        phy_get((void *)icmp->data,  len - sizeof(icmp_echo_packet_t));

        icmp->type = ICMP_TYPE_ECHO_RPLY;
        icmp->cksum += 8; // update cksum
        ip_reply(len, pFrame);
    }
}
// End ICMP Section
//////////////////////////////////////////////////////////////////////
#endif  //  NET_WITH_ICMP

//////////////////////////////////////////////////////////////////////
// UDP Section

// send UDP packet
// fields must be set:
//  - ip.target_ip
//  - udp.sender_port
//  - udp.target_port
// uint16_t len is UDP data payload length
void udp_send(uint16_t len, eth_frame_t *pFrame)
{
    ip_packet_t *ip = (void*)(pFrame->data);
    udp_packet_t *udp = (void*)(ip->data);

    len += sizeof(udp_packet_t);

    ip->protocol = IP_PROTOCOL_UDP;
    memcpy(ip->sender_ip, &ip_addr, 4);

    udp->len = htons(len);
    udp->cksum = 0;
    udp->cksum = ip_cksum(len + IP_PROTOCOL_UDP, (uint8_t*)udp - 8, len + 8);

    ip_send(len, pFrame);
}

void phy_mqttsn_filter(uint16_t len, eth_frame_t * pFrame);
static void dhcp_filter(uint16_t len, eth_frame_t *pFrame);

// process UDP packet
static void udp_filter(uint16_t len, eth_frame_t * pFrame)
{
    if(len < sizeof(udp_packet_t))
        return;

    ip_packet_t *ip = (void*)(pFrame->data);
    udp_packet_t *udp = (void*)(ip->data);

    phy_get((void *)udp, sizeof(udp_packet_t));

    len = htons(udp->len) - sizeof(udp_packet_t);

    if((udp->target_port == MQTTSN_UDP_PORT) && (len <= (sizeof(MQTTSN_MESSAGE_t) + 1)))
    {
        // Passive ARP Resolve
        if(arp_ip_addr == 0)
        {
            memcpy(&arp_ip_addr, ip->sender_ip, 4);
            memcpy(arp_mac_addr, pFrame->sender_mac, 6);
        }

        phy_mqttsn_filter(len, pFrame);
    }
#ifdef NET_WITH_DHCP
    else if(udp->target_port == DHCP_CLIENT_PORT)
    {
        dhcp_filter(len, pFrame);
    }
#endif  //  NET_WITH_DHCP
}
// End UDP Section
//////////////////////////////////////////////////////////////////////

#ifdef NET_WITH_DHCP
//////////////////////////////////////////////////////////////////////
// DHCP Section

#define dhcp_add_option(ptr, optcode, type, value)  \
    ((dhcp_option_t*)ptr)->code = optcode;          \
    ((dhcp_option_t*)ptr)->len = sizeof(type);      \
    *(type*)(((dhcp_option_t*)ptr)->data) = value;  \
    ptr += sizeof(dhcp_option_t) + sizeof(type);    \
    if(sizeof(type)&1) *(ptr++) = 0;

static void dhcp_filter(uint16_t len, eth_frame_t *pFrame)
{
    if(len < SIZEOF_DHCP_MESSAGE)   // sizeof dhcp_message w/o options
        return;
  
    ip_packet_t *ip = (void*)pFrame->data;
    udp_packet_t *udp = (void*)(ip->data);
    dhcp_message_t *dhcp = (void*)(udp->data);
  
    uint32_t magic_cookie;
  
    phy_get((void *)dhcp, sizeof(dhcp_message_t));
    phy_skip(192);
    phy_get((void *)&magic_cookie, 4);
    
    // Check if DHCP messages directed to us
    if((dhcp->operation != DHCP_OP_REPLY) ||
       (memcmp(dhcp->transaction_id,  &dhcp_transaction_id, 4) != 0) ||
       (magic_cookie != DHCP_MAGIC_COOKIE))
        return;
        
    len -= SIZEOF_DHCP_MESSAGE;
    
    // parse DHCP message
    uint8_t * op = dhcp->options;
    while(len >= sizeof(dhcp_option_t))
    {
        dhcp_option_t * option = (void*)op;
        if(option->code == DHCP_CODE_PAD)
        {
            op++;
            len--;
        }
        else if(option->code == DHCP_CODE_END)
        {
            break;
        }
        else
        {
/*
            switch(option->code)
            {
                case DHCP_CODE_MESSAGETYPE:
                    type = *(option->data);
                    break;
                case DHCP_CODE_SUBNETMASK:
                    offered_net_mask = *(uint32_t*)(option->data);
                    break;
                case DHCP_CODE_GATEWAY:
                    offered_gateway = *(uint32_t*)(option->data);
                    break;
                case DHCP_CODE_DHCPSERVER:
                    renew_server = *(uint32_t*)(option->data);
                    break;
                case DHCP_CODE_LEASETIME:
                    temp = *(uint32_t*)(option->data);
                    lease_time = ntohl(temp);
                    if(lease_time > 21600)
                        lease_time = 21600;
                    break;
                //case DHCP_CODE_RENEWTIME:
                //    temp = *(uint32_t*)(option->data);
                //    renew_time = ntohl(temp);
                //    if(renew_time > 21600)
                //        renew_time = 21600;
                //    break;
            }
*/
            op += sizeof(dhcp_option_t) + option->len;
            len -= sizeof(dhcp_option_t) + option->len;
        }
    }
}

void dhcp_poll(void)
{
    if(dhcp_status == DHCP_DISABLED)
    {
        return;
    }

    eth_frame_t     * pFrame;
    ip_packet_t     * ip;
    udp_packet_t    * udp;
    dhcp_message_t  * dhcp;

    uint32_t t_ip = NET_BROADCAST;
    uint16_t length;
    uint8_t pos;

    // Initiate DHCP, startup/lease end
    if(dhcp_retry_time < second_count)
    {
        //dhcp_status = 
        dhcp_retry_time = second_count + 15;

        pFrame = (void *)mqAlloc(MAX_FRAME_BUF);
        if(pFrame == NULL)
            return;

        enc28j60EnableBroadcast();

        ip = (void*)(pFrame->data);
        udp = (void*)(ip->data);
        dhcp = (void*)(udp->data);

        for(pos = 0; pos < sizeof(dhcp_message_t); pos++)   udp->data[pos] = 0;
        
        dhcp_transaction_id = (halRNG()<<16) | halRNG();

        dhcp->operation = DHCP_OP_REQUEST;
        dhcp->hw_addr_type = DHCP_HW_ADDR_TYPE_ETH;
        dhcp->hw_addr_len = 6;
        memcpy(dhcp->transaction_id, &dhcp_transaction_id, 4);
        dhcp->flags = DHCP_FLAG_BROADCAST;
        memcpy(dhcp->hw_addr, mac_addr, 6);

        uint8_t * pOpt = (void*)(dhcp->options);
        
        //dhcp->magic_cookie = DHCP_MAGIC_COOKIE;
        *(pOpt++) = ((DHCP_MAGIC_COOKIE>>0) & 0xFF);
        *(pOpt++) = ((DHCP_MAGIC_COOKIE>>8) & 0xFF);
        *(pOpt++) = ((DHCP_MAGIC_COOKIE>>16) & 0xFF);
        *(pOpt++) = ((DHCP_MAGIC_COOKIE>>24) & 0xFF);

        dhcp_add_option(pOpt, DHCP_CODE_MESSAGETYPE, uint8_t, DHCP_MESSAGE_DISCOVER);
        *(pOpt++) = DHCP_CODE_END;

        length = (uint8_t*)pOpt - (uint8_t*)(dhcp->options);
    }
    else
        return;

    // Make UDP header
    uint16_t len = length + (192 + sizeof(dhcp_message_t) + sizeof(udp_packet_t));

    udp->sender_port = DHCP_CLIENT_PORT;
    udp->target_port = DHCP_SERVER_PORT;
    udp->len = htons(len);
    udp->cksum = 0;

    memcpy(ip->sender_ip, &ip_addr, 4);
    memcpy(ip->target_ip, &t_ip, 4);

    udp->cksum = ip_cksum(len + IP_PROTOCOL_UDP, (uint8_t*)udp - 8, length + (8 + sizeof(dhcp_message_t) + sizeof(udp_packet_t)));

    // Make IP Header
    len += sizeof(ip_packet_t);
    ip->ver_head_len = 0x45;
    ip->tos = 0;
    ip->total_len = htons(len);
    ip->fragment_id = 0;
    ip->flags_framgent_offset = 0;
    ip->ttl = IP_PACKET_TTL;
    ip->protocol = IP_PROTOCOL_UDP;
    ip->cksum = 0;
    ip->cksum = ip_cksum(0, (void*)ip, sizeof(ip_packet_t));
    
    // Make Ethernet header
    len += sizeof(eth_frame_t);
    for(pos = 0; pos < 6; pos++) pFrame->target_mac[pos] = 0xFF;
    memcpy(pFrame->sender_mac, mac_addr, 6);
    pFrame->type = ETH_TYPE_IP;

    // Send packet to PHY
    while(!enc28j60_CanSend(len));
    enc28j60_PutData((sizeof(eth_frame_t) +
                      sizeof(ip_packet_t) +
                      sizeof(udp_packet_t) +
                      sizeof(dhcp_message_t)),
                    (uint8_t *)pFrame);
    enc28j60_Fill(192);
    enc28j60_PutData(length, dhcp->options);
    enc28j60_Send();

    mqFree(pFrame);
}
// End DHCP Section
//////////////////////////////////////////////////////////////////////
#endif  //  NET_WITH_DHCP
#endif  //  ENC28J60_PHY
