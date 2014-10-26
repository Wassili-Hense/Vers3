/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// ENC28J60 ethernet phy interface

#include "../../config.h"

#ifdef ENC28J60_PHY

#include "enc28j60_hw.h"
#include "enc28j60_net.h"

#define MAX_FRAME_BUF   (sizeof(eth_frame_t) + sizeof(ip_packet_t) + sizeof(udp_packet_t) + MQTTSN_MSG_SIZE)

#define SUBNET_BROADCAST    (ip_addr | ~ip_mask)
#define NET_BROADCAST       0xFFFFFFFF

#if (ENC28J60_PHY == 1)
#define LAN_ADDR            phy1addr
#elif (ENC28J60_PHY == 2)
#define LAN_ADDR            phy2addr
#endif  //  ENC28J60_PHY

// node MAC & IP addresses
static uint8_t          mac_addr[6];
static uint32_t         ip_addr;
static uint32_t         ip_mask;
static uint32_t         ip_gateway;
// ARP record
static uint32_t         arp_ip_addr;
static uint8_t          arp_mac_addr[6];

static Queue_t          enc_out_queue = {NULL, NULL, 0, 0};

//////////////////////////////////////////////////////////////////////
// Ethernet Section

// send Ethernet frame
// fields must be set:
//  - frame.target_mac
//  - frame.type
static void eth_send(uint16_t len, eth_frame_t * pFrame)
{
  memcpy(pFrame->sender_mac, mac_addr, 6);
  enc28j60PacketSend(sizeof(eth_frame_t) + len, (uint8_t *)pFrame);
}

// send Ethernet frame back
static void eth_reply(uint16_t len, eth_frame_t * pFrame)
{
  memcpy(pFrame->target_mac, pFrame->sender_mac, 6);
  memcpy(pFrame->sender_mac, mac_addr, 6);
  enc28j60PacketSend(sizeof(eth_frame_t) + len, (uint8_t *)pFrame);
}
// End Ethernet Section
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
// ARP Section

// resolve MAC address
static void arp_resolve_req(uint32_t node_ip_addr, eth_frame_t * pFrame)
{
  arp_message_t *arp = (void*)(pFrame->data);

  // send request
  memset(pFrame->target_mac, 0xff, 6);
  pFrame->type = ETH_TYPE_ARP;
  arp->hw_type = ARP_HW_TYPE_ETH;
  arp->proto_type = ARP_PROTO_TYPE_IP;
  arp->hw_addr_len = 6;
  arp->proto_addr_len = 4;
  arp->opcode = ARP_TYPE_REQUEST;
  memcpy(arp->sender_mac, mac_addr, 6);
  memcpy(arp->sender_ip, &ip_addr, 4);
  memset(arp->target_mac, 0x00, 6);
  memcpy(arp->target_ip, &node_ip_addr, 4);

  eth_send(sizeof(arp_message_t), pFrame);
}

// process arp packet
static void arp_filter(uint16_t len, eth_frame_t * pFrame)
{
  if(len < sizeof(arp_message_t))
    return;

  arp_message_t *arp = (void *)pFrame->data;
  
  enc28j60_GetPacket((void *)arp,  sizeof(arp_message_t));

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
    memset(pFrame->target_mac, 0xFF, 6);
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

#ifdef NET_WITH_ICMP
static void icmp_filter(uint16_t len, eth_frame_t * pFrame);
#endif  //  NET_WITH_ICMP
static void udp_filter(uint16_t len, eth_frame_t * pFrame);

// process IP packet
static void ip_filter(uint16_t len, eth_frame_t * pFrame)
{
  if(len < sizeof(ip_packet_t))
    return;

  uint16_t hcs;
  ip_packet_t *ip = (void*)(pFrame->data);
  
  enc28j60_GetPacket((void *)ip,  sizeof(ip_packet_t));

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
  
  enc28j60_GetPacket((void *)icmp, sizeof(icmp_echo_packet_t));
  
  if(icmp->type == ICMP_TYPE_ECHO_RQ)
  {
    enc28j60_GetPacket((void *)icmp->data,  len - sizeof(icmp_echo_packet_t));

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
static void udp_send(uint16_t len, eth_frame_t *pFrame)
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

// process UDP packet
static void udp_filter(uint16_t len, eth_frame_t * pFrame)
{
    if(len < sizeof(udp_packet_t))
        return;

    ip_packet_t *ip = (void*)(pFrame->data);
    udp_packet_t *udp = (void*)(ip->data);

    enc28j60_GetPacket((void *)udp, sizeof(udp_packet_t));
  
    len = htons(udp->len) - sizeof(udp_packet_t);
  
    if((udp->target_port == MQTTSN_UDP_PORT) && (len <= (sizeof(MQTTSN_MESSAGE_t) + 1)))
    {
        // Passive ARP Resolve
        if(arp_ip_addr == 0)
        {
            memcpy(&arp_ip_addr, ip->sender_ip, 4);
            memcpy(arp_mac_addr, pFrame->sender_mac, 6);
        }
    
        MQ_t * pRx_buf = mqAlloc(sizeof(MQ_t));
        if(pRx_buf == NULL)
            return;
      
        enc28j60_GetPacket((void*)pRx_buf->raw, len);
        memcpy(pRx_buf->LAN_ADDR, ip->sender_ip, 4);
        pRx_buf->Length = len;
    
        if(!mqEnqueue(&enc_out_queue, pRx_buf))
            mqFree(pRx_buf);
    }
}
// End UDP Section
//////////////////////////////////////////////////////////////////////

void ENC28J60_Init(void)
{
    MQ_t * pBuf;
    while((pBuf = mqDequeue(&enc_out_queue)) != NULL)
        mqFree(pBuf);

    uint8_t Len;
    // Read Configuration data
    Len = sizeof(mac_addr);
    ReadOD(objMACAddr,    MQTTSN_FL_TOPICID_PREDEF, &Len, (uint8_t *)mac_addr);
    Len = sizeof(ip_addr);
    ReadOD(objIPAddr,     MQTTSN_FL_TOPICID_PREDEF, &Len, (uint8_t *)&ip_addr);
    ReadOD(objIPMask,     MQTTSN_FL_TOPICID_PREDEF, &Len, (uint8_t *)&ip_mask);
    ReadOD(objIPRouter,   MQTTSN_FL_TOPICID_PREDEF, &Len, (uint8_t *)&ip_gateway);

    //initialize enc28j60
    enc28j60Init(mac_addr);

    // Init Variable
    arp_ip_addr = 0;
}

void ENC28J60_Send(void *pBuf)
{
    eth_frame_t * pFrame = (void *)mqAlloc(MAX_FRAME_BUF);
    if(pFrame == NULL)
    {
        mqFree(pBuf);
        return;
    }

    ip_packet_t *ip = (void*)(pFrame->data);
    udp_packet_t *udp = (void*)(ip->data);

    memcpy(ip->target_ip, &(((MQ_t *)pBuf)->LAN_ADDR), 4);
    udp->target_port = MQTTSN_UDP_PORT;
    udp->sender_port = MQTTSN_UDP_PORT;
    uint16_t len = ((MQ_t *)pBuf)->Length;
    memcpy((void*)(udp->data), &(((MQ_t *)pBuf)->raw), len);
    mqFree(pBuf);
    udp_send(len, pFrame);
  
    mqFree(pFrame);
}

void * ENC28J60_Get(void)
{
    // Rx Section
    if(en28j60_DataRdy())
    {
        uint16_t len = enc28j60_GetPacketLen();
       if(len > sizeof(eth_frame_t))
        {
            eth_frame_t * pFrame = (void *)mqAlloc(MAX_FRAME_BUF);
            if(pFrame != NULL)
            {
                enc28j60_GetPacket((void *)pFrame,  sizeof(eth_frame_t));

                if(pFrame->type == ETH_TYPE_ARP)
                    arp_filter(len - sizeof(eth_frame_t), pFrame);
                else if(pFrame->type == ETH_TYPE_IP)
                    ip_filter(len - sizeof(eth_frame_t), pFrame);
            
                mqFree(pFrame);
            }
        }
        enc28j60_ClosePacket();
    }

    return mqDequeue(&enc_out_queue);
}
#endif  //  ENC28J60_PHY
