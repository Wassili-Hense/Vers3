/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _NET_H
#define _NET_H

#define htons(x)   (((x)&0x00FF)<<8)+((x)>>8)
#define htonl(x)   ((((x)&0xFF000000)>>24)|(((x)&0x00FF0000)>>8)|(((x)&0x0000FF00)<<8)|(((x)&0x000000FF)<<24))

// Configuration section
#define NET_WITH_ICMP

#define IP_PACKET_TTL       64

#define MQTTSN_UDP_PORT     htons(1883)

// Ethernet Section
#define ETH_TYPE_ARP        htons(0x0806)
#define ETH_TYPE_IP         htons(0x0800)

typedef struct eth_frame
{
  uint8_t   target_mac[6];
  uint8_t   sender_mac[6];
  uint16_t  type;
  uint8_t   data[];
} eth_frame_t;

// ARP Section
#define ARP_HW_TYPE_ETH     htons(0x0001)
#define ARP_PROTO_TYPE_IP   htons(0x0800)

#define ARP_TYPE_REQUEST    htons(1)
#define ARP_TYPE_RESPONSE   htons(2)

typedef struct arp_message
{
  uint16_t  hw_type;
  uint16_t  proto_type;
  uint8_t   hw_addr_len;
  uint8_t   proto_addr_len;
  uint16_t  opcode;
  uint8_t   sender_mac[6];
  uint8_t   sender_ip[4];
  uint8_t   target_mac[6];
  uint8_t   target_ip[4];
} arp_message_t;

// IP Section
#define IP_PROTOCOL_ICMP    1
#define IP_PROTOCOL_TCP     6
#define IP_PROTOCOL_UDP     17

typedef struct ip_packet
{
  uint8_t   ver_head_len;
  uint8_t   tos;
  uint16_t  total_len;
  uint16_t  fragment_id;
  uint16_t  flags_framgent_offset;
  uint8_t   ttl;
  uint8_t   protocol;
  uint16_t  cksum;
  uint8_t   sender_ip[4];
  uint8_t   target_ip[4];
  uint8_t   data[];
} ip_packet_t;

// ICMP Section
#define ICMP_TYPE_ECHO_RQ   8
#define ICMP_TYPE_ECHO_RPLY 0

typedef struct icmp_echo_packet
{
  uint8_t   type;
  uint8_t   code;
  uint16_t  cksum;
  uint16_t  id;
  uint16_t  seq;
  uint8_t   data[];
} icmp_echo_packet_t;

// UDP Section
typedef struct udp_packet
{
  uint16_t sender_port;
  uint16_t target_port;
  uint16_t len;
  uint16_t cksum;
  uint8_t data[];
} udp_packet_t;

#endif