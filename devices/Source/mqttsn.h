/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _MQTTSN_H
#define _MQTTSN_H

typedef enum e_MQTTSN_POLL_STATUS
{
  MQTTSN_POLL_STAT_NOP = 0,
#ifdef ASLEEP
  MQTTSN_POLL_STAT_ASLEEP,
  MQTTSN_POLL_STAT_AWAKE,
#endif  //  ASLEEP
  MQTTSN_POLL_STAT_CONNECTED,
  MQTTSN_POLL_STAT_DISCONNECTED
}MQTTSN_POLL_STATUS_t;

// Current Status
typedef enum e_MQTTSN_STATUS
{
  MQTTSN_STATUS_DISCONNECTED = 0,
  MQTTSN_STATUS_DHCP,
  MQTTSN_STATUS_SEARCHGW,
  MQTTSN_STATUS_OFFLINE,
  MQTTSN_STATUS_PRE_CONNECT,
  MQTTSN_STATUS_CONNECT,
#ifdef ASLEEP
  MQTTSN_STATUS_POST_CONNECT,
  MQTTSN_STATUS_PRE_ASLEEP,
  MQTTSN_STATUS_ASLEEP,
  MQTTSN_STATUS_AWAKE,
  MQTTSN_STATUS_POST_AWAKE,
#endif  //  ASLEEP
}e_MQTTSN_STATUS_t;

typedef enum e_TRACE_LEVEL
{
    lvlDEBUG    = 0,
    lvlINFO,
    lvlWARNING,
    lvlERROR
}TRACE_LEVEL_t;

typedef void (*cbMQTTSN_PARSER_t)(MQ_t * pMqBuf);   // Callback on PHY data ready.

void mqttsn_trace_msg(uint8_t Level, MQ_t * pMessage);

void mqttsn_parser_phy1(MQ_t * pPHY1outBuf);
void mqttsn_parser_phy2(MQ_t * pPHY2outBuf);

void MQTTSN_Init(void);
e_MQTTSN_STATUS_t MQTTSN_GetStatus(void);

bool MQTTSN_CanSend(void);
void MQTTSN_Send(e_MQTTSN_MSGTYPE_t      MsgType,
                 uint8_t                 Flags,
                 uint16_t                TopicId);

#endif
