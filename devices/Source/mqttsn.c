/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// MQTT-SN Library, Version 4.0.2 alfa

#include "config.h"

//#define MQTTSN_DEF_KEEPALIVE    (uint16_t)300                   // Keep Alive Time - 5 Min
#define MQTTSN_DEF_KEEPALIVE    (uint16_t)30
#define MQTTSN_DEF_NRETRY       (uint8_t)3                      // Number of retry's
#define MQTTSN_DEF_TCONNECT     (uint16_t)10                    // Time between Connect's
#define MQTTSN_DEF_TSGW         (uint16_t)15                    // Time between search gateway request
#define MQTTSN_DEF_TDISCONNECT  (uint16_t)30                    // Pause on Disconnect State

#define MQTTSN_DEF_PROTOCOLID   0x01

#define MQTTSN_MAX_RADIUS       3   // Hops to Gateway

// Local Variables
typedef struct
{
    uint8_t                     GatewayAddr[sizeof(PHY1_ADDR_t)];   // Gateway Address
    uint8_t                     phy1addr[sizeof(PHY1_ADDR_t)];
#ifdef PHY2_ADDR_t
    uint8_t                     phy2addr[sizeof(PHY2_ADDR_t)];
#endif

    uint8_t                     GwId;       // Unique Gateway ID
    uint8_t                     Radius;     // Broadcast Radius

    uint16_t                    Tretry;     // Time between retry's
    uint8_t                     Nretry;     // Retry number

    e_MQTTSN_STATUS_t           Status;     // Actual status

    // Register / Subscribe / Publish buffer
    struct
    {
        e_MQTTSN_MSGTYPE_t      MsgType;
        uint8_t                 Flags;
        uint16_t                TopicId;
        uint16_t                MsgId;
    }MsgBuf;

    uint16_t                    MsgId;      // Message ID
}MQTTSN_VAR_t;

static MQTTSN_VAR_t vMQTTSN;

static const PHY1_ADDR_t addr1_undef = ADDR_UNDEF_PHY1;
static const PHY1_ADDR_t addr1_broad = ADDR_BROADCAST_PHY1;
    
#ifdef PHY2_ADDR_t
static const PHY2_ADDR_t addr2_undef = ADDR_UNDEF_PHY2;
#endif  //  PHY2_ADDR_t

#ifndef mqttsn_get_random       // for devices without HW RNG
// Generate pseudo random uint16
static uint16_t mqttsn_get_random(void)
{
    static uint16_t rand16 = 0xA15E;

    // Galois LFSRs
    if(rand16 & 1)
    {
        rand16 >>= 1;
        rand16 ^= 0xB400;
    }
    else
        rand16 >>= 1;
  
    return rand16;
}
#endif  // mqttsn_get_random

static uint16_t mqttsn_get_random_delay(uint16_t delay)
{
    uint32_t ulTmp = delay;
    ulTmp *= mqttsn_get_random();
  
    return (uint16_t)(ulTmp>>16);
}

// Get new outgoing message ID
static uint16_t mqttsn_new_msgid(void)
{
    vMQTTSN.MsgId++;
    if(vMQTTSN.MsgId > 0xFFFE)
        vMQTTSN.MsgId = 1;
    return vMQTTSN.MsgId;
}

#ifdef DIAG_USED
void mqttsn_trace_msg(uint8_t Level, MQ_t * pMessage)
{
    if(pMessage == NULL)
        return;
    
    if(vMQTTSN.Status != MQTTSN_STATUS_CONNECT)
    {
        mqFree(pMessage);
        return;
    }

    if(Level > lvlERROR)
        Level = lvlERROR;

    uint16_t TopicId = objLogD;
    TopicId += Level;
    mqttsn_new_msgid();
    
    pMessage->mq.MsgType = MQTTSN_MSGTYP_PUBLISH;
    pMessage->mq.publish.Flags = (MQTTSN_FL_QOS0 | MQTTSN_FL_TOPICID_PREDEF);
    pMessage->mq.publish.MsgId[0] = vMQTTSN.MsgId>>8;
    pMessage->mq.publish.MsgId[1] = vMQTTSN.MsgId & 0xFF;
    pMessage->mq.publish.TopicId[0] = TopicId>>8;
    pMessage->mq.publish.TopicId[1] = TopicId & 0xFF;
    pMessage->Length += MQTTSN_SIZEOF_MSG_PUBLISH;
    pMessage->mq.Length = pMessage->Length;
    memcpy(pMessage->phy1addr, vMQTTSN.GatewayAddr, sizeof(PHY1_ADDR_t));
    PHY1_Send(pMessage);
}
#endif  //  DIAG_USED

////////////////////////////////////////////////////////////////////////
// Parse incoming messages

// Parse incoming messages from PHY1
void mqttsn_parser_phy1(MQ_t * pPHY1outBuf)
{
    bool msg_from_gw = (memcmp(pPHY1outBuf->phy1addr, vMQTTSN.GatewayAddr, sizeof(PHY1_ADDR_t)) == 0);

#ifdef MQTTSN_TRANSIT_ON_PHY1
    // Transit messages on PHY1
    if(!msg_from_gw && (vMQTTSN.Status == MQTTSN_STATUS_CONNECT))
    {
        switch(pPHY1outBuf->mq.MsgType)
        {
/*
            case MQTTSN_MSGTYP_SEARCHGW:
                // !! ToDo random delay.
                // Gateway Info request on PHY1 from Remote Node
                if(pPHY1outBuf->mq.searchgw.Radius == (vMQTTSN.Radius + 1))
                {
                    // Send Gateway Info message
                    PHY1_ADDR_t s_addr = ADDR_BROADCAST_PHY1;
                    memcpy(pPHY1outBuf->phy1addr, &s_addr, sizeof(PHY1_ADDR_t));
                    pPHY1outBuf->Length = (MQTTSN_SIZEOF_MSG_GWINFO + sizeof(PHY1_ADDR_t));
                    pPHY1outBuf->mq.Length = (MQTTSN_SIZEOF_MSG_GWINFO + sizeof(PHY1_ADDR_t));
                    pPHY1outBuf->mq.MsgType = MQTTSN_MSGTYP_GWINFO;
                    pPHY1outBuf->mq.gwinfo.GwId = vMQTTSN.GwId;
                    memcpy(pPHY1outBuf->mq.gwinfo.GwAdd, vMQTTSN.phy1addr, sizeof(PHY1_ADDR_t));
                    PHY1_Send(pPHY1outBuf);
                    return;
                }
*/
            case MQTTSN_MSGTYP_CONNECT:
            case MQTTSN_MSGTYP_REGISTER:
            case MQTTSN_MSGTYP_REGACK:
            case MQTTSN_MSGTYP_PUBLISH:
            case MQTTSN_MSGTYP_PUBACK:
            case MQTTSN_MSGTYP_SUBSCRIBE:
            case MQTTSN_MSGTYP_PINGREQ:
            //case MQTTSN_MSGTYP_DISCONNECT:        ASleep not realised yet.
#ifdef MQTTSN_USE_DHCP
            case MQTTSN_MSGTYP_DHCPREQ:
#endif  //  MQTTSN_USE_DHCP
            case MQTTSN_MSGTYP_FORWARD:
                {
                // Forward message on PHY1 from Remote Node to Gateway
                uint8_t Size = (MQTTSN_SIZEOF_MSG_FORWARD + sizeof(PHY1_ADDR_t) + 1);
                uint8_t Length = pPHY1outBuf->Length + Size;
                uint8_t pos;

                if(Length > sizeof(MQTTSN_MESSAGE_t))
                    break;

                // Don't use memcpy !
                for(pos = (Length - 1); pos >= Size; pos--)
                    pPHY1outBuf->raw[pos] = pPHY1outBuf->raw[pos - Size];

                // Make forward message header
                pPHY1outBuf->Length = Length;
                pPHY1outBuf->mq.Length = Size;
                pPHY1outBuf->mq.MsgType = MQTTSN_MSGTYP_FORWARD;
                pPHY1outBuf->mq.forward.Ctrl = 0;   // ?? TTL
                pPHY1outBuf->mq.forward.wNodeID[0] = 1;     // PHY1
                memcpy(&pPHY1outBuf->mq.forward.wNodeID[1], pPHY1outBuf->phy1addr, sizeof(PHY1_ADDR_t));
                memcpy(pPHY1outBuf->phy1addr, vMQTTSN.GatewayAddr, sizeof(PHY1_ADDR_t));
                PHY1_Send(pPHY1outBuf);
                }
                return;
            // Unknown, bad or unsupported message
            default:
                break;
        }
        mqFree(pPHY1outBuf);
        return;
    }
#endif  //  MQTTSN_TRANSIT_ON_PHY1

    switch(pPHY1outBuf->mq.MsgType)
    {
        // Advertise message from Gate, equivalent GWINFO 
        case MQTTSN_MSGTYP_ADVERTISE:
            if(vMQTTSN.Status == MQTTSN_STATUS_SEARCHGW)
            {
                memcpy(vMQTTSN.GatewayAddr, pPHY1outBuf->phy1addr, sizeof(PHY1_ADDR_t));
                vMQTTSN.GwId = pPHY1outBuf->mq.advertise.GwId;
                vMQTTSN.Status = MQTTSN_STATUS_OFFLINE;
                vMQTTSN.Tretry = mqttsn_get_random_delay(MQTTSN_DEF_TCONNECT * configTICK_RATE_HZ);
                vMQTTSN.Nretry = MQTTSN_DEF_NRETRY;
            }
            break;
        // Search gateway request from another node
        case MQTTSN_MSGTYP_SEARCHGW:
            if(vMQTTSN.Status == MQTTSN_STATUS_SEARCHGW)
            {
                vMQTTSN.Tretry = mqttsn_get_random_delay(MQTTSN_DEF_TSGW  * configTICK_RATE_HZ) + configTICK_RATE_HZ;
            }
            break;
        // Gateway Info message
        case MQTTSN_MSGTYP_GWINFO:
            if(vMQTTSN.Status == MQTTSN_STATUS_SEARCHGW)
            {
                memcpy(vMQTTSN.GatewayAddr, pPHY1outBuf->phy1addr, sizeof(PHY1_ADDR_t));
                vMQTTSN.GwId = pPHY1outBuf->mq.gwinfo.GwId;
                vMQTTSN.Status = MQTTSN_STATUS_OFFLINE;
                vMQTTSN.Tretry = mqttsn_get_random_delay(MQTTSN_DEF_TCONNECT * configTICK_RATE_HZ);
                vMQTTSN.Nretry = MQTTSN_DEF_NRETRY;
            }
            break;
        // Connack message
        case MQTTSN_MSGTYP_CONNACK:
            if(msg_from_gw)
            {
                if(vMQTTSN.Status == MQTTSN_STATUS_OFFLINE)
                {
                    if(pPHY1outBuf->mq.connack.ReturnCode == MQTTSN_RET_ACCEPTED)
                    {
                        vMQTTSN.Status = MQTTSN_STATUS_PRE_CONNECT;
                        vMQTTSN.Tretry = (MQTTSN_DEF_KEEPALIVE * configTICK_RATE_HZ);
                        vMQTTSN.Nretry = MQTTSN_DEF_NRETRY;
                    }
                }
                // else
                // ToDo
                // Message lost, broker - gateway Problems, Connected another Node with same Address.
                // Potential dangerous
            }
            break;
        // Register Topic request
        case MQTTSN_MSGTYP_REGISTER:
            if(vMQTTSN.Status == MQTTSN_STATUS_CONNECT)
            {
                if(msg_from_gw)
                {
                    pPHY1outBuf->mq.regack.ReturnCode = RegisterOD(&pPHY1outBuf->mq);
                    pPHY1outBuf->Length = MQTTSN_SIZEOF_MSG_REGACK;
                    pPHY1outBuf->mq.Length = MQTTSN_SIZEOF_MSG_REGACK;
                    pPHY1outBuf->mq.MsgType = MQTTSN_MSGTYP_REGACK;
                    PHY1_Send(pPHY1outBuf);
                    return;
                }
            }
            break;
        // RegAck Answer
        case MQTTSN_MSGTYP_REGACK:
            if(msg_from_gw)
            {
                if(vMQTTSN.Status == MQTTSN_STATUS_PRE_CONNECT)
                {
                    if((vMQTTSN.MsgBuf.MsgType == MQTTSN_MSGTYP_REGISTER) &&
                       (vMQTTSN.MsgBuf.MsgId == ((pPHY1outBuf->mq.regack.MsgId[0]<<8) |
                                                  pPHY1outBuf->mq.regack.MsgId[1])))
                    {
                        vMQTTSN.MsgBuf.MsgType = MQTTSN_MSGTYP_PINGREQ;

                        uint16_t index;
                        if(pPHY1outBuf->mq.regack.ReturnCode == MQTTSN_RET_ACCEPTED)
                            index = (pPHY1outBuf->mq.regack.TopicId[0]<<8) |
                                     pPHY1outBuf->mq.regack.TopicId[1];
                        else
                            index = 0;
                        RegAckOD(index);
                        
                        vMQTTSN.Tretry = (MQTTSN_DEF_KEEPALIVE * configTICK_RATE_HZ);
                        vMQTTSN.Nretry = MQTTSN_DEF_NRETRY;
                    }
                }
            }
            break;
        // Publish Topic request
        case MQTTSN_MSGTYP_PUBLISH:
            if(vMQTTSN.Status == MQTTSN_STATUS_CONNECT)
            {
                if(msg_from_gw)
                {
                    uint8_t Flags = pPHY1outBuf->mq.publish.Flags;
                    uint16_t TopicId = (pPHY1outBuf->mq.publish.TopicId[0]<<8) |
                                        pPHY1outBuf->mq.publish.TopicId[1];
                    uint16_t MsgId =   (pPHY1outBuf->mq.publish.MsgId[0]<<8) |
                                        pPHY1outBuf->mq.publish.MsgId[1];
                    // Make PubAck message
                    pPHY1outBuf->mq.puback.ReturnCode = WriteODpack(
                        TopicId,
                        Flags, 
                        (pPHY1outBuf->mq.Length - MQTTSN_SIZEOF_MSG_PUBLISH),
                        (uint8_t *)pPHY1outBuf->mq.publish.Data);
                        
                    // ToDo Not Supported QOS2
                    if((Flags & MQTTSN_FL_QOS_MASK) == MQTTSN_FL_QOS1)           // Need Ack
                    {
                        pPHY1outBuf->Length = MQTTSN_SIZEOF_MSG_PUBACK;
                        pPHY1outBuf->mq.Length = MQTTSN_SIZEOF_MSG_PUBACK;
                        pPHY1outBuf->mq.MsgType = MQTTSN_MSGTYP_PUBACK;
                        pPHY1outBuf->mq.puback.TopicId[0] = TopicId>>8;
                        pPHY1outBuf->mq.puback.TopicId[1] = TopicId & 0xFF;
                        pPHY1outBuf->mq.puback.MsgId[0] = MsgId>>8;
                        pPHY1outBuf->mq.puback.MsgId[1] = MsgId & 0xFF;
                        PHY1_Send(pPHY1outBuf);
                        return;
                    }
                }
            }
            break;
        // PubAck Answer
        case MQTTSN_MSGTYP_PUBACK:
            if(msg_from_gw)
            {
                if(vMQTTSN.Status == MQTTSN_STATUS_CONNECT)
                {
                    if((vMQTTSN.MsgBuf.MsgType == MQTTSN_MSGTYP_PUBLISH) &&
                       (vMQTTSN.MsgBuf.MsgId == ((pPHY1outBuf->mq.puback.MsgId[0]<<8) |
                                                  pPHY1outBuf->mq.puback.MsgId[1])))
                    {
                        vMQTTSN.MsgBuf.MsgType = MQTTSN_MSGTYP_PINGREQ;
                        
                        vMQTTSN.Tretry = (MQTTSN_DEF_KEEPALIVE * configTICK_RATE_HZ);
                        vMQTTSN.Nretry = MQTTSN_DEF_NRETRY;
                    }
                }
            }
            break;
        // SubAck answer
        case MQTTSN_MSGTYP_SUBACK:
            if(msg_from_gw)
            {
                if(vMQTTSN.Status == MQTTSN_STATUS_PRE_CONNECT)
                {
                    if((vMQTTSN.MsgBuf.MsgType == MQTTSN_MSGTYP_SUBSCRIBE) &&
                       (vMQTTSN.MsgBuf.MsgId == ((pPHY1outBuf->mq.suback.MsgId[0]<<8) |
                                                  pPHY1outBuf->mq.suback.MsgId[1])))
                    {
                        vMQTTSN.Status = MQTTSN_STATUS_CONNECT;
                        // Send Device Info
                        MQTTSN_Send(MQTTSN_MSGTYP_PUBLISH,
                                   (MQTTSN_FL_QOS1 | MQTTSN_FL_TOPICID_PREDEF),
                                   objDeviceTyp);
                    }
                }
            }
            break;
        // Ping Response
        case MQTTSN_MSGTYP_PINGRESP:
            if(vMQTTSN.Status == MQTTSN_STATUS_CONNECT)
            {
                if(msg_from_gw)
                {
                    vMQTTSN.Tretry = (MQTTSN_DEF_KEEPALIVE * configTICK_RATE_HZ);
                    vMQTTSN.Nretry = MQTTSN_DEF_NRETRY;
                }
            }
            break;
        // Disconnect Request
        case MQTTSN_MSGTYP_DISCONNECT:
            if(vMQTTSN.Status < MQTTSN_STATUS_OFFLINE)
            {
                vMQTTSN.Radius = 1;
                vMQTTSN.Tretry = 0;
                vMQTTSN.Nretry = MQTTSN_DEF_NRETRY;
            }
            else if(msg_from_gw)
            {
                vMQTTSN.Status = MQTTSN_STATUS_DISCONNECTED;
                vMQTTSN.Tretry = 0;
                vMQTTSN.Nretry = MQTTSN_DEF_NRETRY;
            }
            break;
#ifdef MQTTSN_USE_DHCP
        // NOT STANDARD MESSAGE, DON'T USE WITH ANOTHER SYSTEMS
        // DHCP request from another node
        case MQTTSN_MSGTYP_DHCPREQ:
            if(vMQTTSN.Status == MQTTSN_STATUS_DHCP)
                vMQTTSN.Tretry = mqttsn_get_random_delay(MQTTSN_DEF_TSGW  * configTICK_RATE_HZ) + configTICK_RATE_HZ;
            break;
        // DHCP Response
        case MQTTSN_MSGTYP_DHCPRESP:
            if(vMQTTSN.Status == MQTTSN_STATUS_DHCP)
            {
                if(memcmp(pPHY1outBuf->mq.dhcpresp.MsgId, &vMQTTSN.MsgId, sizeof(vMQTTSN.MsgId)) == 0)  // Own message
                {
                    uint8_t Mask = 0;
                    uint8_t * pData = pPHY1outBuf->mq.dhcpresp.addr;
                    uint8_t Length = MQTTSN_SIZEOF_MSG_DHCPRESP;

                    if(memcmp(vMQTTSN.phy1addr, &addr1_undef, sizeof(PHY1_ADDR_t)) == 0)
                    {
                        Length += sizeof(PHY1_ADDR_t);
                        Mask = 1; 
                    }
#ifdef PHY2_ADDR_t
                    if(memcmp(vMQTTSN.phy2addr, &addr2_undef, sizeof(PHY2_ADDR_t))== 0)
                    {
                        Length += sizeof(PHY2_ADDR_t);
                        Mask |= 2; 
                    }
#endif  //  PHY2_ADDR_t
                    if(pPHY1outBuf->mq.Length != Length)
                        break;

                    if(Mask & 1)
                    {
                        // Check, own address != Gateway Address
                        if(memcmp(pPHY1outBuf->phy1addr, pData, sizeof(PHY1_ADDR_t)) == 0)
                            break;
                            
                        memcpy(vMQTTSN.phy1addr, pData, sizeof(PHY1_ADDR_t));
                        pData += sizeof(PHY1_ADDR_t);
                        WriteOD(PHY1_NodeId, MQTTSN_FL_TOPICID_PREDEF, sizeof(PHY1_ADDR_t), (uint8_t *)vMQTTSN.phy1addr);
                        PHY1_Init();
                    }
#ifdef PHY2_ADDR_t
                    if(Mask & 2)
                    {
                        memcpy(vMQTTSN.phy2addr, pData, sizeof(PHY2_ADDR_t));
                        WriteOD(PHY2_NodeId, MQTTSN_FL_TOPICID_PREDEF, sizeof(PHY2_ADDR_t), (uint8_t *)vMQTTSN.phy2addr);
                        PHY2_Init();
                    }
#endif  //  PHY2_ADDR_t

                    memcpy(vMQTTSN.GatewayAddr, pPHY1outBuf->phy1addr, sizeof(PHY1_ADDR_t));
                    vMQTTSN.GwId = pPHY1outBuf->mq.dhcpresp.GwId;
                    vMQTTSN.Status = MQTTSN_STATUS_OFFLINE;
                    vMQTTSN.Tretry = mqttsn_get_random_delay(MQTTSN_DEF_TCONNECT * configTICK_RATE_HZ);
                    vMQTTSN.Nretry = MQTTSN_DEF_NRETRY;
                    vMQTTSN.MsgId = 0;
                }
            }
            break;
#endif  //  MQTTSN_USE_DHCP
        // Forward message
#if ((defined MQTTSN_TRANSIT_ON_PHY1) || (defined PHY2_Send))
        case MQTTSN_MSGTYP_FORWARD:
            if(vMQTTSN.Status == MQTTSN_STATUS_CONNECT)
            {
                if(msg_from_gw)   // message from Gateway to Node
                {
                    uint8_t Length = pPHY1outBuf->mq.Length;
                    uint8_t phynr = pPHY1outBuf->mq.forward.wNodeID[0];
#ifdef MQTTSN_TRANSIT_ON_PHY1
                    // Direction: Gateway to Remote node on PHY1
                    if((phynr == 1) && (Length == (MQTTSN_SIZEOF_MSG_FORWARD + sizeof(PHY1_ADDR_t) + 1)))
                    {
                        memcpy(pPHY1outBuf->phy1addr, &pPHY1outBuf->mq.forward.wNodeID[1], sizeof(PHY1_ADDR_t));
                        // truncate header
                        pPHY1outBuf->Length -= Length;
                        memcpy(&pPHY1outBuf->raw[0], &pPHY1outBuf->raw[Length], pPHY1outBuf->Length);
                        PHY1_Send(pPHY1outBuf);
                        return;
                    }
#endif  //  MQTTSN_TRANSIT_ON_PHY1
#ifdef PHY2_Send
                    // Direction Gateway to PHY2
                    if((phynr == 2) && (Length == (MQTTSN_SIZEOF_MSG_FORWARD + sizeof(PHY2_ADDR_t) + 1)))
                    {
                        memcpy(pPHY1outBuf->phy2addr, &pPHY1outBuf->mq.forward.wNodeID[1], sizeof(PHY2_ADDR_t));
                        // truncate header
                        pPHY1outBuf->Length -= Length;
                        memcpy(&pPHY1outBuf->raw[0], &pPHY1outBuf->raw[Length], pPHY1outBuf->Length);
                        PHY2_Send(pPHY1outBuf);
                        return;
                    }
#endif  //  PHY2_Send
                }
            }
            break;
#endif  //  ((defined MQTTSN_TRANSIT_ON_PHY1) || (defined PHY2_Send))
        // Unknown message type
        default:
            break;
    }
    mqFree(pPHY1outBuf);
}

// Parse Incoming messages from PHY2
#ifdef PHY2_Get
void mqttsn_parser_phy2(MQ_t * pPHY2outBuf)
{
    if(vMQTTSN.Status == MQTTSN_STATUS_CONNECT)
    {
        if(pPHY2outBuf->mq.MsgType == MQTTSN_MSGTYP_SEARCHGW)
        {
            if(pPHY2outBuf->mq.searchgw.Radius == vMQTTSN.Radius)
            {
                // Send Gateway Info message
                PHY2_ADDR_t s_addr = ADDR_BROADCAST_PHY2;
                memcpy(pPHY2outBuf->phy2addr, &s_addr, sizeof(PHY2_ADDR_t));
                uint8_t Length = MQTTSN_SIZEOF_MSG_GWINFO;

                pPHY2outBuf->mq.MsgType = MQTTSN_MSGTYP_GWINFO;
                pPHY2outBuf->mq.gwinfo.GwId = vMQTTSN.GwId;
                if(vMQTTSN.Radius > 1)
                {
                    memcpy(pPHY2outBuf->mq.gwinfo.GwAdd, vMQTTSN.phy2addr, sizeof(PHY2_ADDR_t));
                    Length += sizeof(PHY2_ADDR_t);
                }

                pPHY2outBuf->Length = Length;
                pPHY2outBuf->mq.Length = Length;
                PHY2_Send(pPHY2outBuf);
                return;
            }
        }
        else  // Encapulate message to Forward Packet and send to Gateway
        {
            uint8_t Length = pPHY2outBuf->Length;
            uint8_t Size = (MQTTSN_SIZEOF_MSG_FORWARD + sizeof(PHY2_ADDR_t) + 1);
            uint8_t pos;
            if((Length + Size) <= sizeof(MQTTSN_MESSAGE_t))
            {
                for(pos = (Size + Length - 1); pos >= Size; pos--)
                    pPHY2outBuf->raw[pos] = pPHY2outBuf->raw[pos - Size];

                // Make forward message
                pPHY2outBuf->Length += Size;
                pPHY2outBuf->mq.Length = Size;
                pPHY2outBuf->mq.MsgType = MQTTSN_MSGTYP_FORWARD;
                pPHY2outBuf->mq.forward.Ctrl = 0;   // ?? TTL
                pPHY2outBuf->mq.forward.wNodeID[0] = 2;     // PHY2
                memcpy(&pPHY2outBuf->mq.forward.wNodeID[1], pPHY2outBuf->phy2addr, sizeof(PHY2_ADDR_t));
                memcpy(pPHY2outBuf->phy1addr, vMQTTSN.GatewayAddr, sizeof(PHY1_ADDR_t));
                PHY1_Send(pPHY2outBuf);
                return;
            }
        }
    }
    mqFree(pPHY2outBuf);
}
#endif  //  PHY2_Get

// End parse incoming messages
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
// Start Poll Task

// Build Name
static uint8_t mqttsn_build_node_name(uint8_t * pBuf)
{
    uint8_t Length = MQTTSN_SIZEOF_CLIENTID;
    ReadOD(objNodeName, MQTTSN_FL_TOPICID_PREDEF, &Length, pBuf);
    if(Length > 1)
        return Length;

    // Node Name not defined, use default name
    uint8_t pos, ch;
    Length = OD_DEV_TYP_LEN;
    ReadOD(objDeviceTyp, MQTTSN_FL_TOPICID_PREDEF, &Length, pBuf);
    pBuf += Length;
    *(pBuf++) = '_';
    Length++;

    for(pos = 0; pos < sizeof(PHY1_ADDR_t); pos++)
    {
        ch = vMQTTSN.phy1addr[pos]>>4;
        if(ch > 0x09)
            ch += 0x37;
        else
            ch += 0x30;
        *pBuf = ch;
        pBuf++;
    
        ch = vMQTTSN.phy1addr[pos] & 0x0F;
        if(ch > 0x09)
            ch += 0x37;
        else
            ch += 0x30;
        *pBuf = ch;
        pBuf++;
    }
    Length += (sizeof(PHY1_ADDR_t)*2);

    return Length;
}

void MQTTSN_Poll(void)
{
    MQ_t * pMessage;
    
    if(vMQTTSN.Tretry > 0)
    {
        vMQTTSN.Tretry--;
        if(vMQTTSN.Tretry != 0)
            return;
    }

    switch(vMQTTSN.Status)
    {
        case MQTTSN_STATUS_DISCONNECTED:
            {
            uint8_t uTmp = sizeof(PHY1_ADDR_t);
            ReadOD(PHY1_GateId, MQTTSN_FL_TOPICID_PREDEF, &uTmp, (uint8_t *)vMQTTSN.GatewayAddr);
            ReadOD(PHY1_NodeId, MQTTSN_FL_TOPICID_PREDEF, &uTmp, (uint8_t *)vMQTTSN.phy1addr);
#ifdef PHY2_ADDR_t
            uTmp = sizeof(PHY2_ADDR_t);
            ReadOD(PHY2_NodeId, MQTTSN_FL_TOPICID_PREDEF, &uTmp, (uint8_t *)vMQTTSN.phy2addr);
#endif  //  PHY2_ADDR_t
#ifdef MQTTSN_USE_DHCP
            if(memcmp(vMQTTSN.phy1addr, &addr1_undef, sizeof(PHY1_ADDR_t)) == 0)
                vMQTTSN.Status = MQTTSN_STATUS_DHCP;
            else
#ifdef PHY2_ADDR_t
            if(memcmp(vMQTTSN.phy2addr, &addr2_undef, sizeof(PHY2_ADDR_t)) == 0)
                vMQTTSN.Status = MQTTSN_STATUS_DHCP;
            else
#endif  //  PHY2_ADDR_t
#endif  //  MQTTSN_USE_DHCP
            if(memcmp(vMQTTSN.GatewayAddr, &addr1_undef, sizeof(PHY1_ADDR_t)) == 0)
                vMQTTSN.Status = MQTTSN_STATUS_SEARCHGW;
            else
                vMQTTSN.Status = MQTTSN_STATUS_OFFLINE;

            vMQTTSN.GwId = 0;
            vMQTTSN.Radius = 1;
            vMQTTSN.Tretry = 0;
            vMQTTSN.Nretry = MQTTSN_DEF_NRETRY;
            vMQTTSN.MsgBuf.MsgType = MQTTSN_MSGTYP_PINGREQ;
            vMQTTSN.MsgId = 0;
            }
            break;
#ifdef MQTTSN_USE_DHCP
        case MQTTSN_STATUS_DHCP:
            {
            if(vMQTTSN.Nretry > 0)
                vMQTTSN.Nretry--;
            else
            {
                if(vMQTTSN.Radius < MQTTSN_MAX_RADIUS)
                {
                    vMQTTSN.Radius++;
                    vMQTTSN.Nretry = (MQTTSN_DEF_NRETRY - 1);
                }
                else
                {
                    vMQTTSN.Tretry = (MQTTSN_DEF_TDISCONNECT  * configTICK_RATE_HZ);
                    vMQTTSN.Status = MQTTSN_STATUS_DISCONNECTED;
                    break;
                }
            }

            vMQTTSN.Tretry = mqttsn_get_random_delay(MQTTSN_DEF_TSGW  * configTICK_RATE_HZ);

            pMessage = mqAlloc(sizeof(MQ_t));
            if(pMessage == NULL)
                break;

            vMQTTSN.MsgId = mqttsn_get_random();

            uint8_t Length = 0;
            if(memcmp(vMQTTSN.phy1addr, &addr1_undef, sizeof(PHY1_ADDR_t)) == 0)
                pMessage->mq.dhcpreq.hlen[Length++] = sizeof(PHY1_ADDR_t);
#ifdef PHY2_ADDR_t
            if(memcmp(vMQTTSN.phy2addr, &addr2_undef, sizeof(PHY2_ADDR_t))== 0)
                pMessage->mq.dhcpreq.hlen[Length++] = sizeof(PHY2_ADDR_t);
#endif  //  PHY2_ADDR_t

            memcpy(pMessage->phy1addr, &addr1_broad, sizeof(PHY1_ADDR_t));

            pMessage->Length = MQTTSN_SIZEOF_MSG_DHCPREQ + Length;
            pMessage->mq.Length = MQTTSN_SIZEOF_MSG_DHCPREQ + Length;
    
            pMessage->mq.MsgType = MQTTSN_MSGTYP_DHCPREQ;
            pMessage->mq.dhcpreq.Radius = vMQTTSN.Radius;
            memcpy(pMessage->mq.dhcpreq.MsgId, &vMQTTSN.MsgId, sizeof(vMQTTSN.MsgId));

            PHY1_Send(pMessage);
            }
            break;
#endif  //  MQTTSN_USE_DHCP
        case MQTTSN_STATUS_SEARCHGW:
            {
            if(vMQTTSN.Nretry > 0)
                vMQTTSN.Nretry--;
            else
            {
                if(vMQTTSN.Radius < MQTTSN_MAX_RADIUS)
                {
                    vMQTTSN.Radius++;
                    vMQTTSN.Nretry = MQTTSN_DEF_NRETRY;
                }
                else
                {
                    vMQTTSN.Tretry = (MQTTSN_DEF_TDISCONNECT  * configTICK_RATE_HZ);
                    vMQTTSN.Status = MQTTSN_STATUS_DISCONNECTED;
                    break;
                }
            }
            vMQTTSN.Tretry = mqttsn_get_random_delay(MQTTSN_DEF_TSGW  * configTICK_RATE_HZ);

            pMessage = mqAlloc(sizeof(MQ_t));
            if(pMessage == NULL)
                break;

            memcpy(pMessage->phy1addr, &addr1_broad, sizeof(PHY1_ADDR_t));
            pMessage->Length = MQTTSN_SIZEOF_MSG_SEARCHGW;
            pMessage->mq.Length = MQTTSN_SIZEOF_MSG_SEARCHGW;
            pMessage->mq.MsgType = MQTTSN_MSGTYP_SEARCHGW;
            pMessage->mq.searchgw.Radius = vMQTTSN.Radius;

            PHY1_Send(pMessage);
            }
            break;
        case MQTTSN_STATUS_OFFLINE:
            {
            if(vMQTTSN.Nretry > 0)
                vMQTTSN.Nretry--;
            else
            {
                vMQTTSN.Radius = 0;
                vMQTTSN.Tretry = (MQTTSN_DEF_TSGW  * configTICK_RATE_HZ);
                vMQTTSN.Nretry = MQTTSN_DEF_NRETRY;
                vMQTTSN.Status = MQTTSN_STATUS_SEARCHGW;
                break;
            }
            vMQTTSN.Tretry = mqttsn_get_random_delay(MQTTSN_DEF_TCONNECT * configTICK_RATE_HZ);

            pMessage = mqAlloc(sizeof(MQ_t));
            if(pMessage == NULL)
                break;

            pMessage->mq.MsgType = MQTTSN_MSGTYP_CONNECT;
            pMessage->mq.connect.Flags = MQTTSN_FL_CLEANSESSION;
            pMessage->mq.connect.ProtocolId = MQTTSN_DEF_PROTOCOLID;
            pMessage->mq.connect.Duration[0] = (MQTTSN_DEF_KEEPALIVE>>8);
            pMessage->mq.connect.Duration[1] = (MQTTSN_DEF_KEEPALIVE & 0xFF);

            uint8_t Length = mqttsn_build_node_name((uint8_t *)&pMessage->mq.connect.ClientId);

            Length += MQTTSN_SIZEOF_MSG_CONNECT;
            pMessage->Length = Length;
            pMessage->mq.Length = Length;
            memcpy(pMessage->phy1addr, vMQTTSN.GatewayAddr, sizeof(PHY1_ADDR_t));
            PHY1_Send(pMessage);
            }
            break;
        case MQTTSN_STATUS_PRE_CONNECT:
        case MQTTSN_STATUS_CONNECT:
            {
            if(vMQTTSN.Nretry > 0)
                vMQTTSN.Nretry--;
            else
            {
                vMQTTSN.Status = MQTTSN_STATUS_DISCONNECTED;
                break;
            }

            uint8_t Length;
            pMessage = mqAlloc(sizeof(MQ_t));
            if(pMessage == NULL)
                break;

            switch(vMQTTSN.MsgBuf.MsgType)
            {
                // Make Publish message
                case MQTTSN_MSGTYP_PUBLISH:
                    {
                    pMessage->mq.MsgType = MQTTSN_MSGTYP_PUBLISH;
                    pMessage->mq.publish.Flags = vMQTTSN.MsgBuf.Flags;
                    pMessage->mq.publish.TopicId[0] = vMQTTSN.MsgBuf.TopicId>>8;
                    pMessage->mq.publish.TopicId[1] = vMQTTSN.MsgBuf.TopicId & 0xFF;
                    pMessage->mq.publish.MsgId[0] = vMQTTSN.MsgBuf.MsgId>>8;
                    pMessage->mq.publish.MsgId[1] = vMQTTSN.MsgBuf.MsgId & 0xFF;
                    Length = (MQTTSN_MSG_SIZE - 5);
                    ReadODpack(vMQTTSN.MsgBuf.TopicId, vMQTTSN.MsgBuf.Flags, &Length, pMessage->mq.publish.Data);
                    Length += MQTTSN_SIZEOF_MSG_PUBLISH;

                    // ToDo QoS2 not supported
                    if((vMQTTSN.MsgBuf.Flags & MQTTSN_FL_QOS_MASK) != MQTTSN_FL_QOS1)
                        vMQTTSN.MsgBuf.MsgType = MQTTSN_MSGTYP_PINGREQ;
                    else
                    {
                        vMQTTSN.Tretry = mqttsn_get_random_delay(configTICK_RATE_HZ/8) + (configTICK_RATE_HZ/8);
                        vMQTTSN.MsgBuf.Flags |= MQTTSN_FL_DUP;
                    }
                    }
                    break;
                // Make Register message
                case MQTTSN_MSGTYP_REGISTER:
                    {
                    pMessage->mq.MsgType = MQTTSN_MSGTYP_REGISTER;
                    pMessage->mq.regist.TopicId[0] = vMQTTSN.MsgBuf.TopicId>>8;
                    pMessage->mq.regist.TopicId[1] = vMQTTSN.MsgBuf.TopicId & 0xFF;
                    pMessage->mq.regist.MsgId[0] = vMQTTSN.MsgBuf.MsgId>>8;
                    pMessage->mq.regist.MsgId[1] = vMQTTSN.MsgBuf.MsgId & 0xFF;
                    Length = MakeTopicName(vMQTTSN.MsgBuf.Flags, pMessage->mq.regist.TopicName);
                    Length += MQTTSN_SIZEOF_MSG_REGISTER;

                    vMQTTSN.Tretry = mqttsn_get_random_delay(configTICK_RATE_HZ/8) + (configTICK_RATE_HZ/8);
                    }
                    break;
                // Make Subscribe message
                case MQTTSN_MSGTYP_SUBSCRIBE:
                    {
                    pMessage->mq.MsgType = MQTTSN_MSGTYP_SUBSCRIBE;
                    pMessage->mq.subscribe.Flags = vMQTTSN.MsgBuf.Flags;
                    pMessage->mq.subscribe.MsgId[0] = vMQTTSN.MsgBuf.MsgId>>8;
                    pMessage->mq.subscribe.MsgId[1] = vMQTTSN.MsgBuf.MsgId & 0xFF;
                    pMessage->mq.subscribe.Topic[0] = '#';
                    Length = (MQTTSN_SIZEOF_MSG_SUBSCRIBE + 1);
        
                    vMQTTSN.Tretry = mqttsn_get_random_delay(configTICK_RATE_HZ/8) + (configTICK_RATE_HZ/8);
                    vMQTTSN.MsgBuf.Flags |= MQTTSN_FL_DUP;
                    }
                    break;
                // No messages, send PingReq
                default:
                    {
                    vMQTTSN.MsgBuf.MsgType = MQTTSN_MSGTYP_PINGREQ;
                    Length = MQTTSN_SIZEOF_MSG_PINGREQ;
                    pMessage->mq.MsgType = MQTTSN_MSGTYP_PINGREQ;
                    vMQTTSN.Tretry = (MQTTSN_DEF_KEEPALIVE * configTICK_RATE_HZ);
                    }
                    break;
            }
            pMessage->Length = Length;
            pMessage->mq.Length = Length;
            memcpy(pMessage->phy1addr, vMQTTSN.GatewayAddr, sizeof(PHY1_ADDR_t));
            PHY1_Send(pMessage);                
            }
            break;
        default:
            break;
    }
}

// End Poll Task
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
// API

// Initialise MQTTSN tasks
void MQTTSN_Init(void)
{
    vMQTTSN.Status = MQTTSN_STATUS_DISCONNECTED;
    vMQTTSN.Tretry = configTICK_RATE_HZ;

    //xTaskCreate(mqttsn_poll_task, "poll", configMINIMAL_STACK_SIZE + 20, NULL, tskIDLE_PRIORITY + 1, NULL );
}

// Get MQTTSN Status
e_MQTTSN_STATUS_t MQTTSN_GetStatus(void)
{
    return vMQTTSN.Status;
}

bool MQTTSN_CanSend(void)
{
    return (vMQTTSN.MsgBuf.MsgType == MQTTSN_MSGTYP_PINGREQ);
}

void MQTTSN_Send(e_MQTTSN_MSGTYPE_t      MsgType,
                 uint8_t                 Flags,
                 uint16_t                TopicId)
{
    vMQTTSN.MsgBuf.MsgType  = MsgType;
    vMQTTSN.MsgBuf.Flags    = Flags;
    vMQTTSN.MsgBuf.TopicId  = TopicId;
    vMQTTSN.MsgBuf.MsgId    = mqttsn_new_msgid();
    
    vMQTTSN.Tretry = 0;
    vMQTTSN.Nretry = MQTTSN_DEF_NRETRY;
}
