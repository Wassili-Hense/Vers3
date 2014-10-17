/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// CC11xx RF interface

#include "../../config.h"

#ifdef CC11_PHY

#include "cc11_hw.h"

#if (CC11_PHY == 1)
#define cc11_adr                phy1addr
#elif (CC11_PHY == 2)
#define cc11_adr                phy2addr
#endif


void CC11_Init(void)
{
    uint8_t NodeID, Channel;
    uint16_t GroupID;

    // Load Device ID
    uint8_t Len = sizeof(uint8_t);
    ReadOD(objRFNodeId, MQTTSN_FL_TOPICID_PREDEF,  &Len, &NodeID);
    // Load Frequency channel
    ReadOD(objRFChannel, MQTTSN_FL_TOPICID_PREDEF, &Len, &Channel);
    // Load Group ID(Synchro)
    Len = sizeof(uint16_t);
    ReadOD(objRFGroup, MQTTSN_FL_TOPICID_PREDEF,  &Len, (uint8_t *)&GroupID);
    cc11_init_hw(NodeID, GroupID, Channel);
}

void CC11_Send(void *pBuf)
{
    mqFree(pBuf);
}


void * CC11_Get(void)
{
    return NULL;
}

#endif  //  CC11_PHY
