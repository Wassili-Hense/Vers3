/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// TWI Bus interface

#include "../../config.h"

#ifdef TWIMM_PHY

static Queue_t  twimm_tx_queue = {NULL, NULL};

void hal_twimm_init_hw(uint8_t addr);
bool hal_twimm_can_send(void);
void hal_twimm_send(MQ_t *pBuf);

static void twimm_tx_task(void)
{
    if(!hal_twimm_can_send())
        return;

    MQ_t * pTx_buf = mqDequeue(&twimm_tx_queue);
    if(pTx_buf == NULL)
        return;
        
    hal_twimm_send(pTx_buf);
    mqFree(pTx_buf);
}

void TWIMM_Init(void)
{
    // Load Device ID
    uint8_t Len = sizeof(uint8_t);
    uint8_t Addr;
    ReadOD(objRFNodeId, MQTTSN_FL_TOPICID_PREDEF,  &Len, &Addr);

    hal_twimm_init_hw(Addr);
}

void TWIMM_Send(void *pBuf)
{
    if(!mqEnqueue(&twimm_tx_queue, pBuf))
        mqFree(pBuf);
    else
        twimm_tx_task();
}

void * TWIMM_Get(void)
{
    twimm_tx_task();

    return NULL;
}

#endif  //  TWIMM_PHY