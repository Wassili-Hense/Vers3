/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// Extensions TWI/I2C

#include "../config.h"

#ifdef EXTTWI_USED

#include "exttwi.h"

typedef struct sTWI_QUEUE
{
    struct  sTWI_QUEUE * pNext;
    TWI_FRAME_t frame;
}TWI_QUEUE_t;

// Global variable used in HAL
volatile TWI_FRAME_t twi_exchange;

// local queues
static Queue_t  twi_tx_queue = {NULL, NULL, 0, 0};

// HAL
bool hal_twi_configure(uint8_t enable);
void hal_twi_tick(void);

e_MQTTSN_RETURNS_t twiReadOD(subidx_t * pSubidx, uint8_t *pLen, uint8_t *pBuf)
{
    *pLen = twi_exchange.read + sizeof(TWI_FRAME_t);
    memcpy(pBuf, (void *)&twi_exchange, *pLen);
    twi_exchange.access = 0;
    return MQTTSN_RET_ACCEPTED;
}

e_MQTTSN_RETURNS_t twiWriteOD(subidx_t * pSubidx, uint8_t Len, uint8_t *pBuf)
{
    if(Len < sizeof(TWI_FRAME_t))
        return MQTTSN_RET_REJ_NOT_SUPP;

    TWI_QUEUE_t * pQueue = mqAlloc(sizeof(MQ_t));
    if(pQueue == NULL)
        return MQTTSN_RET_REJ_CONG;

    memcpy(&pQueue->frame, pBuf, Len);

    if(((pQueue->frame.access & 0xFC) != 0) ||
        (pQueue->frame.write != (Len - sizeof(TWI_FRAME_t))) || 
        (pQueue->frame.read > (MQTTSN_MSG_SIZE - sizeof(TWI_FRAME_t) - MQTTSN_SIZEOF_MSG_PUBLISH)))
    {
        mqFree(pQueue);
        return MQTTSN_RET_REJ_NOT_SUPP;
    }

    if(!mqEnqueue(&twi_tx_queue, pQueue))
    {
        mqFree(pQueue);
        return MQTTSN_RET_REJ_CONG;
    }

    return MQTTSN_RET_ACCEPTED;
}

uint8_t twiPollOD(subidx_t * pSubidx, uint8_t sleep)
{
    if(twi_exchange.access != 0)
    {
        if((twi_exchange.access & 0xF0) != 0)
            return 1;
        else
            hal_twi_tick();
    }
    else if(twi_tx_queue.Size != 0)
    {
        TWI_QUEUE_t * pTwi = mqDequeue(&twi_tx_queue);
        if(pTwi != NULL)
        {
            memcpy((void *)&twi_exchange, &pTwi->frame, sizeof(MQ_t));
            mqFree(pTwi);
        }
    }

    return 0;
}

void twiInit()
{
    if(!hal_twi_configure(1))           // Enable
        return;

    twi_exchange.access = 0;

    // Register variable Ta0
    indextable_t * pIndex = getFreeIdxOD();
    if(pIndex == NULL)
    {
        hal_twi_configure(0);
        return;
    }

    pIndex->cbRead     = &twiReadOD;
    pIndex->cbWrite    = &twiWriteOD;
    pIndex->cbPoll     = &twiPollOD;
    pIndex->sidx.Place = objTWI;        // TWI object
    pIndex->sidx.Type  = objArray;      // Variable Type -  Byte Array
    pIndex->sidx.Base  = 0;             // Device address
}
#endif    //  EXTTWI_USED
