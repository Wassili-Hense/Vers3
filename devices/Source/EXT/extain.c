/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// Extensions analogue inputs

#include "../config.h"

#ifdef EXTAIN_USED

#include "extain.h"

static uint8_t ain_ref[EXTAIN_MAXPORT_NR];

static uint8_t ainCheckAnalogBase(uint16_t base)
{
    if(base >= EXTAIN_MAXPORT_NR)
        return 2;
        
    if(ain_ref[(uint8_t)(base & 0xFF)] != 0xFF)
        return 1;

    return 0;
}

static uint8_t ainSubidx2Ref(subidx_t * pSubidx)
{
    switch(pSubidx->Type)
    {
#if ((EXTAIN_REF & 0x01) != 0)
        case objArefExt:
            return 0;
#endif
#if ((EXTAIN_REF & 0x02) != 0)
        case objArefVcc:
            return 1;
#endif
#if ((EXTAIN_REF & 0x04) != 0)
        case objArefInt1:
            return 2;
#endif
#if ((EXTAIN_REF & 0x08) != 0)
        case objArefInt2:
            return 3;
#endif
        default:
            return 0xFF;
    }
}

// Check Index analogue inputs
uint8_t ainCheckIdx(subidx_t * pSubidx)
{
    if(ainSubidx2Ref(pSubidx) == 0xFF)
        return 2;

    return ainCheckAnalogBase(pSubidx->Base);
}

void ainInit()
{
    uint8_t apin;
    for(apin = 0; apin < EXTAIN_MAXPORT_NR; apin++)
    {
        ain_ref[apin] = 0xFF;
    }
}

/*
// Read digital Inputs
e_MQTTSN_RETURNS_t dioReadOD(subidx_t * pSubidx, uint8_t *pLen, uint8_t *pBuf)
{
    uint16_t base = pSubidx->Base;
    DIO_PORT_TYPE state = dio_status[dioBase2Port(base)];
    DIO_PORT_TYPE mask = dioBase2Mask(base);
    dio_change_flag[dioBase2Port(base)] &= ~mask;
  
    if(pSubidx->Type == objPinNPN)
        state = ~state;
    *pLen = 1;
    *pBuf = ((state & mask) != 0) ? 1 : 0;
    return MQTTSN_RET_ACCEPTED;
}

// Write DIO Object's
e_MQTTSN_RETURNS_t dioWriteOD(subidx_t * pSubidx, uint8_t Len, uint8_t *pBuf)
{
    uint16_t base = pSubidx->Base;
    uint8_t state = *pBuf;
    uint8_t port = dioBase2Port(base);
    DIO_PORT_TYPE mask = dioBase2Mask(base);
    dio_change_flag[dioBase2Port(base)] &= ~mask;

    if(pSubidx->Type == objPinNPN)
        state = ~state;

    if(state & 1)
        dio_status[port] |= mask;
    else
        dio_status[port] &= ~mask;

    return MQTTSN_RET_ACCEPTED;
}

// Poll Procedure
uint8_t dioPollOD(subidx_t * pSubidx, uint8_t sleep)
{
    uint16_t base = pSubidx->Base;
    return ((dio_change_flag[dioBase2Port(base)] & dioBase2Mask(base)) != 0) ? 1 : 0;
}
*/

// Register analogue input
e_MQTTSN_RETURNS_t ainRegisterOD(indextable_t *pIdx)
{
    uint16_t base = pIdx->sidx.Base;
    if(ainCheckAnalogBase(base) != 0)
        return MQTTSN_RET_REJ_INV_ID;

    uint8_t apin = (uint8_t)(base & 0xFF);

    ain_ref[apin] = ainSubidx2Ref(&pIdx->sidx);

    pIdx->cbRead  = NULL;
    pIdx->cbWrite = NULL;
    pIdx->cbPoll  = NULL;

    return MQTTSN_RET_ACCEPTED;
}

void ainDeleteOD(subidx_t * pSubidx)
{
    uint16_t base = pSubidx->Base;
    if(ainCheckAnalogBase(base) != 1)
        return;

    ain_ref[(uint8_t)(base & 0xFF)] = 0xFF;
}

void ainProc(void)
{
}
#endif    //  EXTAIN_USED
