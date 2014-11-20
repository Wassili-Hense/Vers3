/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// Extensions, serial port

#include "../config.h"

#ifdef EXTSER_USED

#include "extser.h"

void serInit()
{
}

// Check Index
uint8_t serCheckIdx(subidx_t * pSubidx)
{
    uint8_t type = pSubidx->Type;
    if((pSubidx->Base > SER_MAX_BASE) || ((type != objSerRx) && (type != objSerTx)))
        return MQTTS_RET_REJ_NOT_SUPP;
    return 2;
}

// Read data
e_MQTTSN_RETURNS_t serReadOD(subidx_t * pSubidx, uint8_t *pLen, uint8_t *pBuf)
{
    //uint16_t base = pSubidx->Base;

    *pLen = 1;
    *pBuf = 0;
    return MQTTSN_RET_ACCEPTED;
}

// Write data
e_MQTTSN_RETURNS_t serWriteOD(subidx_t * pSubidx, uint8_t Len, uint8_t *pBuf)
{
    //uint16_t base = pSubidx->Base;
    
    return MQTTSN_RET_ACCEPTED;
}

// Poll Procedure
uint8_t serPollOD(subidx_t * pSubidx, uint8_t sleep)
{
    //uint16_t base = pSubidx->Base;
    return 0;
}

// Register Object
e_MQTTSN_RETURNS_t serRegisterOD(indextable_t *pIdx)
{
    //uint16_t base = pIdx->sidx.Base;
    
    return MQTTSN_RET_ACCEPTED;
}

void serDeleteOD(subidx_t * pSubidx)
{
    //uint16_t base = pSubidx->Base;
}

void serProc(void)
{
}
#endif    //  EXTSER_USED
