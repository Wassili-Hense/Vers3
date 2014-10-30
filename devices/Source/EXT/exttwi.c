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

#ifdef EXTTWI_BLINKM
#include "TWI/twi_blinkm.h"
#endif  //  EXTTWI_BLINKM


e_MQTTSN_RETURNS_t twiReadOD(subidx_t * pSubidx, uint8_t *pLen, uint8_t *pBuf)
{
    *pLen = 1;
    pBuf[0] = (uint8_t)(0xAA);
    return MQTTSN_RET_ACCEPTED;
}

e_MQTTSN_RETURNS_t twiWriteOD(subidx_t * pSubidx, uint8_t Len, uint8_t *pBuf)
{
    return MQTTSN_RET_ACCEPTED;
}

uint8_t twiPollOD(subidx_t * pSubidx, uint8_t sleep)
{
    return 0;
}

void twiInit()
{
    if(!hal_twi_configure(1))       // Enable
        return;

}

void twiProc(void)
{
}
#endif    //  EXTTWI_USED
