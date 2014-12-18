/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#include "../config.h"

#ifdef EXTPLC_USED

#include "extplc.h"

indextable_t PLCexchgOD;

static e_MQTTSN_RETURNS_t plcWriteOD(subidx_t * pSubidx, uint8_t Len, uint8_t *pBuf)
{
    return MQTTSN_RET_ACCEPTED;
}

static e_MQTTSN_RETURNS_t merkerWriteOD(subidx_t * pSubidx, uint8_t Len, uint8_t *pBuf)
{
    return MQTTSN_RET_ACCEPTED;
}

void plcInit(void)
{
    PLCexchgOD.Index = 0;
    PLCexchgOD.cbRead = NULL;
    PLCexchgOD.cbWrite = &plcWriteOD;
    PLCexchgOD.cbPoll = NULL;
}

// Check Index
uint8_t plcCheckIdx(subidx_t * pSubidx)
{
    return 0;
}

// Register merker
e_MQTTSN_RETURNS_t plcRegisterOD(indextable_t *pIdx)
{
    pIdx->cbRead  = NULL;
    pIdx->cbWrite = &merkerWriteOD;
    pIdx->cbPoll  = NULL;

    return MQTTSN_RET_ACCEPTED;
}

void plcDeleteOD(subidx_t * pSubidx)
{
}

void plcProc(void)
{
}

#endif  //  EXTPLC_USED
