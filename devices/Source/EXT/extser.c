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



#define EXTSER_FLAG_TXEN    1
#define EXTSER_FLAG_RXEN    2
#define EXTSER_FLAG_RXRDY   0x10

typedef struct
{
    uint8_t     nBaud;
    uint8_t     flags;
}EXTSER_VAR_t;

static const uint8_t extser2uart[] = EXTSER_PORT2UART;
static const uint8_t extser2port[] = EXTSER_PORT2DIO;
#define MAX_SER_PORT    (sizeof(extser2uart)/sizeof(extser2uart[0]))
static EXTSER_VAR_t * extSerV[MAX_SER_PORT] = {NULL,};


// HAL Section, subroutines defined in hal_uart.c
void hal_uart_deinit(uint8_t port);

void serInit()
{
    uint8_t port;
    
    for(port = 0; port < MAX_SER_PORT; port++)
    {
        hal_uart_deinit(extser2uart[port]);
        if(extSerV[port] != NULL)
        {
            mqFree(extSerV[port]);
            extSerV[port] = NULL;
        }
    }
}

// Check Index
uint8_t serCheckIdx(subidx_t * pSubidx)
{
    uint8_t type = pSubidx->Type;
    uint8_t port = pSubidx->Base/10;
    uint8_t nBaud = pSubidx->Base % 10;

    if((port >= MAX_SER_PORT) ||
       (nBaud > 4) ||
       ((type != ObjSerRx) && (type != ObjSerTx)))
        return 2;

    if(extSerV[port] != NULL)
        return 1;

    return 0;
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
    uint8_t port = pIdx->sidx.Base / 10;
    uint8_t nBaud = pIdx->sidx.Base % 10;
    
    if(extSerV[port] != NULL)
    {
        if(extSerV[port]->nBaud != nBaud)
            return MQTTSN_RET_REJ_INV_ID;
    }
    else
    {
        extSerV[port] = mqAlloc(sizeof(EXTSER_VAR_t));
        if(extSerV[port] == NULL)
            return MQTTSN_RET_REJ_CONG;
    
        extSerV[port]->nBaud = nBaud;
        extSerV[port]->flags = 0;
    }

    if(pIdx->sidx.Type == ObjSerTx)
    {
        if(extSerV[port]->flags & EXTSER_FLAG_TXEN)
            return MQTTSN_RET_REJ_INV_ID;

        extSerV[port]->flags |= EXTSER_FLAG_TXEN;
    }
    else // ObjSerRx
    {
        if(extSerV[port]->flags & EXTSER_FLAG_RXEN)
            return MQTTSN_RET_REJ_INV_ID;

        extSerV[port]->flags |= EXTSER_FLAG_RXEN;
    }

    return MQTTSN_RET_ACCEPTED;
}

void serDeleteOD(subidx_t * pSubidx)
{
    uint8_t port = pSubidx->Base / 10;

    if(extSerV[port] != NULL)
    {
        if(pSubidx->Type == ObjSerTx)
        {
            extSerV[port]->flags &= ~EXTSER_FLAG_TXEN;
        }
        else
        {
            extSerV[port]->flags &= ~EXTSER_FLAG_RXEN;
        }
        
        if((extSerV[port]->flags & (EXTSER_FLAG_TXEN | EXTSER_FLAG_RXEN)) == 0)
        {
            hal_uart_deinit(extser2uart[port]);
            mqFree(extSerV[port]);
            extSerV[port] = NULL;
        }
    }
}

void serProc(void)
{
    uint8_t port;
    for(port = 0; port < MAX_SER_PORT; port++)
    {
        if(extSerV[port] != NULL)
        {
        }
    }
}
#endif    //  EXTSER_USED
