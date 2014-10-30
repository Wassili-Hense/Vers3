/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _EXTTWI_H
#define _EXTTWI_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*cbTWI)(void);            // TWI Callback function

typedef struct
{
    uint8_t     addr;
    uint8_t     toRd;
    uint8_t     toWr;
    cbTWI       pCb;
    uint8_t     data[];
} TWI_FRAME_t;

void twiInit(void);
void twiProc(void);

#ifdef __cplusplus
}
#endif

#endif  //  _EXTTWI_H
