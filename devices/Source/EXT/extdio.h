/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _EXTDIO_H
#define _EXTDIO_H

void dioInit(void *pBuf);
uint16_t dioCheckIdx(subidx_t * pSubidx);
e_MQTTSN_RETURNS_t dioRegisterOD(indextable_t *pIdx);
void dioDeleteOD(subidx_t * pSubidx);
void dioProc(void);

#endif  //  _EXTDIO_H
