/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _EXTPLC_H
#define _EXTPLC_H

#ifdef __cplusplus
extern "C" {
#endif

void plcInit(void);
uint8_t plcCheckIdx(subidx_t * pSubidx);
e_MQTTSN_RETURNS_t plcRegisterOD(indextable_t *pIdx);
void plcDeleteOD(subidx_t * pSubidx);
void plcProc(void);

#ifdef __cplusplus
}
#endif

#endif  //  _EXTPLC_H