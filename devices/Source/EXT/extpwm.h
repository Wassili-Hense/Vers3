/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _EXTPWM_H
#define _EXTPWM_H

#ifdef __cplusplus
extern "C" {
#endif

uint8_t pwmCheckIdx(subidx_t * pSubidx);
e_MQTTSN_RETURNS_t pwmRegisterOD(indextable_t *pIdx);
void pwmDeleteOD(subidx_t * pSubidx);

#ifdef __cplusplus
}
#endif

#endif  //  _EXTPWM_H





