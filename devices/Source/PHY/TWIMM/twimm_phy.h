/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _TWIMM_PHY_H
#define _TWIMM_PHY_H

// API Section
void TWIMM_Init(void);
void TWIMM_Send(void *pBuf);
void * TWIMM_Get(void);

#endif  //  _TWIMM_PHY_H

