/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _ENC28J60_PHY_H
#define _ENC28J60_PHY_H

#ifdef __cplusplus
extern "C" {
#endif

#define LAN_NODE

// API Section
void ENC28J60_Init(void);
void ENC28J60_Send(void *pBuf);
void * ENC28J60_Get(void);

#ifdef __cplusplus
}
#endif

#endif  //  _UART_PHY_H
