/*
Copyright (c) 2011-2015 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _RFM12_PHY_H
#define _RFM12_PHY_H

#define OD_DEFAULT_GROUP        0x2DD4

#ifdef RF_ADDR_t
#undef RF_ADDR_t
#warning redefine RF_ADDR_t in rfm12_phy.h
#endif  //  RF_ADDR_t

#ifdef ADDR_UNDEF_RF
#undef ADDR_UNDEF_RF
#warning redefine ADDR_UNDEF_RF in rfm12_phy.h
#endif  //  ADDR_UNDEF_RF

#define RF_ADDR_t               uint8_t
#define ADDR_UNDEF_RF           (RF_ADDR_t)0xFF

// API Section
void RFM12_Init(void);
void RFM12_Send(void *pBuf);
void * RFM12_Get(void);
void * RFM12_GetAddr(void);

#endif  //  _RFM12_PHY_H
