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

///////////////////////////////////////////////////////////////
// Configuration settings

#define OD_DEFAULT_GROUP        0x2DD4

// 433 MHz
#if (RF_BASE_FREQ > 433050000UL) && (RF_BASE_FREQ < 434790000UL)
#define RFM12_BAND          RFM12_BAND_433
#define OD_DEFAULT_CHANNEL  ((RF_BASE_FREQ - 433000000UL)/25000)
// 868 MHz
#elif (RF_BASE_FREQ > 868000000UL) && (RF_BASE_FREQ < 870000000UL)
#define RFM12_BAND          RFM12_BAND_868
#define OD_DEFAULT_CHANNEL  ((RF_BASE_FREQ - 868000000UL)/25000)
// 915 MHz
#elif (RF_BASE_FREQ > 902000000UL) && (RF_BASE_FREQ < 928000000UL)
#define RFM12_BAND          RFM12_BAND_915
#define OD_DEFAULT_CHANNEL  ((RF_BASE_FREQ - 902000000UL)/25000)
#else
#error  RF_BASE_FREQ does not belond to ISM band
#endif  // RF_BASE_FREQ

// API Section
void RFM12_Init(void);
void RFM12_Send(void *pBuf);
void * RFM12_Get(void);
void * RFM12_GetAddr(void);

#endif  //  _RFM12_PHY_H
