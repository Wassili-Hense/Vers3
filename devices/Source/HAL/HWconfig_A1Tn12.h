/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef HWCONFIG_A1TN12_H
#define HWCONFIG_A1TN12_H

// uc ATMega328p
// PHY1: TWI

// 0 - 7    PORTA - not exist
// PORTB
//  8   PB0     --          LED
//  9   PB1     --          RF_IRQ
// 10   PB2     --          RF_CSN
// 11   PB3     ISP-4       RF_MOSI
// 12   PB4     ISP-1       RF_MISO
// 13   PB5     ISP-3       RF_SCK
// 14   PB6     --          OSC
// 15   PB7     --          OSC
// PORT C
// 16   PC0     SV1-18      Ain0
// 17   PC1     SV1-17      Ain1
// 18   PC2     SV1-16      Ain2
// 19   PC3     SV1-15      Ain3
// 20   PC4     SV1-14      SDA
// 21   PC5     SV1-13      SCL
// 22   PC6     ISP-5       RESET
// --   --      SV1-20      Ain6
// --   --      SV1-19      Ain7
// PORT D
// 24   PD0     SV1-10      RXD
// 25   PD1     SV1-9       TXD
// 26   PD2     SV1-8       IRQ 0 //** RF-IRQ
// 27   PD3     SV1-7       IRQ 1
// 28   PD4     SV1-6
// 29   PD5     SV1-5       PWM0
// 30   PD6     SV1-4       PWM1
// 31   PD7     SV1-3

#ifdef __cplusplus
extern "C" {
#endif

#include "Atmel/hal.h"

// DIO Section
#define DIO_PORT_SIZE               8
#define EXTDIO_BASE_OFFSET          1
#define EXTDIO_MAXPORT_NR           3                                     // Number of digital Ports
#define EXTDIO_PORTNUM2PORT         {(uint16_t)&PORTB, (uint16_t)&PORTC, (uint16_t)&PORTD}
#define EXTDIO_PORTNUM2MASK         {(uint8_t)0xC0, (uint8_t)0xF0, (uint8_t)0x00}
// End DIO Section

// TWIMM Section
#define TWIMM_SCL_STAT()            (PINC & (1<<PC5))
// End TWIMM Section

#define TWIMM_PHY                    1

#define PHY1_ADDR_t                 uint8_t
#define ADDR_BROADCAST_PHY1         (PHY1_ADDR_t)0x00
#define ADDR_UNDEF_PHY1             (PHY1_ADDR_t)0x7F

#define RF_ADDR_t                   uint8_t
#define ADDR_UNDEF_RF               (RF_ADDR_t)0x7F
//#define ADDR_DEFAULT_RF             (RF_ADDR_t)0x05
//#undef  MQTTSN_USE_DHCP

// Object's Dictionary Section
#define OD_DEV_UC_TYPE              'A'
#define OD_DEV_UC_SUBTYPE           '1'
#define OD_DEV_PHY1                 'T'
#define OD_DEV_PHY2                 'n'
#define OD_DEV_HW_TYP_H             '1'
#define OD_DEV_HW_TYP_L             '2'

#define OD_ADDR_TYPE                objUInt8

#include "../PHY/TWIMM/twimm_phy.h"

#define PHY1_Init                   TWIMM_Init
#define PHY1_Send                   TWIMM_Send
#define PHY1_Get                    TWIMM_Get
#define PHY1_NodeId                 objRFNodeId
#define PHY1_GateId                 objGateID

#ifdef __cplusplus
}
#endif

#endif // HWCONFIG_A1TN12_H