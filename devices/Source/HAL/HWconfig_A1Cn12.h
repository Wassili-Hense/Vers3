/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef HWCONFIG_A1CN12_H
#define HWCONFIG_A1CN12_H

// uNode Version 2.0
// uc ATMega328p
// Phy1: CC1101

// 0 - 7    PORTA - not exist
// PORTB
// --   PB0     --          LED
// --   PB1     --          RF_IRQ
// --   PB2     --          RF_CSN
// --   PB3     ISP-4       RF_MOSI
// --   PB4     ISP-1       RF_MISO
// --   PB5     ISP-3       RF_SCK
// --   PB6     --          OSC
// --   PB7     --          OSC
// PORT C
// 16   PC0     SV1-18      Ain0
// 17   PC1     SV1-17      Ain1
// 18   PC2     SV1-16      Ain2
// 19   PC3     SV1-15      Ain3
// 20   PC4     SV1-14      SDA
// 21   PC5     SV1-13      SCL
// --   PC6     ISP-5       RESET
// --   --      SV1-20      Ain6
// --   --      SV1-19      Ain7
// PORT D
// 24   PD0     SV1-10      RXD - On gateway busy
// 25   PD1     SV1-9       TXD - On gateway busy
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
#include <util/delay.h>

// DIO Section
#define DIO_PORT_SIZE               8
#define EXTDIO_BASE_OFFSET          2
#define EXTDIO_MAXPORT_NR           2                                     // Number of digital Ports
#define EXTDIO_PORTNUM2PORT         {(uint16_t)&PORTC, (uint16_t)&PORTD}
#define EXTDIO_PORTNUM2MASK         {(uint8_t)0xC0, (uint8_t)0x00}
// End DIO Section

// RF Section
#define TxLEDon()                   PORTB &= ~(1<<PB0);
#define LEDsOff()                   PORTB |= (1<<PB0);
#define LEDsInit()                  DDRB |= (1<<PB0);

#define RF_PORT                     PORTB
#define RF_DDR                      DDRB
#define RF_PIN                      PINB
#define RF_PIN_SS                   PB2
#define RF_PIN_MOSI                 PB3
#define RF_PIN_MISO                 PB4
#define RF_PIN_SCK                  PB5

// RF IRQ
#define RF_IRQ_PORT                 PORTB
#define RF_IRQ_DDR                  DDRB
#define RF_PIN_IRQ                  PB1
#define RF_GET_IRQ()                (PINB & (1<<PB1))
#define RF_IRQ_CFG()                {PCIFR = (1<<PCIF0); PCICR = (1<<PCIE0);}
#define RF_DISABLE_IRQ()            PCMSK0 = 0
#define RF_ENABLE_IRQ()             PCMSK0 = (1<<RF_PIN_IRQ)
#define RF_INT_vect                 PCINT0_vect
//  End RF Section

#define CC11_PHY                    1

#define PHY1_ADDR_t                 uint8_t
#define ADDR_BROADCAST_PHY1         (PHY1_ADDR_t)0x00
#define ADDR_UNDEF_PHY1             (PHY1_ADDR_t)0xFF

// Object's Dictionary Section
#define OD_DEV_UC_TYPE              'A'
#define OD_DEV_UC_SUBTYPE           '1'
#define OD_DEV_PHY1                 'C'
#define OD_DEV_PHY2                 'n'
#define OD_DEV_HW_TYP_H             '1'
#define OD_DEV_HW_TYP_L             '2'

#define OD_ADDR_TYPE                objUInt8

#include "../PHY/CC1101/cc11_phy.h"

#define PHY1_Init                   CC11_Init
#define PHY1_Send                   CC11_Send
#define PHY1_Get                    CC11_Get
#define PHY1_NodeId                 objRFNodeId
#define PHY1_GateId                 objGateID

#ifdef __cplusplus
}
#endif

#endif // HWCONFIG_A1SN12_H