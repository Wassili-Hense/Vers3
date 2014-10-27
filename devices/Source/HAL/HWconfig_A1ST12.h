/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef HWCONFIG_A1ST12_H
#define HWCONFIG_A1ST12_H

// uc ATMega328p
// PHY1: UART
// PHY2: TWI

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
#define EXTDIO_PORTNUM2MASK         {(uint8_t)0xC0, (uint8_t)0xF0, (uint8_t)0x03}
// End DIO Section

// LEDs
#define LED1_On()                   PORTB &= ~(1<<PB0)
#define LED1_Off()                  PORTB |= (1<<PB0)
#define LEDsInit()                  {DDRB |= (1<<PB0); PORTB |= (1<<PB0);}

// UART Section
#define UART_PORT                   PORTD
#define UART_DDR                    DDRD
#define UART_RX_PIN                 PD0
#define UART_TX_PIN                 PD1

#define USART_USE_PORT              1
// End UART Section

// TWIMM Section
#define TWIMM_SCL_STAT()            (PINC & (1<<PC5))
// End TWIMM Section

#define UART_PHY                    1
#define TWIMM_PHY                   2

#define PHY1_ADDR_t                 uint8_t
#define ADDR_BROADCAST_PHY1         (PHY1_ADDR_t)0x00
#define ADDR_UNDEF_PHY1             (PHY1_ADDR_t)0x7F

#define PHY2_ADDR_t                 uint8_t
#define ADDR_BROADCAST_PHY2         (PHY1_ADDR_t)0x00
#define ADDR_UNDEF_PHY2             (PHY1_ADDR_t)0x7F

#define RF_ADDR_t                   uint8_t
#define ADDR_UNDEF_RF               (RF_ADDR_t)0x7F
//#define ADDR_DEFAULT_RF             (RF_ADDR_t)0x04
//#undef  MQTTSN_USE_DHCP

// Object's Dictionary Section
#define OD_DEV_UC_TYPE              'A'
#define OD_DEV_UC_SUBTYPE           '1'
#define OD_DEV_PHY1                 'S'
#define OD_DEV_PHY2                 'T'
#define OD_DEV_HW_TYP_H             '1'
#define OD_DEV_HW_TYP_L             '2'

#define OD_ADDR_TYPE                objUInt8

#include "../PHY/UART/uart_phy.h"
#include "../PHY/TWIMM/twimm_phy.h"

#define PHY1_Init                   UART_Init
#define PHY1_Send                   UART_Send
#define PHY1_Get                    UART_Get
#define PHY1_NodeId                 objRFNodeId
#define PHY1_GateId                 objGateID

#define PHY2_Init                   TWIMM_Init
#define PHY2_Send                   TWIMM_Send
#define PHY2_Get                    TWIMM_Get
#define PHY2_NodeId                 objRFNodeId

#ifdef __cplusplus
}
#endif

#endif // HWCONFIG_A12N12_H