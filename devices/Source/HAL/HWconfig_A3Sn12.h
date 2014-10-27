/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef HWCONFIG_A3SN12_H
#define HWCONFIG_A3SN12_H

// Busware CSM V2.0
// uc ATMega1284p
// Phy1: UART

// PORTA
// PA0  --    LEDG
// PA1  --    LEDR
// PA2  --    Loopback to PA3
// PA3  --    Loopback to PA2
// PA4  CN9
// PA5  CN8
// PA6  --
// PA7  --
// PORTB
// PB0  CN30
// PB1  CN22  RF_IRQ
// PB2  CN24
// PB3  CN25
// PB4  --    RF_NCS
// PB5  CN28  RF_MOSI
// PB6  CN27  RF_MISO
// PB7  CN29  RF_SCK
// PORTC
// PC0  CN15  SCL
// PC1  CN14  SDA
// PC2  CN13  JTAG_TCK
// PC3  CN11  JTAG_TMS
// PC4  CN10  JTAG_TDO
// PC5  CN12  JTAG_TDI
// PC6  --
// PC7  --
// PORTD
// PD0  CN3   RXD0
// PD1  CN4   TXD0
// PD2  CN23  RF_GDO2/Not Used
// PD3  CN6
// PD4  CN7
// PD5  --    LEDB
// PD6  --    LEDY
// PD7  --

#ifdef __cplusplus
extern "C" {
#endif

#include "Atmel/hal.h"

// DIO Section
#define DIO_PORT_SIZE               8
#define EXTDIO_MAXPORT_NR           4                                     // Number of digital Ports
#define EXTDIO_PORTNUM2PORT         {(uint16_t)&PORTA, (uint16_t)&PORTB, (uint16_t)&PORTC, (uint16_t)&PORTD}
#define EXTDIO_PORTNUM2MASK         {(uint8_t)0xC0, (uint8_t)0x10, (uint8_t)0xFC, (uint8_t)0x83}
// End DIO Section

// LEDs
#define LED1_On()                   PORTA |= (1<<PA0)
#define LED1_Off()                  PORTA &= ~(1<<PA0)
#define LEDsInit()                  {DDRA |= ((1<<PA0) | (1<<PA1)); PORTA |= (1<<PA0) | (1<<PA1);}

// UART Section
#define UART_PORT                   PORTD
#define UART_DDR                    DDRD
#define UART_RX_PIN                 PD0
#define UART_TX_PIN                 PD1

#define USART_USE_PORT              1
// End UART Section

#define UART_PHY                    1

#define PHY1_ADDR_t                 uint8_t
#define ADDR_BROADCAST_PHY1         (PHY1_ADDR_t)0x00
#define ADDR_UNDEF_PHY1             (PHY1_ADDR_t)0xFF

#define RF_ADDR_t                   uint8_t
#define ADDR_UNDEF_RF               (RF_ADDR_t)0xFF
//#define ADDR_DEFAULT_RF             (RF_ADDR_t)0x04

// Object's Dictionary Section
#define OD_DEV_UC_TYPE              'A'
#define OD_DEV_UC_SUBTYPE           '3'
#define OD_DEV_PHY1                 'S'
#define OD_DEV_PHY2                 'n'
#define OD_DEV_HW_TYP_H             '1'
#define OD_DEV_HW_TYP_L             '2'

#define OD_ADDR_TYPE                objUInt8

#include "../PHY/UART/uart_phy.h"

#define PHY1_Init                   UART_Init
#define PHY1_Send                   UART_Send
#define PHY1_Get                    UART_Get
#define PHY1_NodeId                 objRFNodeId
#define PHY1_GateId                 objGateID

#ifdef __cplusplus
}
#endif

#endif // HWCONFIG_A3SN12_H