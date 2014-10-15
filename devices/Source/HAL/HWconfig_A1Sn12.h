/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef HWCONFIG_A1SN12_H
#define HWCONFIG_A1SN12_H

// uNode Version 2.0
// uc ATMega328p
// Phy1: UART

// 0 - 7    PORTA - not exist
// PORTB
// --   PB0     --      LED
// --   PB1     --      RF_IRQ
// --   PB2     --      RF_CSN
// --   PB3     ISP-4   RF_MOSI
// --   PB4     ISP-1   RF_MISO
// --   PB5     ISP-3   RF_SCK
// --   PB6     --      OSC
// --   PB7     --      OSC
// PORT C
// 16   PC0     SV1-18   Ain0
// 17   PC1     SV1-17   Ain1
// 18   PC2     SV1-16   Ain2
// 19   PC3     SV1-15   Ain3
// 20   PC4     SV1-14   SDA
// 21   PC5     SV1-13 - SCL
// --   PC6      ISP-5   RESET
// --   --      SV1-20   Ain6
// --   --      SV1-19   Ain7
// PORT D
// 24   PD0     SV1-10  RXD - On gateway busy
// 25   PD1     SV1-9   TXD - On gateway busy
// 26   PD2     SV1-8  IRQ 0 //** RF-IRQ
// 27   PD3     SV1-7  IRQ 1
// 28   PD4     SV1-6
// 29   PD5     SV1-5  PWM0
// 30   PD6     SV1-4  PWM1
// 31   PD7     SV1-3

#include <avr/io.h>
#include <avr/eeprom.h>
//#include <util/delay.h>

#define INIT_SYSTEM()

// UART Section
#define UART_PORT                   PORTD
#define UART_DDR                    DDRD
#define UART_RX_PIN                 PD0
#define UART_TX_PIN                 PD1

#define USART_USE_PORT              1
// End UART Section

#define UART_PHY                    1
#define RF_NODE                     1

#define PHY1_ADDR_t                 uint8_t
#define ADDR_BROADCAST_PHY1         (PHY1_ADDR_t)0x00
#define ADDR_UNDEF_PHY1             (PHY1_ADDR_t)0xFF

#define RF_ADDR_t                   uint8_t
#define ADDR_UNDEF_RF               (RF_ADDR_t)0xFF
//#define ADDR_DEFAULT_RF             (RF_ADDR_t)0x04

// Object's Dictionary Section
#define OD_DEV_UC_TYPE              'A'
#define OD_DEV_UC_SUBTYPE           '1'
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

#define eeprom_init_hw()
#define eeprom_read(pBuf, Addr, Len)  eeprom_read_block((void *)pBuf, (const void *)Addr, (size_t)Len)
#define eeprom_write(pBuf, Addr, Len) eeprom_write_block((const void *)pBuf, (void *)Addr, (size_t)Len)

#endif // HWCONFIG_A1SN12_H