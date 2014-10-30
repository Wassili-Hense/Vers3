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

// Arduino MEGA
// uc ATMega2560
// PHY: UART

// PORTA
// PA0  22
// PA1  23
// PA2  24
// PA3  25
// PA4  26
// PA5  27
// PA6  28
// PA7  29
// PORTB
// PB0  53      SS
// PB1  52      SCK       ICSP-3
// PB2  51      MOSI      ICSP-4
// PB3  50      MISO      ICSP-1
// PB4  10      OC2A
// PB5  11      OC1A
// PB6  12      OC1B
// PB7  13      OC0A/OC1C       LED_L
// PORTC
// PC0  37
// PC1  36
// PC2  35
// PC3  34
// PC4  33
// PC5  32
// PC6  31
// PC7  30
// PORTD
// PD0  21      SCL
// PD1  20      SDA
// PD2  19      RXD1
// PD3  18      TXD1
// PD4  --
// PD5  --
// PD6  --
// PD7  38
// PORTE
// PE0  0       RXD0
// PE1  1       TXD0
// PE2  --
// PE3  5       OC3A
// PE4  2       OC3B
// PE5  3       OC3C
// PE6  --
// PE7  --
// PORTF
// PF0  A0
// PF1  A1
// PF2  A2
// PF3  A3
// PF4  A4      JTAG-TCK
// PF5  A5      JTAG-TMS
// PF6  A6      JTAG-TDO
// PF7  A7      JTAG-TDO
// PORTG
// PG0  41
// PG1  40
// PG2  39
// PG3  --
// PG4  --
// PG5  4       OC0B
// PORTH
// PH0  17      RXD2
// PH1  16      TXD2
// PH2  --
// PH3  6       OC4A
// PH4  7       OC4B
// PH5  8       OC4C
// PH6  9       OC2B
// PH7  --
// PORTJ
// PJ0  15      RXD3
// PJ1  14      TXD3
// PJ2  --
// PJ3  --
// PJ4  --
// PJ5  --
// PJ6  --
// PJ7  --
// PORTK
// PK0  A8
// PK1  A9
// PK2  A10
// PK3  A11
// PK4  A12
// PK5  A13
// PK6  A14
// PK7  A15
// PORTL
// PL0  49
// PL1  48
// PL2  47
// PL3  46      OC5A
// PL4  45      OC5B
// PL5  44      OC5C
// PL6  43
// PL7  42

#ifdef __cplusplus
extern "C" {
#endif

#include "Atmel/hal.h"

// DIO Section
#define DIO_PORT_SIZE               8
#define EXTDIO_MAXPORT_NR           11                                     // Number of digital Ports
#define EXTDIO_PORTNUM2PORT         {(uint16_t)&PORTA, (uint16_t)&PORTB, (uint16_t)&PORTC, (uint16_t)&PORTD,  \
                                     (uint16_t)&PORTE, (uint16_t)&PORTF, (uint16_t)&PORTG, (uint16_t)&PORTH,  \
                                     (uint16_t)&PORTJ, (uint16_t)&PORTK, (uint16_t)&PORTL}
#define EXTDIO_PORTNUM2MASK         {(uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x70,  \
                                     (uint8_t)0xC7, (uint8_t)0xF0, (uint8_t)0xD8, (uint8_t)0x84,  \
                                     (uint8_t)0xFC, (uint8_t)0x00, (uint8_t)0x00}
// End DIO Section

// Analogue Inputs
#define EXTAIN_MAXPORT_NR           16           // ADC0-ADC7, ADC8-ADC15
#define EXTAIN_BASE_2_APIN          {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, \
                                     0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27}
//#define EXTAIN_BASE_2_DIO           {4, 5, 0xFF}
#define EXTAIN_REF                  0x0F        // Bit0 - Ext, Bit1 - Vcc, Bit2 - Int1, Bit3 - Int2
// End Analogue Inputs


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
#define OD_DEV_UC_SUBTYPE           '4'
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