/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef HWCONFIG_A3SC12_H
#define HWCONFIG_A3SC12_H

// Busware CSM V2.0
// uc ATMega1284p
// Phy1: UART
// Phy2: RF - CC1101

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
#include <util/delay.h>

// DIO Section
#define DIO_PORT_SIZE               8
#define EXTDIO_MAXPORT_NR           4                                     // Number of digital Ports
#define EXTDIO_PORTNUM2PORT         {(uint16_t)&PORTA, (uint16_t)&PORTB, (uint16_t)&PORTC, (uint16_t)&PORTD}
#define EXTDIO_PORTNUM2MASK         {(uint8_t)0xC3, (uint8_t)0xF2, (uint8_t)0xFC, (uint8_t)0x87}
// End DIO Section

// UART Section
#define UART_PORT                   PORTD
#define UART_DDR                    DDRD
#define UART_RX_PIN                 PD0
#define UART_TX_PIN                 PD1

#define USART_USE_PORT              1
// End UART Section

// RF Section
#define TxLEDon()                   PORTA |= (1<<PA1);
#define RxLEDon()                   PORTA |= (1<<PA0);
#define LEDsOff()                   PORTA &= ~((1<<PA0) | (1<<PA1));

#define RF_PORT                     PORTB
#define RF_DDR                      DDRB
#define RF_PIN                      PINB
#define RF_PIN_SS                   PB4
#define RF_PIN_MOSI                 PB5
#define RF_PIN_MISO                 PB6
#define RF_PIN_SCK                  PB7

// RF IRQ
#define RF_IRQ_PORT                 PORTB
#define RF_IRQ_DDR                  DDRB
#define RF_IRQ_PIN                  PIND
#define RF_PIN_IRQ                  PB1
#define RF_IRQ_CFG()                {PCIFR = (1<<PCIF1); PCICR = (1<<PCIE1);}
#define RF_DISABLE_IRQ()            PCMSK1 = 0
#define RF_ENABLE_IRQ()             PCMSK1 = (1<<RF_PIN_IRQ)
#define RF_INT_vect                 PCINT1_vect

#define RF_PORT_INIT()              {PRR0 &= ~(1<<PRSPI);                     \
                                     DDRA |= ((1<<PA0) | (1<<PA1));           \
                                     RF_PORT = (1<<RF_PIN_SS);                \
                                     RF_DDR = (1<<RF_PIN_SCK) | (1<<RF_PIN_MOSI) | (1<<RF_PIN_SS);  \
                                     RF_IRQ_DDR &= ~(1<<RF_PIN_IRQ); RF_IRQ_PORT |= (1<<RF_PIN_IRQ);}
// RF SPI
#define RF_SPI_DATA                 SPDR
#define RF_SPI_BISY                 (!(SPSR &(1<<SPIF)))

#if (F_CPU > 13000000UL)
#define RF_SPI_INIT()               {SPCR = (1<<SPE) | (1<<MSTR); SPSR = 0;}            // F_CPU/4
#else   //  (F_CPU <= 13000000UL)
#define RF_SPI_INIT()               {SPCR = (1<<SPE) | (1<<MSTR); SPSR = (1<<SPI2X);}   // F_CPU/2
#endif  //  (F_CPU > 13000000UL)
//  End RF Section

#define UART_PHY                    1
#define CC11_PHY                    2

#define PHY1_ADDR_t                 uint8_t
#define ADDR_BROADCAST_PHY1         (PHY1_ADDR_t)0x00
#define ADDR_UNDEF_PHY1             (PHY1_ADDR_t)0xFF

#define PHY2_ADDR_t                 uint8_t
#define ADDR_BROADCAST_PHY2         (PHY2_ADDR_t)0x00
#define ADDR_UNDEF_PHY2             (PHY2_ADDR_t)0xFF

// Object's Dictionary Section
#define OD_DEV_UC_TYPE              'A'
#define OD_DEV_UC_SUBTYPE           '3'
#define OD_DEV_PHY1                 'S'
#define OD_DEV_PHY2                 'C'
#define OD_DEV_HW_TYP_H             '1'
#define OD_DEV_HW_TYP_L             '2'

#define OD_ADDR_TYPE                objUInt8

#include "../PHY/UART/uart_phy.h"
#include "../PHY/CC1101/cc11_phy.h"

#define PHY1_Init                   UART_Init
#define PHY1_Send                   UART_Send
#define PHY1_Get                    UART_Get
#define PHY1_NodeId                 objRFNodeId
#define PHY1_GateId                 objGateID

#define PHY2_Init                   CC11_Init
#define PHY2_Send                   CC11_Send
#define PHY2_Get                    CC11_Get
#define PHY2_NodeId                 objRFNodeId

#ifdef __cplusplus
}
#endif

#endif // HWCONFIG_A3SN12_H