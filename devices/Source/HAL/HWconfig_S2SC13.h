/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef HWCONFIG_S2SC13_H
#define HWCONFIG_S2SC13_H

// Board: Gesture Switch
// uC: STM32F051K8T6
// PHY1: UART
// PHY2: CC1101/26M

// GPIOA
// Pin  CN  Port    Func
//   0  3   PA0     
//   1  4   PA1
//   2  5   PA2     _USART2_TX
//   3  6   PA3     _USART2_RX
//   4  15  PA4     _SPI1_NSS
//   5  16  PA5     _SPI1_SCK
//   6  17  PA6     _SPI1_MISO
//   7  18  PA7     _SPI1_MOSI
//   8      PA8     Sensor_Int
//   9  9   PA9     USART1_TX
//  10  10  PA10    USART1_RX
//  11  7   PA11
//  12  8   PA12
//  13      PA13    SWDIO       _LED
//  14      PA14    SWCLK
//  15      PA15    RF_Select
// GPIOB
//  16  19  PB0
//  17  20  PB1
//  19      PB3     RF_SCK
//  20      PB4     RF_MISO
//  21      PB5     RF_MOSI
//  22  13  PB6     SCL
//  23  14  PB7     SDA


#ifdef __cplusplus
extern "C" {
#endif

#include "STM32/hal.h"

// DIO Section
#define EXTDIO_USED                 1
#define EXTDIO_MAXPORT_NR           2
#define EXTDIO_PORTNUM2PORT         {GPIOA, GPIOB}
#define EXTDIO_PORTNUM2MASK         {(uint16_t)0xC700, (uint16_t)0xFF3C}
// End DIO Section

// PA0-PA7: 0 - 7
// PB0-PB1: 8 - 9
// Analogue Inputs
#define EXTAIN_USED                 1
#define EXTAIN_MAXPORT_NR           10
#define EXTAIN_BASE_2_APIN          {0, 1, 2, 3, 4, 5, 6, 7, 8, 9}
#define EXTAIN_BASE_2_DIO           {0, 1, 2, 3, 4, 5, 6, 7, 16, 17}
#define EXTAIN_REF                  0x02        // Bit0 - Ext, Bit1 - Vcc, Bit2 - Int1, Bit3 - Int2
// End Analogue Inputs

// UART Section
#define UART_PHY_PORT               0   //  0 - USART1 PA9,PA10 GPIOA MASK 0x0600, 1 - USART2 PA2,PA3 GPIO MASK 0x000C
#define EXTSER_USED                 1
#define EXTSER_PORT2UART            {1}
// End UART Section

// LED Section
//#define LED2_On()                   GPIOA->BRR = GPIO_Pin_13
//#define LED2_Off()                  GPIOA->BSRR = GPIO_Pin_13
//#define LEDsInit()                  hal_dio_gpio_cfg(GPIOA, GPIO_Pin_13, DIO_MODE_OUT)
// End LED Section

// CC11 Section
#define CC11_USE_SPI                11

#define CC11_NSS_PORT               GPIOA
#define CC11_NSS_PIN                GPIO_Pin_15

#define CC11_SPI_PORT               GPIOB
#define CC11_SPI_MISO_PIN           GPIO_Pin_4

#define RF_WAIT_LOW_MISO()          while(CC11_SPI_PORT->IDR & CC11_SPI_MISO_PIN)

#define RF_SELECT()                 CC11_NSS_PORT->BRR = CC11_NSS_PIN
#define RF_RELEASE()                CC11_NSS_PORT->BSRR = CC11_NSS_PIN

// End CC11 Section

#define UART_PHY                    1
#define CC11_PHY                    2

#define PHY1_ADDR_t                 uint8_t
#define ADDR_BROADCAST_PHY1         (PHY1_ADDR_t)0x00
#define ADDR_UNDEF_PHY1             (PHY1_ADDR_t)0xFF

#define PHY2_ADDR_t                 uint8_t
#define ADDR_BROADCAST_PHY2         (PHY2_ADDR_t)0x00
#define ADDR_UNDEF_PHY2             (PHY2_ADDR_t)0xFF

// Object's Dictionary Section
#define OD_DEV_UC_TYPE              'S'
#define OD_DEV_UC_SUBTYPE           '2'
#define OD_DEV_PHY1                 'S'
#define OD_DEV_PHY2                 'C'
#define OD_DEV_HW_TYP_H             '1'
#define OD_DEV_HW_TYP_L             '3'

#define OD_ADDR_TYPE                objUInt8

#include "../PHY/UART/uart_phy.h"
#include "../PHY/CC1101/cc11_phy.h"

#define PHY1_Init                   UART_Init
#define PHY1_Send                   UART_Send
#define PHY1_Get                    UART_Get
#define PHY1_GetAddr                UART_GetAddr
#define PHY1_NodeId                 objRFNodeId
#define PHY1_GateId                 objGateID

#define PHY2_Init                   CC11_Init
#define PHY2_Send                   CC11_Send
#define PHY2_Get                    CC11_Get
#define PHY2_GetRSSI                CC11_GetRSSI
#define PHY2_GetAddr                CC11_GetAddr
#define PHY2_NodeId                 objRFNodeId

#ifdef __cplusplus
}
#endif

#endif // HWCONFIG_S2SN12_H
