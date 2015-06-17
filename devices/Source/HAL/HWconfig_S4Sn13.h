/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef HWCONFIG_S4SN13_H
#define HWCONFIG_S4SN13_H

// uC: STM32F405RG
// PHY1: UART

/**
GPIOA

0     SN_TX
1     SN_RX
2     CCD_CLK
3     CCD_ST
4     DAC_OUT1
5     DAC_OUT2
6     AIN1
7     AIN2
8
9
10
11    USB_DM
12    USB_DP
13    SWDIO
14    SWCLK
15    M_NSS


GPIOB

0
1
2     BOOT1
3     IO_SCK
4     IO_MISO
5     IO_MOSI
6     IO_TX
7     IO_RX
8     IO_TX_EN
9     IO_NSS
10    OLED_NRST
11    OLED_DNC
12    OLED_NCS
13    OLED_SCK
14    OLED_MISO
15    OLED_MOSI


GPIOC

0     AO2_M
1     AO2_E
2     AO1_M
3     AO1_E
4     IO_O3
5
6     KEY1
7     KEY2
8     KEY3
9     KEY4
10    M_SCK
11    M_MISO
12    M_MOSI
13    IO_IRQ
14    IO_LI
15    IO_O2

GPIOD
2     M_NWP

**/




// GPIOA
// Pin  Port    Func
//   0  PA0     USART4_TX
//   1  PA1     USART4_RX
//   2  PA2     USART2_TX
//   3  PA3     USART2_RX
//   4  PA4
//   5  PA5     SPI1_SCK
//   6  PA6     SPI1_MISO
//   7  PA7     SPI1_MOSI
//   8  PA8
//   9  PA9     USART1_TX
//  10  PA10    USART1_RX
//  11  PA11
//  12  PA12
//  13  PA13    SWDIO
//  14  PA14    SWCLK
//  15  PA15
// GPIOB
//  16  PB0
//  17  PB1
//  18  PB2
//  19  PB3
//  20  PB4
//  21  PB5
//  22  PB6
//  23  PB7
//  24  PB8     I2C1-SCL
//  25  PB9     I2C1-SDA
//  26  PB10
//  27  PB11
//  28  PB12
//  29  PB13
//  30  PB14
//  31  PB15
// GPIOC
//  32  PC0
//  33  PC1
//  34  PC2
//  35  PC3
//  36  PC4
//  37  PC5
//  38  PC6
//  39  PC7
//  40  PC8
//  41  PC9
//  42  PC10    SPI3_SCK
//  43  PC11    SPI3_MISO
//  44  PC12    SPI3_MOSI
//  45  PC13
//  46  PC14
//  47  PC15
// GPIOD
//      PD2


#ifdef __cplusplus
extern "C" {
#endif

#ifdef OD_MAX_INDEX_LIST
#undef OD_MAX_INDEX_LIST
#endif  // OD_MAX_INDEX_LIST

#define OD_MAX_INDEX_LIST           40

#include "STM32/hal.h"

// DIO Section
//#define EXTDIO_USED                 1
#define EXTDIO_MAXPORT_NR           3
#define EXTDIO_PORTNUM2PORT         {GPIOA, GPIOB, GPIOC}
#define EXTDIO_PORTNUM2MASK         {(uint16_t)0x600C, (uint16_t)0x0000, (uint16_t)0x0000}
// End DIO Section

// PA0-PA7: 0 - 7
// PB0-PB1: 8 - 9
// PC0-PC5: 10-15
// Analogue Inputs
//#define EXTAIN_USED                 0
#define EXTAIN_MAXPORT_NR           16
#define EXTAIN_BASE_2_APIN          {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}
#define EXTAIN_BASE_2_DIO           {0, 1, 2, 3, 4, 5, 6, 7, 16, 17, 32, 33, 34, 35, 36, 37}
#define EXTAIN_REF                  0x02        // Bit0 - Ext, Bit1 - Vcc, Bit2 - Int1, Bit3 - Int2
// End Analogue Inputs

// UART Section
#define HAL_USE_UART4
#define UART_PHY_PORT               3
//#define EXTSER_USED                 1
//#define EXTSER_PORT2UART            {1}
// End UART Section

// TWI Section
//#define EXTTWI_USED                 0       // I2C_Bus 1 - I2C1, 2 - I2C2
// End TWI Section

#define UART_PHY                    1

#define PHY1_ADDR_t                 uint8_t
#define ADDR_BROADCAST_PHY1         (PHY1_ADDR_t)0x00
#define ADDR_UNDEF_PHY1             (PHY1_ADDR_t)0xFF

#define RF_ADDR_t                   uint8_t
#define ADDR_UNDEF_RF               (RF_ADDR_t)0xFF
//#define ADDR_DEFAULT_RF             (RF_ADDR_t)0x04

// Object's Dictionary Section
#define OD_DEV_UC_TYPE              'S'
#define OD_DEV_UC_SUBTYPE           '4'
#define OD_DEV_PHY1                 'S'
#define OD_DEV_PHY2                 'n'
#define OD_DEV_HW_TYP_H             '1'
#define OD_DEV_HW_TYP_L             '3'

#define OD_ADDR_TYPE                objUInt8

#include "../PHY/UART/uart_phy.h"

#define PHY1_Init                   UART_Init
#define PHY1_Send                   UART_Send
#define PHY1_Get                    UART_Get
#define PHY1_GetAddr                UART_GetAddr
#define PHY1_NodeId                 objRFNodeId
#define PHY1_GateId                 objGateID

#ifdef __cplusplus
}
#endif

#endif // HWCONFIG_S4SN13_H
