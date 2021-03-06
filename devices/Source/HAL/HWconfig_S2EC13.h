/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef HWCONFIG_S2EC13_H
#define HWCONFIG_S2EC13_H

// Board: S2EC13
// uC: STM32F051C8T6
// PHY1: ENC28J60
// PHY2: CC1101

// GPIOA
// Pin  Port    CN  Func
//   0  PA0     3
//   1  PA1     4
//   2  PA2     5   _USART2_TX
//   3  PA3     6   _USART2_RX
//   4  PA4     15
//   5  PA5     16
//   6  PA6     17
//   7  PA7     18
//   8  PA8
//   9  PA9     9   _USART1_TX
//  10  PA10    10  _USART1_RX
//  11  PA11    7
//  12  PA12    8
//  13  PA13        SWDIO
//  14  PA14        SWCLK
//  15  PA15        ENC_IRQ
// GPIOB
//  16  PB0     19
//  17  PB1     20
//  18  PB2         LED
//  19  PB3         ENC_SCK
//  20  PB4         ENC_MISO
//  21  PB5         ENC_MOSI
//  22  PB6         ENC_SEL
//  23  PB7         ENC_RST
//  24  PB8
//  25  PB9
//  26  PB10    13  SCL2
//  27  PB11    14  SDA2
//  28  PB12        CC11_SEL
//  29  PB13        CC11_SCK
//  30  PB14        CC11_MISO
//  31  PB15        CC11_MOSI


#ifdef __cplusplus
extern "C" {
#endif

#ifdef OD_MAX_INDEX_LIST
#undef OD_MAX_INDEX_LIST
#endif  // OD_MAX_INDEX_LIST

#define OD_MAX_INDEX_LIST           40

#include "STM32/hal.h"

// DIO Section
#define EXTDIO_USED                 1
#define EXTDIO_MAXPORT_NR           2
#define EXTDIO_PORTNUM2PORT         {GPIOA, GPIOB}
#define EXTDIO_PORTNUM2MASK         {(uint16_t)0xE100, (uint16_t)0xF3FC}
// End DIO Section

// PA0-PA7: 0 - 7
// PB0-PB1: 8 - 9
// Analogue Inputs
#define EXTAIN_USED                 1
#define EXTAIN_MAXPORT_NR           10
#define EXTAIN_BASE_2_APIN          {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0xFF, 0xFF}  // MUST BE ALIGNED TO 4
#define EXTAIN_REF                  0x02        // Bit0 - Ext, Bit1 - Vcc, Bit2 - Int1, Bit3 - Int2
// End Analogue Inputs

// UART Section
#define HAL_UART_NUM_PORTS          2
#define HAL_USE_USART1              0           // Mapping to logical port
#define HAL_USE_USART2              1

#define EXTSER_USED                 2
// End UART Section

// TWI Section
#define EXTTWI_USED                 2       // I2C_Bus 1 - I2C1, 2 - I2C2
// End TWI Section

// LEDs
#define LED2_On()                   GPIOB->BSRR = GPIO_BSRR_BS_2
#define LED2_Off()                  GPIOB->BSRR = GPIO_BSRR_BR_2
#define LEDsInit()                  hal_dio_gpio_cfg(GPIOB, GPIO_Pin_2, DIO_MODE_OUT_PP)

// ENC Section
#define HAL_USE_SPI1                1   // SPI1
#define ENC_USE_SPI                 11  // Config 2, PB3-PB5
#define ENC_NSS_PORT                GPIOB
#define ENC_NSS_PIN                 GPIO_Pin_6

#define ENC_SELECT()                ENC_NSS_PORT->BRR = ENC_NSS_PIN
#define ENC_RELEASE()               {while(SPI1->SR & SPI_SR_BSY); ENC_NSS_PORT->BSRR = ENC_NSS_PIN;}

#define ENC28J60_PHY                1
// End ENC Section

// CC11 Section
#define HAL_USE_SPI2                1

#define CC11_USE_SPI                2

#define CC11_NSS_PORT               GPIOB
#define CC11_NSS_PIN                GPIO_Pin_12

#define CC11_SPI_PORT               GPIOB
#define CC11_SPI_MISO_PIN           GPIO_Pin_14

#define CC11_WAIT_LOW_MISO()        while(CC11_SPI_PORT->IDR & CC11_SPI_MISO_PIN)

#define CC11_SELECT()               CC11_NSS_PORT->BRR = CC11_NSS_PIN
#define CC11_RELEASE()              CC11_NSS_PORT->BSRR = CC11_NSS_PIN

#define CC11_PHY                    2
// End CC11 Section

// Object's Dictionary Section
#define OD_DEV_UC_TYPE              'S'
#define OD_DEV_UC_SUBTYPE           '2'
#define OD_DEV_PHY1                 'E'
#define OD_DEV_PHY2                 'C'
#define OD_DEV_HW_TYP_H             '1'
#define OD_DEV_HW_TYP_L             '3'

#define OD_DEV_MAC                  {0x00,0x04,0xA3,0x00,0x00,0x03}   // MAC MSB->LSB
//#define OD_DEF_IP_ADDR              inet_addr(192,168,10,202)
//#define OD_DEF_IP_MASK              inet_addr(255,255,255,0)
//#define OD_DEF_IP_ROUTER            inet_addr(192,168,10,1)
//#define OD_DEF_IP_BROKER            inet_addr(192,168,20,8)

#include "../PHY/ENC28J60/enc28j60_phy.h"
#include "../PHY/CC1101/cc11_phy.h"

#ifdef __cplusplus
}
#endif

#endif // HWCONFIG_S2EC13_H
