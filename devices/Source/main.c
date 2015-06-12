/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#include "config.h"
#include "diag.h"

static volatile uint8_t SystemTickCnt;

// External procedure defined in HAL
void StartSheduler(void);

int main(void)
{
    // Initialise System Hardware
    INIT_SYSTEM();
    // Initialise Memory manager
    mqInit();
    // Initialise Object's Dictionary
    InitOD();
    // Initialise PHY's
    PHY1_Init();
#ifdef PHY2_ADDR_t
    PHY2_Init();
#endif  //  PHY2_ADDR_t
    // Initialize MQTTSN
    MQTTSN_Init();
    // Initialise optional components
#ifdef  LEDsInit
    LEDsInit();
#endif  //  LEDsInit
#ifdef DIAG_USED
    DIAG_Init();
#endif  //  USE_DIAG

    SystemTickCnt = 0;

    StartSheduler();
  
    while(1)
    {
        if(SystemTickCnt)
        {
            SystemTickCnt--;
            OD_Poll();

            MQTTSN_Poll();
#ifdef DIAG_USED
            DIAG_Poll();
#endif  //  USE_DIAG
        }

        MQ_t * pBuf;
        pBuf = PHY1_Get();
        if(pBuf != NULL)
        {
            mqttsn_parser_phy1(pBuf);
        }

#ifdef PHY2_Get
        pBuf = PHY2_Get();
        if(pBuf != NULL)
        {
            mqttsn_parser_phy2(pBuf);
        }
#endif  //  PHY2_Get
    }
}

static uint8_t LED_Mask = 0;

// PHY activity indicator
void Activity(uint8_t pin)
{
    LED_Mask |= 1 << pin;
}

void SystemTick(void)
{
    SystemTickCnt++;

#ifdef LED1_On
    if(LED_Mask & 2)
    {
        LED_Mask &= 0xFD;
        LED1_On();
    }
    else
        LED1_Off();
#endif  //  LED1_On

#ifdef LED2_On
    if(LED_Mask & 4)
    {
        LED_Mask &= 0xFB;
        LED2_On();
    }
    else
        LED2_Off();
#endif  //  LED2_On
}
