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

#include <avr/interrupt.h>

static volatile uint8_t SystemTickCnt;

int main(void)
{
    // Initialise System Hardware
    INIT_SYSTEM();
    // Initialise Memory manager
    mqInit();
    // Initialise Object's Dictionary
    InitOD();
    // Initialise PHY Interfaces
    PHY1_Init();
#ifdef PHY2_Init
    PHY2_Init();
#endif  //  PHY2_Init
    // Initialize MQTTSN
    MQTTSN_Init();
#ifdef DIAG_USED
    DIAG_Init();
#endif  //  USE_DIAG
    
    SystemTickCnt = 0;

    StartSheduler();
  
    while(1)
    {
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
        
        if(SystemTickCnt)
        {
            SystemTickCnt = 0;
            MQTTSN_Poll();
            
            OD_Poll();
            
#ifdef DIAG_USED
            DIAG_Poll();
#endif  //  USE_DIAG
        }
    }
}

void SystemTick(void)
{
    SystemTickCnt = 1;
}
