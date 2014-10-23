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

static volatile uint8_t SystemTickCnt;

int main(void)
{
    // Initialise System Hardware
    INIT_SYSTEM();
    // Initialise Memory manager
    mqInit();
    // Initialise Object's Dictionary
    InitOD();
    // Initialize MQTTSN
    MQTTSN_Init();
#ifdef DIAG_USED
    DIAG_Init();
#endif  //  USE_DIAG
    
    SystemTickCnt = 1;

    StartSheduler();
  
    while(1)
    {
        if(SystemTickCnt)
        {
            SystemTickCnt = 0;
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

void SystemTick(void)
{
    SystemTickCnt = 1;
}
