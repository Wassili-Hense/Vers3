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

int main(void)
{
    // Initialise System Hardware
    INIT_SYSTEM();
    // Initialise Memory manager
    MEM_Init();
    // Initialise Object's Dictionary
    InitOD();
    // Initialise PHY Interfaces
    PHY1_Init();
#ifdef PHY2_Init
//    PHY2_Init();
#endif  //  PHY2_Init
    // Initialize MQTTSN
//    MQTTSN_Init();

    sei();
  
    while(1)
    {
        MQ_t * pBuf;
        pBuf = PHY1_Get();
        if(pBuf != NULL)
            PHY1_Send(pBuf);
    }

}
