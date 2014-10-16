#include "config.h"

#ifdef DIAG_USED

#define DIAG_DEF_TIMEOUT    (300 * configTICK_RATE_HZ)

static uint16_t diag_timeout;

void DIAG_Init(void)
{
    diag_timeout = DIAG_DEF_TIMEOUT;
}

void DIAG_Poll(void)
{
    if(diag_timeout > 0)
    {
        diag_timeout--;
        return;
    }
    
    diag_timeout = DIAG_DEF_TIMEOUT;
    
    MQ_t * pMessage = mqAlloc(sizeof(MQ_t));
    if(pMessage == NULL)
        return;
    
    uint16_t heap = mqGetFreeHeap();

    pMessage->mq.publish.Data[0] = (heap >> 8);
    pMessage->mq.publish.Data[1] = heap & 0xFF;
    pMessage->Length = 2;
    
    mqttsn_trace_msg(lvlINFO, pMessage);
}


#endif  //  DIAG_USED
