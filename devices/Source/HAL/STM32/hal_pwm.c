#include "../../config.h"

#ifdef EXTPWM_USED

static const uint8_t  hal_pwm_port2dio[] = EXTPWM_PORT2DIO;

uint8_t hal_pwm_base2dio(uint16_t base)
{
    if(base >= sizeof(hal_pwm_port2dio))
        return 0xFF;
    
    return hal_pwm_port2dio[base];
}


#endif  //  EXTPWM_USED