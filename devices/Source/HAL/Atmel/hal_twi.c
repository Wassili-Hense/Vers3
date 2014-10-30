#include "../../config.h"

#ifdef EXTTWI_USED

#include "../../EXT/exttwi.h"

#include <avr/interrupt.h>
#include <util/twi.h>

static volatile bool twi_free = true;

TWI_FRAME_t * pTwiFrame;

bool hal_twi_configure(uint8_t enable)
{
    if(enable)
    {
        // Disable TWI
        TWCR = (1<<TWINT);
        // Set Speed
        TWBR = (((F_CPU/100000UL)-16)/2);   // 100kHz
        // Clear data register
        TWDR = 0xFF;
        // Enable TWI & Interrupt
        TWCR = (1<<TWEN) | (1<<TWIE);
    }
    else
    {
        // Disable TWI
        TWCR = (1<<TWINT);
    }

    if(TWIM_SCL_STAT() == 0)
        return false;

    return true;
}

bool hal_twi_can_send(void)
{
    return (twi_free && (TWIM_SCL_STAT() != 0));
}

void hal_twim_send(TWI_FRAME_t * pFrame, bool blocked)
{
    if(blocked)
    {
    }
}


ISR(TWI_vect)
{
}


#endif  //  EXTTWI_USED
