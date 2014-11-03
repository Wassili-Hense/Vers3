#include "../../config.h"

#ifdef EXTAIN_USED

#include <avr/pgmspace.h>

#ifndef PRR
#define PRR PRR0
#endif  //  PRR

static const PROGMEM uint8_t hal_ainBase2Apin[] = EXTAIN_BASE_2_APIN;

void hal_ain_select(uint8_t apin, uint8_t aref)
{
    if((ADCSRA & (1<<ADEN)) == 0)   // ADC disabled
    {
        PRR &= ~(1<<PRADC);
        ADCSRA = (1<<ADEN) | (7<<ADPS0);
    }

    uint8_t mux = pgm_read_byte(&hal_ainBase2Apin[apin]);
    mux |= aref<<6;

    ADMUX = mux | (1<<ADLAR);
#ifdef  MUX5
    ADCSRB = ((mux & 0x20)>>2);
#endif  //  MUX5

    // Start Conversion
    ADCSRA |= (1<<ADSC);
}

int16_t hal_ain_get(void)
{
    int16_t retval = ADC>>1;
    ADCSRA |= (1<<ADSC);
    return retval;
}
#endif  //EXTAIN_USED
