#include "../../config.h"

#if (defined EXTDIO_USED) && (defined EXTPWM_USED)

void hal_pwm_write(uint8_t Config, uint16_t value)
{
    uint8_t channel = Config & 0x07;
    
    uint8_t tccra;
    
    volatile uint8_t * pTIM;

    switch(Config>>3)
    {
        case 0:     // Timer 0
            tccra = TCCR0A & ((1<<COM0A1) | (1<<COM0B1));
            if(channel == 0)
            {
                OCR0A = value>>8;
                if(value != 0)
                    tccra |= (1<<COM0A1);
                else
                    tccra &= ~(1<<COM0A1);
            }
            else
            {
                OCR0B = value>>8;
                if(value != 0)
                    tccra |= (1<<COM0B1);
                else
                    tccra &= ~(1<<COM0B1);
            }

            if(tccra == 0)  // stop timer
            {
                TCCR0A = 0;
                TCCR0B = 0;
            }
            else
            {
                TCCR0A = tccra | (3<<WGM00);    // FastPWM 0x00 - 0xFF
                TCCR0B = (4<<CS00);             // Clock = Fcpu/256
            }
            return;
        case 1:         //  Timer 1
            pTIM = &TCCR1A;
            break;
#ifdef TCCR3A
        case 3:         //  Timer 3
            pTIM = &TCCR3A;
            break;
#endif  // TCCR3A
#ifdef TCCR4A
        case 4:         //  Timer 4
            pTIM = &TCCR4A;
            break;
#endif  // TCCR3A
#ifdef TCCR5A
        case 5:         //  Timer 5
            pTIM = &TCCR5A;
            break;
#endif  // TCCR3A
        default:
            return;
    }

#ifndef OCR1C   // ATM328 + ATM1284
    tccra = *pTIM & ((1<<COM0A1) | (1<<COM0B1));
#else           // ATM2560
    tccra = *pTIM & ((1<<COM1A1) | (1<<COM1B1) | (1<<COM1C1));
    if(channel == 2)
    {
        *(uint16_t *)(pTIM + 0x0C) = value;     // OCRnC
        if(value != 0)
            tccra |= (1<<COM1C1);
        else
            tccra &= ~(1<<COM1C1);
    }
    else
#endif  //  OCR1C
    if(channel == 0)        // Channel 0
    {
        *(uint16_t *)(pTIM + 0x08) = value;     // OCRnA
        if(value != 0)
            tccra |= (1<<COM1A1);
        else
            tccra &= ~(1<<COM1A1);
    }
    else                    // Channel 1
    {
        *(uint16_t *)(pTIM + 0x0A) = value;     // OCRnB
        if(value != 0)
            tccra |= (1<<COM1B1);
        else
            tccra &= ~(1<<COM1B1);
    }

    if(tccra == 0)
    {
        *(pTIM + 0) = 0;        // TCCRnA
        *(pTIM + 1) = 0;        // TCCRnB
    }
    else
    {
        *(uint16_t *)(pTIM + 0x06) = 0xFFFF;    // ICRn
        *(pTIM + 0) = tccra | (2<<WGM10);       // FastPWM, top in ICR1
        *(pTIM + 1) = (3<<WGM12) | (1<<CS00);   // Clock = Fosc
    }
}

#endif  //  (defined EXTDIO_USED) && (defined EXTPWM_USED)