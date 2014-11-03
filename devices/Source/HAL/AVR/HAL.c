#include "../../config.h"

#include <avr/interrupt.h>

/* Hardware constants for timer 2. */
#define halCLOCK_CONFIG                 0x07
#define halCLOCK_PRESCALER              1024UL

// halCLOCK_CONFIG
// 1 - halCLOCK_PRESCALER = 1
// 2 - halCLOCK_PRESCALER = 8
// 3 - halCLOCK_PRESCALER = 32
// 4 - halCLOCK_PRESCALER = 64
// 5 - halCLOCK_PRESCALER = 128
// 6 - halCLOCK_PRESCALER = 256
// 7 - halCLOCK_PRESCALER = 1024

#define halCLOCK_COMPARE_VALUE ((F_CPU/halCLOCK_PRESCALER/POLL_TMR_FREQ)-1)
#if (halCLOCK_COMPARE_VALUE > 255) || (halCLOCK_COMPARE_VALUE < 60)
#error Check F_CPU, POLL_TMR_FREQ and halCLOCK_PRESCALER
#endif

void StartSheduler(void)
{
    TCCR2A = (1<<WGM21);
    TCNT2 = 0;
    OCR2A = halCLOCK_COMPARE_VALUE;
    TIFR2 = (1<<OCF2A);
    TIMSK2 = (1<<OCIE2A);
    TCCR2B = halCLOCK_CONFIG;

    sei();
}

// Generate pseudo random uint16
uint16_t halRNG()
{
    static uint16_t rand16 = 0xA15E;

    // Galois LFSRs
    if(rand16 & 1)
    {
        rand16 >>= 1;
        rand16 ^= 0xB400;
    }
    else
        rand16 >>= 1;
  
    return rand16;
}

// Main program tick procedure
void SystemTick(void);

ISR(TIMER2_COMPA_vect)
{
    SystemTick();
}
