#include "../../config.h"

#include <avr/interrupt.h>

/* Hardware constants for timer 2. */
#define portCLOCK_PRESCALER             1024UL
#define portPRESCALE                    0x07

// portPRESCALE
// 1 - portCLOCK_PRESCALER = 1
// 2 - portCLOCK_PRESCALER = 8
// 3 - portCLOCK_PRESCALER = 32
// 4 - portCLOCK_PRESCALER = 64
// 5 - portCLOCK_PRESCALER = 128
// 6 - portCLOCK_PRESCALER = 256
// 7 - portCLOCK_PRESCALER = 1024

#define portCOMPARE_VALUE ((F_CPU/portCLOCK_PRESCALER/configTICK_RATE_HZ)-1)
#if (portCOMPARE_VALUE > 255) || (portCOMPARE_VALUE < 60)
#error Check F_CPU, configTICK_RATE_HZ and portCLOCK_PRESCALER
#endif

void StartSheduler(void)
{
    TCCR2A = (1<<WGM21);
    TCNT2 = 0;
    OCR2A = portCOMPARE_VALUE;
    TIFR2 = (1<<OCF2A);
    TIMSK2 = (1<<OCIE2A);
    TCCR2B = portPRESCALE;

    sei();
}

ISR(TIMER2_COMPA_vect)
{
    SystemTick();
}
