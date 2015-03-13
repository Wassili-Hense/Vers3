#include "../../config.h"

#ifdef RFM12_PHY

#include <avr/interrupt.h>

#ifndef PRR
#define PRR     PRR0
#endif  //  PRR

#define RF_SELECT()     RF_PORT &= ~(1<<RF_PIN_SS);
#define RF_RELEASE()    RF_PORT |= (1<<RF_PIN_SS);

void rfm12_irq(void);

// low level SPI exchange
uint16_t hal_rfm12_spiExch(uint16_t data)
{
    uint16_t retval;
    RF_SELECT();
    SPDR = data>>8;
    while(!(SPSR &(1<<SPIF)));          // Wait until SPI operation is terminated
    retval = SPDR<<8;
    SPDR = data & 0xFF;
    while(!(SPSR &(1<<SPIF)));          // Wait until SPI operation is terminated
    retval |= SPDR;
    RF_RELEASE();
    return retval;
}

void hal_rfm12_init_hw(void)
{
    // ToDo IRQ only for UNode.

    // HW Initialise
    PRR &= ~(1<<PRSPI);                                 // Enable SPI
    PCMSK0 = 0;                                         // Disable RF IRQ
    RF_PORT |= (1<<RF_PIN_SS) | (1<<RF_PIN_IRQ);        // Init DIO
    RF_DDR  |= (1<<RF_PIN_SCK) | (1<<RF_PIN_MOSI) | (1<<RF_PIN_SS);
    RF_DDR &= ~(1<<RF_PIN_MISO);
    // Init SPI
#if (F_CPU > 10000000UL)
    SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0);
    SPSR = (1<<SPI2X);
#else   //  (F_CPU <= 10000000UL)
    SPCR = (1<<SPE) | (1<<MSTR);
    SPSR = 0;
#endif  //  (F_CPU > 10000000UL)

    // HW End
}

void hal_rfm12_spi_fast(void)
{
#if (F_CPU > 10000000UL)
    SPCR &= ~(1<<SPR0);
#else   //  (F_CPU <= 10000000UL)
    SPSR = (1<<SPI2X);
#endif  //  (F_CPU > 10000000UL)
}

void hal_rfm12_spi_slow(void)
{
#if (F_CPU > 10000000UL)
    SPCR |= (1<<SPR0);
#else   //  (F_CPU <= 10000000UL)
    SPSR = 0;
#endif  //  (F_CPU > 10000000UL)
}

bool hal_rfm12_irq_stat(void)
{
    return ((RF_PIN & (1<<RF_PIN_IRQ)) == 0);
}

void hal_rfm12_enable_irq(void)
{
    PCICR = (1<<PCIE0);                                 // Enable RF IRQ;
}

ISR(PCINT0_vect)
{
    if(RF_PIN & (1<<RF_PIN_IRQ))
        return;

    rfm12_irq();
}

#endif  //  RFM12_PHY
