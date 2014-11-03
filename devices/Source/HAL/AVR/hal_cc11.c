#include "../../config.h"

#ifdef CC11_PHY

#ifndef PRR
#define PRR     PRR0
#endif  //  PRR

// low level SPI exchange
uint8_t hal_cc11_spiExch(uint8_t data)
{
    SPDR = data;
    while(!(SPSR &(1<<SPIF)));          // Wait until SPI operation is terminated
    return SPDR;
}

void hal_cc11_init_hw(void)
{
    // Init Ports
    PRR &= ~(1<<PRSPI);
    RF_PORT |= (1<<RF_PIN_SS);
    RF_DDR |= (1<<RF_PIN_SCK) | (1<<RF_PIN_MOSI) | (1<<RF_PIN_SS);
    RF_DDR &= ~(1<<RF_PIN_MISO);

#if (F_CPU > 13000000UL)
    // F_CPU/4
    SPCR = (1<<SPE) | (1<<MSTR); 
    SPSR = 0;            
#else   //  (F_CPU <= 13000000UL)
    // F_CPU/2
    SPCR = (1<<SPE) | (1<<MSTR);
    SPSR = (1<<SPI2X);
#endif  //  (F_CPU > 13000000UL)
}

#endif  //  CC11_PHY