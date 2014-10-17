#include "../../config.h"

#ifdef CC11_PHY

// low level SPI exchange
uint8_t hal_cc11_spiExch(uint8_t data)
{
    RF_SPI_DATA = data;
    while(RF_SPI_BISY);                 // Wait until SPI operation is terminated
    return RF_SPI_DATA;
}

void hal_cc11_init_hw(void)
{
    // HW Initialise
    RF_DISABLE_IRQ();
    RF_PORT_INIT();                         // Ports Init
    RF_SPI_INIT();                          // init SPI controller
    RF_IRQ_CFG();                           // init IRQ input
    // HW End
}

/*
ISR(RF_INT_vect)
{
//  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//  CC11_IRQ_Handler(&xHigherPriorityTaskWoken);
}
*/

#endif  //  CC11_PHY