#include "../../config.h"

#if (defined RFM12_PHY)

void hal_rfm12_init_hw(void)
{
    NVIC_DisableIRQ(RFM12_IRQ);
    
    // RFM12_NSS_PIN
    hal_dio_gpio_cfg(RFM12_NSS_PORT, RFM12_NSS_PIN, DIO_MODE_OUT_PP_HS);
    RFM12_NSS_PORT->BSRR = RFM12_NSS_PIN;
    // RFM12_IRQ_PIN
    hal_dio_gpio_cfg(RFM12_IRQ_PORT, RFM12_IRQ_PIN, DIO_MODE_IN_PU);

    hal_spi_cfg(RFM12_USE_SPI, (HAL_SPI_MODE_0 | HAL_SPI_MSB | HAL_SPI_16B), 2500000UL);
}

uint16_t hal_rfm12_spiExch(uint16_t data)
{
    RFM12_NSS_PORT->BSRR = (RFM12_NSS_PIN<<16);
    data = hal_spi_exch16(RFM12_USE_SPI, data);
    RFM12_NSS_PORT->BSRR = RFM12_NSS_PIN;
    return data;
}

bool hal_rfm12_irq_stat(void)
{
    if(RFM12_IRQ_PORT->IDR & RFM12_IRQ_PIN)
        return false;
    
    return true;
}

void hal_rfm12_enable_irq(void)
{
    hal_exti_config(RFM12_IRQ_PORT, RFM12_IRQ_PIN, HAL_EXTI_TRIGGER_FALLING);

    // Enable and set interrupt
    NVIC_SetPriority(RFM12_IRQ, 0);
    NVIC_EnableIRQ(RFM12_IRQ);
}

// external procedure, defined in rfm12_phy.c
void rfm12_irq(void);

void RFM12_IRQ_HANDLER(void)
{
    //(EXTI_GetITStatus(EXTI_Line0) != RESET)
    if((EXTI->PR & RFM12_IRQ_PIN) != 0)
    {
        rfm12_irq();

        // Clear the EXTI line 0 pending bit
        //EXTI_ClearITPendingBit(EXTI_Line0);
        EXTI->PR = RFM12_IRQ_PIN;
    }
}

#endif  //  RFM12_PHY

