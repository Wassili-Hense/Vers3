#include "../../config.h"

#if (defined RFM12_PHY)

#if (defined __STM32F0XX_H)

// ToDo Definition for Debug
#if (RFM12_USE_SPI == 2)

#define RFM12_NSS_PORT          GPIOB
#define RFM12_NSS_PIN           GPIO_Pin_12

#define RFM12_IRQ_PORT          GPIOB
#define RFM12_IRQ_PIN           GPIO_Pin_11
#define RFM12_IRQ_EXTI_PORT     1                   // 0 - GPIOA, 1 - GPIOB, 2 - GPIOC
#define RFM12_IRQ_EXTI_PIN      11                  // 0 - GPIO_Pin_0, .., 11 -  GPIO_Pin_11
#endif      // (RFM12_USE_SPI == 2)

#if (RFM12_IRQ_EXTI_PIN < 2)
    #define RFM12_IRQ           EXTI0_1_IRQn
    #define RFM12_IRQ_HANDLER   EXTI0_1_IRQHandler
#elif (RFM12_IRQ_EXTI_PIN < 4)
    #define RFM12_IRQ           EXTI2_3_IRQn
    #define RFM12_IRQ_HANDLER   EXTI2_3_IRQHandler
#else
    #define RFM12_IRQ           EXTI4_15_IRQn
    #define RFM12_IRQ_HANDLER   EXTI4_15_IRQHandler
#endif


void hal_rfm12_init_hw(void)
{
    // ToDo Disable IRQ
    
    // RFM12_NSS_PIN
    hal_dio_gpio_cfg(RFM12_NSS_PORT, RFM12_NSS_PIN, DIO_MODE_OUT_HS);
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
    // Enable SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Connect EXTIx Line to IRQ PIN
    uint32_t tmp = ((uint32_t)0x0F) << (0x04 * (RFM12_IRQ_EXTI_PIN & (uint8_t)0x03));
    SYSCFG->EXTICR[RFM12_IRQ_EXTI_PIN >> 0x02] &= ~tmp;
    SYSCFG->EXTICR[RFM12_IRQ_EXTI_PIN >> 0x02] |= (((uint32_t)RFM12_IRQ_EXTI_PORT) << (0x04 * (RFM12_IRQ_EXTI_PIN & (uint8_t)0x03)));
/*
    // Configure EXTI0 line
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Enable and set EXTI0 Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
*/
}


void RFM12_IRQ_HANDLER(void)
{
    //(EXTI_GetITStatus(EXTI_Line0) != RESET)
    if((EXTI->PR & RFM12_IRQ_PIN) != 0)
    {
        // ToDo EXTI procedure

        // Clear the EXTI line 0 pending bit
        //EXTI_ClearITPendingBit(EXTI_Line0);
        EXTI->PR = RFM12_IRQ_PIN;
    }
}

#endif  //  CPU

#endif  //  RFM12_PHY

