#include "../../config.h"

void hal_exti_config(GPIO_TypeDef *GPIOx, uint16_t Mask, uint8_t Trigger)
{

#if (defined __STM32F0XX_H)
    // Enable SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
#else
    #error Unknown uC
#endif  //  CPU

    uint32_t port;
    uint8_t  pin;

    switch((uint32_t)GPIOx)
    {
#ifdef GPIOA
        case (uint32_t)GPIOA:
            port = 0;
            break;
#endif  //  GPIOA
#ifdef GPIOB
        case (uint32_t)GPIOB:
            port = 1;
            break;
#endif  //  GPIOB
#ifdef GPIOC
        case (uint32_t)GPIOC:
            port = 2;
            break;
#endif  //  GPIOC
#ifdef GPIOD
        case (uint32_t)GPIOD:
            port = 3;
            break;
#endif  //  GPIOD
#ifdef GPIOE
        case (uint32_t)GPIOE:
            port = 4;
            break;
#endif  //  GPIOE
#ifdef GPIOF
        case (uint32_t)GPIOF:
            port = 5;
            break;
#endif  //  GPIOF
#ifdef GPIOG
        case (uint32_t)GPIOG:
            port = 6;
            break;
#endif  //  GPIOG
        default:
            assert(0);      // Unknown Port
    }
    
    uint32_t u32_tmp;
    for(pin = 0; pin < 16; pin++)
    {
        if(Mask & (1 << pin))
        {
#if (defined __STM32F0XX_H)
            // Connect EXTIx Line to IRQ PIN
            u32_tmp = ((uint32_t)0x0F) << (0x04 * (pin & (uint8_t)0x03));
            SYSCFG->EXTICR[pin >> 0x02] &= ~u32_tmp;
            SYSCFG->EXTICR[pin >> 0x02] |= port << (0x04 * (pin & (uint8_t)0x03));
#endif  // CPU
        }
    }

    u32_tmp = Mask;
    EXTI->IMR |= u32_tmp;               // Enable Interrupt
    EXTI->EMR &= ~u32_tmp;              // Disable Events

    if(Trigger & 1)                     // Falling trigger
        EXTI->FTSR |= u32_tmp;
    else
        EXTI->FTSR &= ~u32_tmp;
    
    if(Trigger & 2)                     // Rising trigger
        EXTI->RTSR |= u32_tmp;
    else
        EXTI->RTSR &= ~u32_tmp;
}
