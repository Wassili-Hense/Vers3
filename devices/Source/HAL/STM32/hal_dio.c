#include "../../config.h"

static const GPIO_TypeDef * dio_pGPIOx[] = EXTDIO_PORTNUM2PORT;

static GPIO_TypeDef * dioPortNr2GPIOx(uint8_t PortNr)
{
    if(PortNr < EXTDIO_MAXPORT_NR)
        return (GPIO_TypeDef *)dio_pGPIOx[PortNr];

    return NULL;
}

void hal_dio_configure(uint8_t PortNr, uint16_t Mask, uint8_t Mode)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

#if (defined __STM32F0XX_GPIO_H)
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;

    switch(Mode)
    {
        case DIO_MODE_IN_PD:
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
            GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
            break;
        case DIO_MODE_IN_PU:
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
            GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
            break;
        case DIO_MODE_OUT:
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
            GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
            break;
        case DIO_MODE_AIN:
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
            GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
            break;
        case DIO_MODE_UART:
            if(PortNr == 0)    // USART1 - PA9, PA10
            {
                GPIOA->MODER   |= GPIO_MODER_MODER9_1;          // PA9  (TX) - Alternate function mode
                GPIOA->MODER   |= GPIO_MODER_MODER10_1;         // PA10 (RX) - Alternate function mode
                GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;       // PA9  (TX) - High speed
                GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10;      // PA10 (RX) - High speed
                GPIOA->AFR[1]  |= 0x0110;                       // PA9, PA10 - AF1
            }
            else if(PortNr == 1)    // USART2 - PA2, PA3
            {
                GPIOA->MODER   |= GPIO_MODER_MODER2_1;          // PA2  (TX) - Alternate function mode
                GPIOA->MODER   |= GPIO_MODER_MODER3_1;          // PA3  (RX) - Alternate function mode
                GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2;       // PA2  (TX) - High speed
                GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;       // PA3  (RX) - High speed
                GPIOA->AFR[0]  |= 0x1100;                       // PA2, PA3  - AF1
            }
            return;
//        case DIO_MODE_IN_FLOAT:
//        case DIO_MODE_PWM:
        default:
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
            GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
            break;
    }
#elif (defined __STM32F10x_GPIO_H)
    switch(Mode)
    {
        case DIO_MODE_IN_PD:
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
            break;
        case DIO_MODE_IN_PU:
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
            break;
        case DIO_MODE_OUT:
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
            break;
        case DIO_MODE_AIN:
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
            break;
        case DIO_MODE_UART:
            if(PortNr == 0)         // USART1 - PA9, PA10
            {
                // Configure GPIO, Tx on PA9, Rx on PA10
                GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
                GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9; // AF Push-Pull out (TX)
            }
            else if(PortNr == 1)    // USART2 - PA2, PA3
            {
                // Configure GPIO, Tx on PA2, Rx on PA3
                GPIOA->CRL &= ~GPIO_CRL_CNF2_0;
                GPIOA->CRL |= GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2; // AF Push-Pull out (TX)
            }
            return;
//        case DIO_MODE_IN_FLOAT:
//        case DIO_MODE_PWM:
        default:
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
            break;
    }
#endif

    GPIO_InitStructure.GPIO_Pin     = Mask;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_2MHz;
    GPIO_Init(dioPortNr2GPIOx(PortNr), &GPIO_InitStructure);
}

uint16_t hal_dio_read(uint8_t PortNr)
{
    GPIO_TypeDef * GPIOx = dioPortNr2GPIOx(PortNr);
    if(GPIOx == NULL)
        return 0;

    return ((uint16_t)GPIOx->IDR);
}

void hal_dio_set(uint8_t PortNr, uint16_t Mask)
{
    GPIO_TypeDef * GPIOx = dioPortNr2GPIOx(PortNr);
    if(GPIOx != NULL)
        GPIOx->BSRR = Mask;
}

void hal_dio_reset(uint8_t PortNr, uint16_t Mask)
{
    GPIO_TypeDef * GPIOx = dioPortNr2GPIOx(PortNr);
    if(GPIOx != NULL)
        GPIOx->BRR = Mask;
}
