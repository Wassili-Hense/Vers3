#include "../../config.h"

void hal_dio_gpio_cfg(GPIO_TypeDef * GPIOx, uint16_t Mask, uint16_t Mode)
{
    uint16_t pinpos;
    uint32_t pos;
    
    uint32_t pupd = Mode & 0x03;            // Float / Pull-Up / Pull-Down
    uint32_t ppod = (Mode & 0x04) >> 2;     // Push-Pull / Open Drain
    uint32_t mod  = (Mode & 0x18) >> 3;     // Input / Output / AF / Analog
    uint32_t spd  = (Mode & 0x60) >> 5;     // Low / Medim / Fast / High Speed
    uint32_t afr  = (Mode & 0x0F00) >> 8;   // Alternative function
    
    for(pinpos = 0; pinpos < 0x10; pinpos++)
    {
        pos = ((uint32_t)0x01) << pinpos;
        if(Mask & pos)
        {
#if (defined __STM32F0XX_H) || (defined STM32F4XX)
            pos = pinpos << 1;

            // Low Speed
            GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << pos);
            // Output type - push/pull
            GPIOx->OTYPER &= ~((GPIO_OTYPER_OT_0) << pinpos);
            // default state input
            GPIOx->MODER  &= ~(GPIO_MODER_MODER0 << pos);
            // without PullUp/Down
            GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << pos);
            
            GPIOx->PUPDR |= pupd << pos;
            GPIOx->OTYPER |= ppod << pinpos;    
            GPIOx->MODER |= mod << pos;        
            GPIOx->OSPEEDR |= spd << pos;      

            // Set Alternative function
            if(pinpos < 8)              // AFR0
            {
                pos = (pinpos << 2);
                GPIOx->AFR[0] &= ~((uint32_t)0x0000000F << pos);
                GPIOx->AFR[0] |= ((uint32_t)afr << pos);
            }
            else                        // AFR1
            {
                pos = ((pinpos - 8) << 2);
                GPIOx->AFR[1] &= ~((uint32_t)0x0000000F << pos);
                GPIOx->AFR[1] |= ((uint32_t)afr << pos);
            }

#elif (defined __STM32F10x_H)
            uint32_t gpio_cr;

#error Changed Mode
/*
            switch(Mode)
            {
                case DIO_MODE_IN_PU:            // Pull Up
                    gpio_cr = GPIO_CRL_CNF0_1;  // Input with pull-up / pull-down
                    GPIOx->BSRR = pos;
                    break;

                case DIO_MODE_IN_PD:            // Pull Down
                    gpio_cr = GPIO_CRL_CNF0_1;  // Input with pull-up / pull-down
                    GPIOx->BRR = pos;
                    break;

                case DIO_MODE_OUT:              // General purpose output
                    gpio_cr = GPIO_CRL_MODE0_1; // General purpose output push-pull, Low Speed (2MHz)
                    break;

                case DIO_MODE_OUT_HS:           // General purpose output, High Speed
                    gpio_cr = GPIO_CRL_MODE0;   // General purpose output push-pull, High Speed(50MHz)
                    break;

                case DIO_MODE_SPI:              // Alternate functions 0, SPI
                    gpio_cr = GPIO_CRL_CNF0_1 | GPIO_CRL_MODE0;     // AF Push-Pull out, HS
                    break;

                case DIO_MODE_UART:             // Alternate functions 1, UART
                    gpio_cr = GPIO_CRL_CNF0_1 | GPIO_CRL_MODE0_1;     // AF Push-Pull out, LS
                    break;

                case DIO_MODE_AIN:              // Analog Mode
                    gpio_cr = 0;
                    break;
                
//                case DIO_MODE_IN_FLOAT:
//                case DIO_MODE_PWM:
                default:
                    gpio_cr = GPIO_CRL_CNF0_0;  // Floating digital input
                    break;
            }

            pos = (pinpos & 0x07) << 2;
            gpio_cr <<= pos;

            if(pinpos < 0x08)
            {
                GPIOx->CRL &= ~(((uint32_t)0x0F) << pos);
                GPIOx->CRL |= gpio_cr;
            }
            else
            {
                GPIOx->CRH &= ~(((uint32_t)0x0F) << pos);
                GPIOx->CRH |= gpio_cr;
            }
*/
#endif
        }
    }
}

#ifdef EXTDIO_USED

static const GPIO_TypeDef * dio_pGPIOx[] = EXTDIO_PORTNUM2PORT;

static GPIO_TypeDef * dioPortNr2GPIOx(uint8_t PortNr)
{
    if(PortNr < EXTDIO_MAXPORT_NR)
        return (GPIO_TypeDef *)dio_pGPIOx[PortNr];

    return NULL;
}

void hal_dio_configure(uint8_t PortNr, uint16_t Mask, uint16_t Mode)
{
    GPIO_TypeDef * GPIOx = dioPortNr2GPIOx(PortNr);
    if(GPIOx == NULL)
        return;

    hal_dio_gpio_cfg(GPIOx, Mask, Mode);
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

#endif  //  EXTDIO_USED
