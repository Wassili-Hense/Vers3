#include "../../config.h"

#ifdef EXTAIN_USED

static const uint8_t hal_ainBase2Apin[] = EXTAIN_BASE_2_APIN;

void hal_ain_select(uint8_t apin, uint8_t unused)
{
#if (defined __STM32F0XX_H)
    if((RCC->APB2ENR & RCC_APB2ENR_ADC1EN) == 0)
    {
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC Clock

        ADC1->CR |= ADC_CR_ADCAL;           // ADC calibration
        while(ADC1->CR & ADC_CR_ADCAL);
        
        ADC1->CFGR1 =                       // Analog Watchdog disabled
                                            // Regular Channels
                        ADC_CFGR1_AUTOFF |  // auto power off
                                            // Single conversion
                        ADC_CFGR1_OVRMOD |  // Overrun mode
                                            // Hardware trigger disabled
                        ADC_CFGR1_ALIGN;    // Left Aligned 12 bit
                                            // DMA Disabled

        ADC1->SMPR = ADC_SMPR1_SMPR;        // Sampling 239,5 ADC Clock cycles

        ADC1->CR |= ADC_CR_ADEN;            // ADC enabled
    }

    uint32_t mux;
    mux = hal_ainBase2Apin[apin];
    mux = (1 << mux);

    ADC1->CHSELR = mux;

#elif (defined __STM32F10x_H)
    if((RCC->APB2ENR & RCC_APB2ENR_ADC1EN) == 0)
    {
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC Clock

        ADC1->CR2 |= ADC_CR2_CAL;           // ADC calibration
        while (ADC1->CR2 & ADC_CR2_CAL);
        
        ADC1->CR2 = ADC_CR2_ALIGN;          // Left Aligned 12 bit
        // Sampling 239,5 ADC Clock cycles
        ADC1->SMPR1 = ADC_SMPR1_SMP10 | ADC_SMPR1_SMP11 | ADC_SMPR1_SMP12 | 
                      ADC_SMPR1_SMP13 | ADC_SMPR1_SMP14 | ADC_SMPR1_SMP15 |
                      ADC_SMPR1_SMP16 | ADC_SMPR1_SMP17;
        ADC1->SMPR2 = ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1 | ADC_SMPR2_SMP2 |
                      ADC_SMPR2_SMP3 | ADC_SMPR2_SMP4 | ADC_SMPR2_SMP5 |
                      ADC_SMPR2_SMP6 | ADC_SMPR2_SMP7 | ADC_SMPR2_SMP8 |
                      ADC_SMPR2_SMP9;

        ADC1->SQR1 = 0;                     // 1 Conversion

        ADC1->CR2 |= ADC_CR2_ADON;          // ADC enabled
    }

    ADC1->SQR3 = hal_ainBase2Apin[apin];
#else
    #error unknown configuration
#endif
}

int16_t hal_ain_get(void)
{
    int16_t retval;
#if (defined __STM32F0XX_H)
    retval = (ADC1->DR) >> 1;
    // Start Conversion
    ADC1->CR |= ADC_CR_ADSTART;
#elif (defined __STM32F10x_H)
    retval = ((ADC1->DR) & 0x0000FFFF) >> 1;
    // Start Conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;
#endif
    return retval;
}
#endif  //EXTAIN_USED

