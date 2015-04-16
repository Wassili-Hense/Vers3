#include "../../config.h"

#ifdef EXTAIN_USED

static const uint8_t hal_ainBase2Apin[] = EXTAIN_BASE_2_APIN;

void hal_ain_select(uint8_t apin, uint8_t unused)
{
    if((RCC->APB2ENR & RCC_APB2ENR_ADC1EN) == 0)
    {
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

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
                                            
        ADC1->SMPR = 0;                     // Sampling 1,5 ADC Clock cycles

        ADC1->CR |= ADC_CR_ADEN;            // ADC enabled
     //   while((ADC1->ISR & ADC_ISR_ADRDY) == 0);
    }

    uint32_t mux;
    mux = hal_ainBase2Apin[apin];
    mux = (1 << mux);

    ADC1->CHSELR = mux;

    // Start Conversion
    //ADC1->CR |= ADC_CR_ADSTART;
}

int16_t hal_ain_get(void)
{
    int16_t retval = (ADC1->DR) >> 1;

    // Start Conversion
    ADC1->CR |= ADC_CR_ADSTART;

    return retval;
}
#endif  //EXTAIN_USED

/*
STM32F1

void ADC_init(void)
{
// Ð½Ð°Ñ�Ñ‚Ñ€Ð¾Ð¹ÐºÐ° ADC1
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //  Ñ‚Ð°ÐºÑ‚Ñ‹ Ð½Ð° ADC1

        ADC1->SMPR2 |= ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1 
        | ADC_SMPR2_SMP2 | ADC_SMPR2_SMP3 | ADC_SMPR2_SMP4 
        | ADC_SMPR2_SMP5 | ADC_SMPR2_SMP6 | ADC_SMPR2_SMP7;       // ÐºÐ¾Ð»Ð¸Ñ‡ÐµÑ�Ñ‚Ð²Ð¾ Ñ†Ð¸ÐºÐ»Ð¾Ð² Ð¿Ñ€ÐµÐ¾Ð±Ñ€Ð°Ð·Ð¾Ð²Ð°Ð½Ð¸Ñ� 239.5

        ADC1->SQR1 |= ADC_SQR1_L_2 | ADC_SQR1_L_1 | ADC_SQR1_L_0; // Ð´Ð»Ð¸Ð½Ð° Ð¿Ð¾Ñ�Ð»ÐµÐ´Ð¾Ð²Ð°Ñ‚ÐµÐ»ÑŒÐ½Ð¾Ñ�Ñ‚Ð¸ Ñ€ÐµÐ³ÑƒÐ»Ñ�Ñ€Ð½Ñ‹Ñ… ÐºÐ°Ð½Ð°Ð»Ð¾Ð² = 8;
        // Ð´Ð¾Ð±Ð°Ð²Ð»ÐµÐ½Ð¸Ðµ Ð² Ð¿Ð¾Ñ�Ð»ÐµÐ´Ð¾Ð²Ð°Ñ‚ÐµÐ»ÑŒÐ½Ð¾Ñ�Ñ‚ÑŒ ÐºÐ°Ð½Ð°Ð»Ð¾Ð²
        ADC1->SQR2 |= ADC_SQR2_SQ8_2 | ADC_SQR2_SQ8_1 | ADC_SQR2_SQ8_0 //ÐºÐ°Ð½Ð°Ð» 7
        | ADC_SQR2_SQ7_2 | ADC_SQR2_SQ7_1;                             // ÐºÐ°Ð½Ð°Ð» 6

        ADC1->SQR3 |=  ADC_SQR3_SQ6_2 | ADC_SQR3_SQ6_0                 //ÐºÐ°Ð½Ð°Ð» 5
        | ADC_SQR3_SQ5_2                                               //ÐºÐ°Ð½Ð°Ð» 4
        | ADC_SQR3_SQ4_1 | ADC_SQR3_SQ4_0                              //ÐºÐ°Ð½Ð°Ð» 3
        | ADC_SQR3_SQ3_1                                               //ÐºÐ°Ð½Ð°Ð» 2
        | ADC_SQR3_SQ2_0;                                              // ÐºÐ°Ð½Ð°Ð» 1
        //ÐºÐ°Ð½Ð°Ð» 0 Ð²ÐºÐ»ÑŽÑ‡ÐµÐ½ Ð¿Ð¾ ÑƒÐ¼Ð¾Ð»Ñ‡Ð°Ð½Ð¸ÑŽ Ñ‚.Ðº. Ð² Ð¼Ð»Ð°Ð´ÑˆÐ¸Ñ… Ð±Ð¸Ñ‚Ð°Ñ… SQ3 0

//      ADC1->CR2 |= ADC_CR2_CONT;      // Ð²ÐºÐ»ÑŽÑ‡Ð°ÐµÐ¼ ÐµÑ�Ð»Ð¸ Ð½ÑƒÐ¶Ð½Ð¾ Ð½ÐµÐ¿Ñ€ÐµÑ€Ñ‹Ð²Ð½Ð¾Ðµ Ð¿Ñ€ÐµÐ¾Ð±Ñ€Ð°Ð·Ð¾Ð²Ð°Ð½Ð¸Ðµ Ð¿Ð¾Ñ�Ð»ÐµÐ´Ð¾Ð²Ð°Ñ‚ÐµÐ»ÑŒÐ½Ð¾Ñ�Ñ‚Ð¸ Ð² Ñ†Ð¸ÐºÐ»Ðµ

        ADC1->CR2 |= ADC_CR2_DMA //Ð²ÐºÐ»ÑŽÑ‡Ð°ÐµÐ¼ Ñ€Ð°Ð±Ð¾Ñ‚Ñƒ Ñ� DMA
        | ADC_CR2_EXTTRIG //Ð²ÐºÐ»ÑŽÑ‡Ð°ÐµÐ¼ Ñ€Ð°Ð±Ð¾Ñ‚Ñƒ Ð¾Ñ‚ Ð²Ð½ÐµÑˆÐ½ÐµÐ³Ð¾ Ñ�Ð¾Ð±Ñ‹Ñ‚Ð¸Ñ�
        | ADC_CR2_EXTSEL //Ð²Ñ‹Ð±Ð¸Ñ€Ð°ÐµÐ¼ Ñ‚Ñ€Ð¸Ð³Ð³ÐµÑ€Ð¾Ð¼ Ð·Ð°Ð¿ÑƒÑ�ÐºÐ° Ñ€ÐµÐ³ÑƒÐ»Ñ�Ñ€Ð½Ð¾Ð¹ Ð¿Ð¾Ñ�Ð»ÐµÐ´Ð¾Ð²Ð°Ñ‚ÐµÐ»ÑŒÐ½Ð¾Ñ�Ñ‚Ð¸ Ñ�Ð¾Ð±Ñ‹Ñ‚Ð¸Ðµ SWSTART
        | ADC_CR2_JEXTSEL; // Ð²Ñ‹Ð±Ð¸Ñ€Ð°ÐµÐ¼ Ñ‚Ñ€Ð¸Ð³Ð³ÐµÑ€Ð¾Ð¼ Ð·Ð°Ð¿ÑƒÑ�ÐºÐ° Ð²Ñ‹Ð´ÐµÐ»ÐµÐ½Ð½Ð¾Ð¹ Ð¿Ð¾Ñ�Ð»ÐµÐ´Ð¾Ð²Ð°Ñ‚ÐµÐ»ÑŒÐ½Ð¾Ñ�Ñ‚Ð¸ Ñ�Ð¾Ð±Ñ‹Ñ‚Ð¸Ðµ JSWSTART Ð´Ð°Ð±Ñ‹ Ñ�Ñ‚Ð¸ ÐºÐ°Ð½Ð°Ð»Ñ‹ Ð½Ðµ Ð¾Ñ‚Ð½Ð¸Ð¼Ð°Ð»Ð¸ Ñƒ Ð¼Ðº Ð²Ñ€ÐµÐ¼ÐµÐ½Ð¸
        ADC1->CR1 |= ADC_CR1_SCAN; // Ð²ÐºÐ»ÑŽÑ‡Ð°ÐµÐ¼ Ð°Ð²Ñ‚Ð¾Ð¼Ð°Ñ‚Ð¸Ñ‡ÐµÑ�ÐºÐ¸Ð¹ Ð¿ÐµÑ€ÐµÐ±Ð¾Ñ€ Ð²Ñ�ÐµÑ… ÐºÐ°Ð½Ð°Ð»Ð¾Ð² Ð² Ð¿Ð¾Ñ�Ð»ÐµÐ´Ð¾Ð²Ð°Ñ‚ÐµÐ»ÑŒÐ½Ð¾Ñ�Ñ‚Ð¸

//Ð¼Ð°ÐºÑ€Ð¾Ñ�Ñ‹ Ð´Ð»Ñ� Ð²ÐºÐ»ÑŽÑ‡ÐµÐ½Ð¸Ñ�/Ð²Ñ‹ÐºÐ»ÑŽÑ‡ÐµÐ½Ð¸Ñ� Ð�Ð¦ÐŸ Ñ� DMA
#define ADC_ON ADC1->CR2 |= ADC_CR2_ADON;  \
                                ADC1->CR2 |= ADC_CR2_SWSTART; \
                                DMA1_Channel1->CCR |= DMA_CCR1_TCIE | DMA_CCR1_EN;      //Ð²ÐºÐ»ÑŽÑ‡Ð°ÐµÐ¼ Ð¿Ñ€ÐµÐ¾Ð±Ñ€Ð°Ð·Ð¾Ð²Ð°Ð½Ð¸Ðµ Ð¿Ñ€ÐµÑ€Ñ‹Ð²Ð°Ð½Ð¸Ðµ DMA
#define ADC_OFF ADC->CR2 &= (~(ADC_CR2_ADON)); \
                                DMA1_Channel1->CCR &= (~(DMA_CCR1_TCIE | DMA_CCR1_EN)); //Ð²Ñ‹ÐºÐ»ÑŽÑ‡Ð°ÐµÐ¼ Ð¿Ñ€ÐµÐ¾Ð±Ñ€Ð°Ð·Ð¾Ð²Ð°Ð½Ð¸Ðµ Ð¸ Ð¿Ñ€ÐµÑ€Ñ‹Ð²Ð°Ð½Ð¸Ðµ DMA
        
        return;
}

Ð�Ð°Ñ�Ñ‚Ñ€Ð¾Ð¹ÐºÐ° DMA Ð´Ð»Ñ� Ð�Ð¦ÐŸ
        DMA1_Channel1->CPAR = ADC1_BASE+0x4C; // Ð—Ð°Ð³Ñ€ÑƒÐ¶Ð°ÐµÐ¼ Ð°Ð´Ñ€ÐµÑ� Ñ€ÐµÐ³Ð¸Ñ�Ñ‚Ñ€Ð° DR
        DMA1_Channel1->CMAR = pADC_readbuf_addr->Buf_Addr; //Ð³Ñ€ÑƒÐ·Ð¸Ð¼ Ð°Ð´Ñ€ÐµÑ� Ð±ÑƒÑ„ÐµÑ€Ð° Ð¾Ð±Ð¼ÐµÐ½Ð°
        DMA1_Channel1->CNDTR = 8; //Ð´Ð»Ð¸Ð½Ð° Ð±ÑƒÑ„ÐµÑ€Ð°
        DMA1_Channel1->CCR |= DMA_CCR1_MINC     //Ð¸Ð½ÐºÑ€ÐµÐ¼ÐµÐ½Ñ‚ Ð°Ð´Ñ€ÐµÑ�Ð° Ð¿Ð°Ð¼Ñ�Ñ‚Ð¸
        | DMA_CCR1_PSIZE_0 //Ñ€Ð°Ð·Ð¼ÐµÑ€Ð½Ð¾Ñ�Ñ‚ÑŒ Ð´Ð°Ð½Ð½Ñ‹Ñ… Ð¿ÐµÑ€Ð¸Ñ„ÐµÑ€Ð¸Ð¸ 16 Ð±Ð¸Ñ‚
        | DMA_CCR1_MSIZE_0 //Ñ€Ð°Ð·Ð¼ÐµÑ€Ð½Ð¾Ñ�Ñ‚ÑŒ Ð´Ð°Ð½Ð½Ñ‹Ñ… Ð¿Ð°Ð¼Ñ�Ñ‚Ð¸ 16 bit
        | DMA_CCR1_CIRC;   // Ð·Ð°ÐºÐ¾Ð»ÑŒÑ†ÐµÐ²Ð°Ñ‚ÑŒ Ð±ÑƒÑ„ÐµÑ€
*/
