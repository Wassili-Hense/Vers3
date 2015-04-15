#include "../../config.h"

#ifdef EXTAIN_USED

static const uint8_t hal_ainBase2Apin[] = EXTAIN_BASE_2_APIN;

void hal_ain_select(uint8_t apin, uint8_t unused)
{
    uint32_t mux;
    mux = hal_ainBase2Apin[apin];
    mux = (1 << mux);

    // Start Conversion
}

int16_t hal_ain_get(void)
{
    int16_t retval = ADC>>1;
    ADCSRA |= (1<<ADSC);
    return retval;
}
#endif  //EXTAIN_USED

/*
STM32F1

void ADC_init(void)
{
// настройка ADC1
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //  такты на ADC1

        ADC1->SMPR2 |= ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1 
        | ADC_SMPR2_SMP2 | ADC_SMPR2_SMP3 | ADC_SMPR2_SMP4 
        | ADC_SMPR2_SMP5 | ADC_SMPR2_SMP6 | ADC_SMPR2_SMP7;       // количество циклов преобразования 239.5

        ADC1->SQR1 |= ADC_SQR1_L_2 | ADC_SQR1_L_1 | ADC_SQR1_L_0; // длина последовательности регулярных каналов = 8;
        // добавление в последовательность каналов
        ADC1->SQR2 |= ADC_SQR2_SQ8_2 | ADC_SQR2_SQ8_1 | ADC_SQR2_SQ8_0 //канал 7
        | ADC_SQR2_SQ7_2 | ADC_SQR2_SQ7_1;                             // канал 6

        ADC1->SQR3 |=  ADC_SQR3_SQ6_2 | ADC_SQR3_SQ6_0                 //канал 5
        | ADC_SQR3_SQ5_2                                               //канал 4
        | ADC_SQR3_SQ4_1 | ADC_SQR3_SQ4_0                              //канал 3
        | ADC_SQR3_SQ3_1                                               //канал 2
        | ADC_SQR3_SQ2_0;                                              // канал 1
        //канал 0 включен по умолчанию т.к. в младших битах SQ3 0

//      ADC1->CR2 |= ADC_CR2_CONT;      // включаем если нужно непрерывное преобразование последовательности в цикле

        ADC1->CR2 |= ADC_CR2_DMA //включаем работу с DMA
        | ADC_CR2_EXTTRIG //включаем работу от внешнего события 
        | ADC_CR2_EXTSEL //выбираем триггером запуска регулярной последовательности событие SWSTART
        | ADC_CR2_JEXTSEL; // выбираем триггером запуска выделенной последовательности событие JSWSTART дабы эти каналы не отнимали у мк времени
        ADC1->CR1 |= ADC_CR1_SCAN; // включаем автоматический перебор всех каналов в последовательности

//макросы для включения/выключения АЦП с DMA
#define ADC_ON ADC1->CR2 |= ADC_CR2_ADON;  \
                                ADC1->CR2 |= ADC_CR2_SWSTART; \
                                DMA1_Channel1->CCR |= DMA_CCR1_TCIE | DMA_CCR1_EN;      //включаем преобразование прерывание DMA
#define ADC_OFF ADC->CR2 &= (~(ADC_CR2_ADON)); \
                                DMA1_Channel1->CCR &= (~(DMA_CCR1_TCIE | DMA_CCR1_EN)); //выключаем преобразование и прерывание DMA
        
        return;
}

Настройка DMA для АЦП
        DMA1_Channel1->CPAR = ADC1_BASE+0x4C; // Загружаем адрес регистра DR
        DMA1_Channel1->CMAR = pADC_readbuf_addr->Buf_Addr; //грузим адрес буфера обмена
        DMA1_Channel1->CNDTR = 8; //длина буфера 
        DMA1_Channel1->CCR |= DMA_CCR1_MINC     //инкремент адреса памяти
        | DMA_CCR1_PSIZE_0 //размерность данных периферии 16 бит
        | DMA_CCR1_MSIZE_0 //размерность данных памяти 16 bit
        | DMA_CCR1_CIRC;   // закольцевать буфер
*/