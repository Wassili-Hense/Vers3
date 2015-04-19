#include "../../config.h"

extern uint32_t SystemCoreClock;

static SPI_TypeDef * hal_spi_port2spi(uint8_t port)
{
    switch(port)
    {
#ifdef SPI1
        case 1:
            return SPI1;
#endif  //  SPI1
#ifdef SPI2
        case 2:
            return SPI2;
#endif  //  SPI2
#ifdef SPI3
        case 3:
            return SPI3;
#endif  //  SPI3
        default:
            assert(0);      // Unknown SPI port
    }
}

void hal_spi_cfg(uint8_t port, uint8_t mode, uint32_t speed)
{
    SPI_TypeDef * SPIx;
    uint32_t spi_clock;

    switch(port)
    {
#ifdef SPI1
        case 1:
            SPIx = SPI1;
            RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
#if (defined __STM32F0XX_H)
            spi_clock = SystemCoreClock;
#elif (defined __STM32F10x_H)
            spi_clock = SystemCoreClock;
#endif  //CPU
            break;
#endif  //  SPI1
#ifdef SPI2
        case 2:
            SPIx = SPI2;
            RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
#if (defined __STM32F0XX_H)
            spi_clock = SystemCoreClock;
#elif (defined __STM32F10x_H)
            spi_clock = SystemCoreClock/2;
#endif  //CPU
            break;
#endif  //  SPI2
#ifdef SPI3
        case 3:
            SPIx = SPI3;
            RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
#if (defined __STM32F0XX_H)
            spi_clock = SystemCoreClock;
#elif (defined __STM32F10x_H)
            spi_clock = SystemCoreClock/2;
#endif  //CPU
            break;
#endif  //  SPI3
        default:
            assert(0);      // Unknown SPI port
            return;
    }

    // Calculate prescaler
    assert(speed > 0);
    uint32_t prescaler = (spi_clock + (speed/2))/speed;
    assert(prescaler <= 256);

    uint16_t div = 0;
    while(prescaler > 2)
    {
        div++;
        prescaler /= 2;
    }
    
    SPIx->CR1 = (uint16_t)(SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR) | (div<<3);
#if (defined __STM32F0XX_H)
    if(mode & HAL_SPI_16B)
        SPIx->CR2 = (uint16_t)(SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0); // 16 bit
    else
        SPIx->CR2 = (uint16_t)(SPI_CR2_FRXTH | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0); // 8 bit
#elif (defined __STM32F10x_H)
    if(mode & HAL_SPI_16B)
        SPIx->CR1 |= SPI_CR1_DFF;
#else
    #error unknown CPU
#endif  // CPU
    if(mode & HAL_SPI_LSB)
        SPIx->CR1 |= SPI_CR1_LSBFIRST;
    
    if(mode & 1)        // Mode 1/3
        SPIx->CR1 |= SPI_CR1_CPHA;
    if(mode & 2)        // Mode 2/3
        SPIx->CR1 |= SPI_CR1_CPOL;

    SPIx->CRCPR =  7;
    SPIx->I2SCFGR &= (uint16_t)~((uint16_t)SPI_I2SCFGR_I2SMOD);
    SPIx->CR1 |= SPI_CR1_SPE;       // SPIc enable
}

uint8_t hal_spi_exch8(uint8_t port, uint8_t data)
{
    SPI_TypeDef * SPIx = hal_spi_port2spi(port);

#if   (defined __STM32F0XX_H)
    uint32_t spixbase = (uint32_t)SPIx + 0x0C;
    *(__IO uint8_t *)spixbase = data;
    while((SPIx->SR & SPI_SR_RXNE) == 0);
    return *(__IO uint8_t *)spixbase;
#elif (defined __STM32F10x_H)
    SPIx->DR = data;
    while((SPIx->SR & SPI_SR_RXNE) == 0);
    return (SPIx->DR & 0xFF);
#endif
}

uint16_t hal_spi_exch16(uint8_t port, uint16_t data)
{
    SPI_TypeDef * SPIx = hal_spi_port2spi(port);
    SPIx->DR = data;
    while((SPIx->SR & SPI_SR_RXNE) == 0);
    return SPIx->DR;
}

/*
STM32F4
        pack 1                  pack 2                  pack 3
SPIx    MOSI    MISO    SCK     MOSI    MISO    SCK     MOSI    MISO    SCK     APB
SPI1    PA7     PA6     PA5     PB5     PB4     PB3                             2
SPI2    PC3     PC2     PB10    PB15    PB14    PB13    PI3     PI2     PI0     1
SPI3    PB5     PB4     PB3     PC12    PC11    PC10                            1
SPI4    PE6     PE5     PE2     PE14    PE13    PE12                            2
SPI5    PF9     PF8     PF7     PF11    PH7     PH6                             2
SPI6    PG14    PG12    PG13                                                    2

APB1 clk = FCPU/4
APB2 clk = FCPU/2
*/