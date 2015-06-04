#include "../../config.h"

#define SPI1_PORT                   GPIOA
#define SPI1_SCK_PIN                GPIO_Pin_5
#define SPI1_MISO_PIN               GPIO_Pin_6
#define SPI1_MOSI_PIN               GPIO_Pin_7

#define SPI11_PORT                  GPIOB
#define SPI11_SCK_PIN               GPIO_Pin_3
#define SPI11_MISO_PIN              GPIO_Pin_4
#define SPI11_MOSI_PIN              GPIO_Pin_5

#define SPI2_PORT                   GPIOB
#define SPI2_SCK_PIN                GPIO_Pin_13
#define SPI2_MISO_PIN               GPIO_Pin_14
#define SPI2_MOSI_PIN               GPIO_Pin_15

#define SPI3_PORT                   GPIOB
#define SPI3_SCK_PIN                GPIO_Pin_3
#define SPI3_MISO_PIN               GPIO_Pin_4
#define SPI3_MOSI_PIN               GPIO_Pin_5

#define DIO_MODE_SPI                DIO_MODE_AF_PP_HS

/*
        Config 1                Config 2           
SPIx    MOSI    MISO    SCK     MOSI    MISO    SCK     APB
SPI1    PA7     PA6     PA5     PB5     PB4     PB3     2
SPI2    PB15    PB14    PB13                            1
SPI3    PB5     PB4     PB3     PC12    PC11    PC10    1

STM32F0
APB1 clk = SystemCoreClock
APB2 clk = SystemCoreClock

STM32F1
APB1 clk = SystemCoreClock/2
APB2 clk = SystemCoreClock

STM32F4
APB1 clk = SystemCoreClock/4
APB2 clk = SystemCoreClock/2
*/

extern uint32_t SystemCoreClock;

static SPI_TypeDef * hal_spi_port2spi(uint8_t port)
{
    switch(port)
    {
#ifdef SPI1
        case 1:
        case 11:
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
#ifdef RCC_APB2ENR_SPI1EN
        case 1:
            RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
            hal_dio_gpio_cfg(SPI1_PORT, (SPI1_SCK_PIN | SPI1_MISO_PIN | SPI1_MOSI_PIN), DIO_MODE_SPI);
            SPIx = SPI1;
            break;
        case 11:
            RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
            hal_dio_gpio_cfg(SPI11_PORT, (SPI11_SCK_PIN | SPI11_MISO_PIN | SPI11_MOSI_PIN), DIO_MODE_SPI);
            SPIx = SPI1;
            break;
#endif  //  SPI1
#ifdef RCC_APB1ENR_SPI2EN
        case 2:
            RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
            hal_dio_gpio_cfg(SPI2_PORT, (SPI2_SCK_PIN | SPI2_MISO_PIN | SPI2_MOSI_PIN), DIO_MODE_SPI);
            SPIx = SPI2;
            break;
#endif  //  SPI2
#ifdef RCC_APB1ENR_SPI3EN
        case 3:
            RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
            hal_dio_gpio_cfg(SPI3_PORT, (SPI3_SCK_PIN | SPI3_MISO_PIN | SPI3_MOSI_PIN), DIO_MODE_SPI);
            SPIx = SPI3;
            break;
#endif  //  SPI3
        default:
            assert(0);      // Unknown SPI port
            return;
    }
    
    // Calculate pre-scaler
#if (defined __STM32F0XX_H)
    spi_clock = SystemCoreClock;
#elif (defined __STM32F10x_H)
    if(port == 1)
        spi_clock = SystemCoreClock;
    else
        spi_clock = SystemCoreClock/2;
#endif  //CPU

    spi_clock /= 2;

    uint16_t div = 0;
    while((spi_clock > speed) && (div < 7))
    {
        div++;
        spi_clock /= 2;
    }

    // Configure SPI
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
    SPIx->CR1 |= SPI_CR1_SPE;       // SPI enable
}

uint8_t hal_spi_exch8(uint8_t port, uint8_t data)
{
    SPI_TypeDef * SPIx = hal_spi_port2spi(port);
#if (defined __STM32F0XX_H)
    uint32_t spixbase = (uint32_t)SPIx + 0x0C;
    *(__IO uint8_t *)spixbase = data;
    while((SPIx->SR & SPI_SR_RXNE) == 0);
    data = *(__IO uint8_t *)spixbase;
    return data;
#else
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
