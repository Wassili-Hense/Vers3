#include "../../config.h"

#ifdef EXTTWI_USED

#include "../../EXT/exttwi.h"

// Global variable defined in exttwi.c
extern volatile TWI_QUEUE_t * pTwi_exchange;
static volatile uint8_t twi_pnt = 0;

// I2C1, PB6 - SCL, PB7 - SDA

bool hal_twi_configure(uint8_t enable)
{
    if(enable)
    {
        // Check GPIO
        hal_dio_gpio_cfg(GPIOB, (GPIO_Pin_6 | GPIO_Pin_7), DIO_MODE_IN_FLOAT);  // Configure GPIO as floating inputs
        if((GPIOB->IDR & (GPIO_IDR_6 | GPIO_IDR_7)) != (GPIO_IDR_6 | GPIO_IDR_7))
            return false;

        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;     // Enable I2C clock
        RCC->CFGR3 |= RCC_CFGR3_I2C1SW;         // Use SysClk for I2C CLK
        hal_dio_gpio_cfg(GPIOB, (GPIO_Pin_6 | GPIO_Pin_7), DIO_MODE_TWI);       // Configure GPIO

        // Reset I2C1
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

        // Set Timings, RM0091, page 542
        // I2C Clock = 48MHz
        // I2C Bus = 100 KHz
        I2C1->TIMINGR |= 0xB0420F13;

        // Enable I2C
        I2C1->CR1 = I2C_CR1_PE;

        NVIC_SetPriority(I2C1_IRQn, 3);
        NVIC_EnableIRQ(I2C1_IRQn);
    }
    else
    {
        NVIC_DisableIRQ(I2C1_IRQn);

        // Reset I2C1
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

        hal_dio_gpio_cfg(GPIOB, (GPIO_Pin_6 | GPIO_Pin_7), DIO_MODE_IN_FLOAT);  // Release GPIO
        RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;    // Disable clock
    }

    return true;
}

void hal_twi_tick(void)
{
    assert(pTwi_exchange != NULL);

    uint8_t access = pTwi_exchange->frame.access;

    // WatchDog
    static uint8_t twi_wd = 0;
    if((access & (TWI_ERROR | TWI_SLANACK | TWI_WD | TWI_RDY | TWI_BUSY)) == 0)
    {
        twi_wd = 0;
    }
    else
    {
        twi_wd--;
        if(twi_wd == 0) // Timeout
        {
            pTwi_exchange->frame.access |= TWI_WD;
            if(access & TWI_BUSY)
            {
                I2C1->CR1 &= ~I2C_CR1_PE;       // Disable I2C
            }
        }
    }
    // End WatchDog

    if(access & TWI_BUSY)
        return;

    if((I2C1->CR1 & I2C_CR1_PE) == 0)
    {
        I2C1->CR1 = I2C_CR1_PE;                             // Enable I2C
        return;
    }

    uint32_t isr = I2C1->ISR;
    if(isr & I2C_ISR_BUSY)                                  // Bus Busy
    {
        return;
    }

    if(isr & (I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR))   // Bus Error
    {
        I2C1->CR1 &= ~I2C_CR1_PE;                           // Disable I2C
        pTwi_exchange->frame.access |= TWI_ERROR;
        return;
    }

    I2C1->ICR |= I2C_ICR_STOPCF | I2C_ICR_NACKCF;                   // Clear STOP & NACK flags.
    I2C1->CR1 |= (I2C_CR1_ERRIE |                                   // Enable Interrupts on: Erros,
                  I2C_CR1_TCIE |                                    // Transfer complete,
                  I2C_CR1_STOPIE |                                  // Stop Detection,
                  I2C_CR1_NACKIE);                                  // NACK received

    twi_pnt = 0;

    // ToDo Write Only
    if(access & TWI_WRITE)
    {
        I2C1->CR2 = (uint32_t)(pTwi_exchange->frame.address << 1) | // Slave address
                    ((uint32_t)(pTwi_exchange->frame.write) << 16); // Bytes to send

        if((access & TWI_READ) == 0)
            I2C1->CR2 |= I2C_CR2_AUTOEND;

        if(pTwi_exchange->frame.write > 0)
            I2C1->TXDR = pTwi_exchange->frame.data[twi_pnt++];

        I2C1->CR1 |= I2C_CR1_TXIE;                                  // Interrupt on Tx Buffer empty
    }
    else
    {
        I2C1->CR2 = ((uint32_t)(pTwi_exchange->frame.address << 1) |    // Slave address
                    ((uint32_t)(pTwi_exchange->frame.read) << 16) |     // Bytes to read
                    I2C_CR2_RD_WRN | I2C_CR2_AUTOEND);                  // Read request with stop

        I2C1->CR1 |= I2C_CR1_RXIE;
    }

    // Send Start & Address
    I2C1->CR2 |= I2C_CR2_START;

    pTwi_exchange->frame.access |= TWI_BUSY;
}

void I2C1_IRQHandler(void)
{
    uint32_t isr = I2C1->ISR;

    if(isr & (I2C_ISR_OVR | I2C_ISR_ARLO | I2C_ISR_BERR))   // Bus Error
    {
        I2C1->CR1 &= ~I2C_CR1_PE;       // Disable I2C
        pTwi_exchange->frame.access |= TWI_ERROR;

    }
    else if(isr & I2C_ISR_TC)                                           // Transfer complete
    {
        I2C1->CR1 &= ~I2C_CR1_TXIE;                                     // Disable Tx Interrupt
        I2C1->CR2 = ((uint32_t)(pTwi_exchange->frame.address << 1) |    // Slave address
                     ((uint32_t)(pTwi_exchange->frame.read) << 16) |    // Bytes to read
                      I2C_CR2_RD_WRN | I2C_CR2_AUTOEND);                // Read request with stop

        I2C1->CR1 |= I2C_CR1_RXIE;                                      // Enable Rx interrupt

        // Send Repeat Start & Address
        I2C1->CR2 |= I2C_CR2_START;
    }
    else if(isr & I2C_ISR_STOPF)                                    // Stop received
    {
        I2C1->ICR |= I2C_ICR_STOPCF;                                // Clear Stop Flag

        if(pTwi_exchange->frame.access & TWI_WRITE)
        {
            pTwi_exchange->frame.access &= ~TWI_WRITE;
            
            if(I2C1->CR1 & I2C_CR1_TXIE)                            // Last Stop on Tx
            {
                I2C1->CR1 = I2C_CR1_PE;                             // Disable Interrupts
                pTwi_exchange->frame.access &= ~TWI_BUSY;
                pTwi_exchange->frame.access |= TWI_RDY;             // Transaction complete
            }
            pTwi_exchange->frame.write = twi_pnt;
            twi_pnt = 0;
        }
        else                                                    // Stop on Rx
        {
            I2C1->CR1 = I2C_CR1_PE;                             // Disable Interrupts
            pTwi_exchange->frame.data[twi_pnt] = I2C1->RXDR;
            pTwi_exchange->frame.access &= ~(TWI_READ | TWI_BUSY);
            pTwi_exchange->frame.access |= TWI_RDY;             // Transaction complete
            pTwi_exchange->frame.read = twi_pnt;
        }
    }
    else if(isr & I2C_ISR_NACKF)                            // NACK received
    {
        I2C1->ICR |= I2C_ICR_NACKCF;                        // Clear NACK Flag
        I2C1->CR1 = I2C_CR1_PE;                             // Disable Interrupts

        pTwi_exchange->frame.access &= ~(TWI_WRITE | TWI_READ);
        pTwi_exchange->frame.write = 0;
        pTwi_exchange->frame.read = 0;

        pTwi_exchange->frame.access |= TWI_SLANACK;
    }
    else if((I2C1->CR1 & I2C_CR1_TXIE) && (isr & I2C_ISR_TXIS)) // Transmit buffer empty
    {
        I2C1->TXDR = pTwi_exchange->frame.data[twi_pnt++];
    }
    else if((I2C1->CR1 & I2C_CR1_RXIE) && (isr & I2C_ISR_RXNE))     // Data received
    {
         pTwi_exchange->frame.data[twi_pnt++] = I2C1->RXDR;
    }
    else                                                            // Unknown state
    {
        I2C1->CR1 &= ~I2C_CR1_PE;                                   // Disable I2C
    }
}

#endif  //  EXTTWI_USED
