#include "../../config.h"

#ifdef EXTTWI_USED

#include "../../EXT/exttwi.h"

// Global variable defined in exttwi.c
extern volatile TWI_QUEUE_t * pTwi_exchange;

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
    }
    else
    {
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

    uint8_t counter = 0;
    uint8_t access = pTwi_exchange->frame.access;

    if((GPIOB->IDR & (GPIO_IDR_6 | GPIO_IDR_7)) != (GPIO_IDR_6 | GPIO_IDR_7))
        return;

    pTwi_exchange->frame.access |= TWI_BUSY;

    // Clear Flags
    I2C1->ICR |= (I2C_ICR_ARLOCF | I2C_ICR_BERRCF | I2C_ICR_STOPCF | I2C_ICR_NACKCF);

    if(access & TWI_WRITE)
    {
        I2C1->CR2 = (uint32_t)(pTwi_exchange->frame.address << 1) | // Slave address
                    ((uint32_t)(pTwi_exchange->frame.write) << 16); // Bytes to send

        if((access & TWI_READ) == 0)
            I2C1->CR2 |= I2C_CR2_AUTOEND;

        // Send Start & address
        I2C1->CR2 |= I2C_CR2_START;
        while((I2C1->ISR & I2C_ISR_BUSY) == 0);

        while(((I2C1->ISR & I2C_ISR_TXIS) == 0) &&      // Data register is not empty
              ((I2C1->ISR & I2C_ISR_BUSY) != 0))
        {
            if(I2C1->ISR & I2C_ISR_NACKF)               // NACK received
            {
                pTwi_exchange->frame.access |= TWI_SLANACK;
                return;
            }
        }
        
        counter = 0;
        while(counter < pTwi_exchange->frame.write)
        {
            uint32_t isr = I2C1->ISR & (I2C_ISR_BUSY | I2C_ISR_NACKF | I2C_ISR_TXIS);
            if(isr == (I2C_ISR_BUSY | I2C_ISR_TXIS))    // Data Buffer empty
                I2C1->TXDR = pTwi_exchange->frame.data[counter++];
            else if(isr != I2C_ISR_BUSY)                // NACK received, or bus error
                break;
        }

        pTwi_exchange->frame.write = counter;
    }
    
    if(access & TWI_READ)
    {
        I2C1->CR2 = ((uint32_t)(pTwi_exchange->frame.address << 1) |    // Slave address
                    ((uint32_t)pTwi_exchange->frame.read << 16) |       // Bytes to read
                    I2C_CR2_RD_WRN | I2C_CR2_AUTOEND);                  // Read request

        // Send Start & address
        I2C1->CR2 |= I2C_CR2_START;
        while((I2C1->ISR & I2C_ISR_BUSY) == 0);
        
        
        counter = 0;
        while(counter < pTwi_exchange->frame.read)
        {
            uint32_t isr = I2C1->ISR & (I2C_ISR_BUSY | I2C_ISR_NACKF | I2C_ISR_RXNE);
            if(isr == (I2C_ISR_BUSY | I2C_ISR_RXNE))                    // Data ready
            {
                pTwi_exchange->frame.data[counter++] = I2C1->RXDR;
            }
            else if(isr != I2C_ISR_BUSY)
                break;
        }
        pTwi_exchange->frame.read = counter;
    }

    pTwi_exchange->frame.access &= ~(TWI_BUSY | TWI_READ | TWI_WRITE);
    pTwi_exchange->frame.access |= TWI_RDY;
}

/*
void hal_twi_tick(void)
{


    if(TWIM_SCL_STAT() != 0)    // Bus Free
    {


        TWCR = (1<<TWEN) |                      // TWI Interface enabled.
               (1<<TWIE) | (1<<TWINT) |         // Enable TWI Interrupt and clear the flag.
               (1<<TWSTA);                      // Initiate a START condition.
    }
}

ISR(TWI_vect)
{
    static uint8_t twi_ptr;

    switch(TW_STATUS)
    {
        // Master
        case TW_START:                              // start condition transmitted
        case TW_REP_START:                          // repeated start condition transmitted
            twi_ptr = 0;
            
            if(pTwi_exchange->access & TWI_WRITE)
                TWDR = (pTwi_exchange->address<<1);
            else
                TWDR = (pTwi_exchange->address<<1) | TW_READ;
            TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
            break;
        // Master Send
        case TW_MT_SLA_NACK:                        // SLA+W transmitted, NACK received
        case TW_MR_SLA_NACK:                        // SLA+R transmitted, NACK received
            pTwi_exchange->read = 0;
            pTwi_exchange->access |= TWI_SLANACK;
            TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE); // Send Stop
            break;
        case TW_MT_SLA_ACK:                         // SLA+W transmitted, ACK received
        case TW_MT_DATA_ACK:                        // data transmitted, ACK received
            if(twi_ptr < pTwi_exchange->write)
            {
                TWDR = pTwi_exchange->data[twi_ptr];
                twi_ptr++;
                TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
                break;
            }
            // else, ACK received but should be NACK
        case TW_MT_DATA_NACK:                       // data transmitted, NACK received
            pTwi_exchange->write = twi_ptr;
            pTwi_exchange->access &= ~(TWI_WRITE | TWI_BUSY);
            if((pTwi_exchange->access & TWI_READ) == 0)
            {
                pTwi_exchange->access = 0;
                TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE); // Send Stop
            }
            else
                TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWINT) | (1<<TWSTA); // Send RepSTART
            break;
        // Master Receive
        case TW_MR_DATA_ACK:                        // Data byte has been received and ACK transmitted
            pTwi_exchange->data[twi_ptr++] = TWDR;
        case TW_MR_SLA_ACK:                         // SLA+R has been transmitted and ACK received
            if((twi_ptr + 1) < pTwi_exchange->read)
            {
                TWCR = (1<<TWEN) |                  // TWI Interface enabled
                       (1<<TWIE) | (1<<TWINT) |     // Enable TWI Interrupt and clear the flag to read next byte
                       (1<<TWEA);                   // Send ACK after reception
            }
            else                                    // Send NACK after next reception
            {
                TWCR = (1<<TWEN) |                  // TWI Interface enabled
                       (1<<TWIE) | (1<<TWINT);      // Enable TWI Interrupt and clear the flag to read last byte
            }
            break;
        case TW_MR_DATA_NACK:                       // Data byte has been received and NACK transmitted
            pTwi_exchange->data[twi_ptr++] = TWDR;
            pTwi_exchange->read = twi_ptr;
            pTwi_exchange->access &= ~(TWI_READ | TWI_BUSY);
            pTwi_exchange->access |= TWI_RDY;
            TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE); // Send Stop
            break;
        default:                                    // Error
            TWCR &= ~((1<<TWEN) | (1<<TWSTO));
            TWCR |= (1<<TWINT) | (1<<TWEN);
            pTwi_exchange->write = twi_ptr;
            pTwi_exchange->read = 1;
            pTwi_exchange->data[0] = TWSR;
            pTwi_exchange->access |= TWI_ERROR;
            break;
    }
}
*/
#endif  //  EXTTWI_USED
