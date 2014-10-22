#include "../../config.h"

#ifdef TWIMM_PHY

#include <avr/interrupt.h>
#include <util/twi.h>

// Multimaster TWI

enum
{
    TWIMM_STAT_FREE = 0,
    TWIMM_STAT_RX_BUSY,
    TWIMM_STAT_RX_RDY,
    
    TWIMM_STAT_TX_BUSY
}e_TWIMM_STAT;


typedef struct
{
    uint8_t addr;
    uint8_t len;
    uint8_t own_addr;
    uint8_t data[MQTTSN_MSG_SIZE];
}S_TWIMM_t;

static S_TWIMM_t        vTWIMM;
static volatile uint8_t TWIMM_Status;
static uint16_t         TWIMM_BusBusyCnt;

void hal_twimm_init_hw(uint8_t addr)
{
    PORTB |= (1<<PORTB0);
    DDRB |= (1<<PORTB0);

    TWCR = (1<<TWINT);

    vTWIMM.own_addr = addr;
    TWIMM_BusBusyCnt = 0;

    TWBR = (((F_CPU/100000UL)-16)/2);   // 100kHz
    TWAR = (addr<<1) | (1<<TWGCE);
    TWDR = 0xFF;
    TWCR = (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
}

bool hal_twimm_can_send(void)
{
    return ((TWIMM_Status == TWIMM_STAT_FREE) && (TWIMM_SCL_STAT() != 0));
}

void hal_twimm_send(MQ_t *pBuf)
{
    TWIMM_Status = TWIMM_STAT_TX_BUSY;
    PORTB &= ~(1<<PORTB0);

    vTWIMM.addr = ((pBuf->phy1addr[0])<<1) | TW_WRITE;
    vTWIMM.len  = pBuf->Length;
    memcpy(vTWIMM.data, pBuf->raw, vTWIMM.len);
    
    if(TWIMM_SCL_STAT() != 0)                   // Bus free
        TWCR = (1<<TWEN) |                      // TWI Interface enabled.
               (1<<TWIE) | (1<<TWINT) |         // Enable TWI Interrupt and clear the flag.
               (1<<TWSTA);                      // Initiate a START condition.
}

MQ_t * hal_twimm_get(void)
{
    if(TWIMM_Status == TWIMM_STAT_RX_RDY)
    {
        MQ_t * pBuf = mqAlloc(sizeof(MQ_t));
        if(pBuf != NULL)
        {
            pBuf->phy1addr[0] = vTWIMM.addr;
            pBuf->Length = vTWIMM.len;
            memcpy(pBuf->raw, vTWIMM.data, vTWIMM.len);
        }
        TWIMM_Status = TWIMM_STAT_FREE;
        return pBuf;
    }
    else if((TWIMM_Status != TWIMM_STAT_FREE) || (TWCR & (1<<TWINT)))
    {
        TWIMM_BusBusyCnt--;
        if(TWIMM_BusBusyCnt == 0)
        {
            TWCR = (1<<TWINT);
            TWDR = 0xFF;
            TWCR = (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
            TWIMM_Status = TWIMM_STAT_FREE;
        }
    }
    else
        TWIMM_BusBusyCnt = 0;

    return NULL;
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
            TWDR = vTWIMM.addr;
            TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
            break;
        case TW_MT_SLA_ACK:                         // SLA+W transmitted, ACK received
        case TW_MT_DATA_ACK:                        // data transmitted, ACK received
            if(twi_ptr <= vTWIMM.len)
            {
                if(twi_ptr == 0)
                    TWDR = vTWIMM.own_addr;
                else if(twi_ptr == 1)
                {
                    TWDR = vTWIMM.len;
                    vTWIMM.len++;
                }
                else
                    TWDR = vTWIMM.data[twi_ptr - 2];
                twi_ptr++;
                TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
                break;
            }
            // else, ACK received but should be NACK
        case TW_MT_SLA_NACK:                        // SLA+W transmitted, NACK received
        case TW_MT_DATA_NACK:                       // data transmitted, NACK received
            TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE); // Send Stop
            PORTB |= (1<<PORTB0);
            TWIMM_Status = TWIMM_STAT_FREE;
            break;

        // Slave
        case TW_SR_SLA_ACK:                         // SLA+W received, ACK returned
        case TW_SR_GCALL_ACK:                       // general call received, ACK returned
            PORTB &= ~(1<<PORTB0);
            twi_ptr = 0;
            vTWIMM.len = 0;
            TWIMM_Status = TWIMM_STAT_RX_BUSY;
            TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
            break;
        case TW_SR_DATA_ACK:                        // data received, ACK returned
        case TW_SR_GCALL_DATA_ACK:                  // general call data received, ACK returned
            if(twi_ptr == 0)                        // Receive Sender Address
            {
                vTWIMM.addr = TWDR;
                twi_ptr = 1;
            }
            else if(twi_ptr == 1)                   // Receive data length
            {
                vTWIMM.len = TWDR;
                twi_ptr = 2;
            }
            else
            {
                vTWIMM.data[twi_ptr - 2] = TWDR;
                twi_ptr++;

                if(twi_ptr > vTWIMM.len)
                {
                    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
                    break;
                }
            }
            TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
            break;
        case TW_SR_DATA_NACK:                       // data received, NACK returned
        case TW_SR_GCALL_DATA_NACK:                 // general call data received, NACK returned
            vTWIMM.data[twi_ptr - 2] = TWDR;
            TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
            TWIMM_Status = TWIMM_STAT_RX_RDY;
            PORTB |= (1<<PORTB0);
            break;

        default:                                    // Error
            TWCR &= ~((1<<TWEN) | (1<<TWSTO));
            TWCR |= (1<<TWINT) | (1<<TWEN);
            TWIMM_Status = TWIMM_STAT_FREE;
            break;
    }
}

#endif  //  TWIMM_PHY