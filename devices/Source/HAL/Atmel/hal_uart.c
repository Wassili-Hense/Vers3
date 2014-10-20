#include "../../config.h"

#ifdef UART_PHY

#include <avr/interrupt.h>

#define HAL_SIZEOF_UART_RX_FIFO     16      // Should be 2^n
#define HAL_SIZEOF_UART_TX_FIFO     16      // Should be 2^n

#if (USART_USE_PORT == 1)  // USART1

#define UART_RX_DATA                UDR0
#define UART_TX_DATA                UDR0

#ifndef USART_RX_vect
#define USART_RX_vect               USART0_RX_vect
#endif  //  USART_RX_vect

#ifndef USART_UDRE_vect
#define USART_UDRE_vect             USART0_UDRE_vect
#endif  //  USART_UDRE_vect

#define UART_TX_DISABLE_INT()       UCSR0B &= ~(1<<UDRIE0)
#define UART_TX_ENABLE_INT()        UCSR0B |= (1<<UDRIE0)
#define IS_UART_TX_INT_ENABLED()    (UCSR0B & (1<<UDRIE0))

#endif  //  USART_USE_PORT

static uint8_t          hal_uart_rx_fifo[HAL_SIZEOF_UART_RX_FIFO];
static volatile uint8_t hal_uart_rx_head;
static uint8_t          hal_uart_rx_tail;

static uint8_t          hal_uart_tx_fifo[HAL_SIZEOF_UART_TX_FIFO];
static uint8_t          hal_uart_tx_head;
static volatile uint8_t hal_uart_tx_tail;

void hal_uart_init_hw(void)
{
    UART_PORT |= (1<<UART_RX_PIN) | (1<<UART_TX_PIN);
    UART_DDR |= (1<<UART_TX_PIN);
    UART_DDR &= ~(1<<UART_RX_PIN);

    UBRR0H = (((F_CPU/16/38400) - 1)>>8);
    UBRR0L = (((F_CPU/16/38400) - 1) & 0xFF);

    UCSR0B = ((1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0));
    UCSR0C = (3<<UCSZ00);

    hal_uart_rx_head = 0;
    hal_uart_rx_tail = 0;
    hal_uart_tx_head = 0;
    hal_uart_tx_tail = 0;
}

bool hal_uart_tx_busy(void)
{
    if(hal_uart_tx_head == hal_uart_tx_tail)
        return false;

    if(IS_UART_TX_INT_ENABLED() == 0)
    {
        UART_TX_DATA = hal_uart_tx_fifo[hal_uart_tx_tail];
        hal_uart_tx_tail++;
        hal_uart_tx_tail &= (uint8_t)(HAL_SIZEOF_UART_TX_FIFO - 1);
        UART_TX_ENABLE_INT();
        return false;
    }

    return (((hal_uart_tx_head + 1) & (uint8_t)(HAL_SIZEOF_UART_TX_FIFO - 1)) == hal_uart_tx_tail);
}

void hal_uart_send(uint8_t data)
{
    uint8_t tmp_head = (hal_uart_tx_head + 1) & (uint8_t)(HAL_SIZEOF_UART_TX_FIFO - 1);
    if(tmp_head == hal_uart_tx_tail)        // Overflow
        return;

    hal_uart_tx_fifo[hal_uart_tx_head] = data;
    hal_uart_tx_head = tmp_head;
}

bool hal_uart_get(uint8_t * pData)
{
    if(hal_uart_rx_head == hal_uart_rx_tail)
        return false;
    
    *pData = hal_uart_rx_fifo[hal_uart_rx_tail];
    hal_uart_rx_tail++;
    hal_uart_rx_tail &= (uint8_t)(HAL_SIZEOF_UART_RX_FIFO - 1);

    return true;
}

ISR(USART_RX_vect)
{
    uint8_t data = UART_RX_DATA;
    uint8_t tmp_head = (hal_uart_rx_head + 1) & (uint8_t)(HAL_SIZEOF_UART_RX_FIFO - 1);
    if(tmp_head == hal_uart_rx_tail)        // Overflow
        return;

    hal_uart_rx_fifo[hal_uart_rx_head] = data;
    hal_uart_rx_head = tmp_head;
}

ISR(USART_UDRE_vect)
{
    if(hal_uart_tx_head == hal_uart_tx_tail)
    {
        UART_TX_DISABLE_INT();
        return;
    }

    UART_TX_DATA = hal_uart_tx_fifo[hal_uart_tx_tail];
    hal_uart_tx_tail++;
    hal_uart_tx_tail &= (uint8_t)(HAL_SIZEOF_UART_TX_FIFO - 1);
}

#endif  //  UART_PHY
