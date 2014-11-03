#include "../../config.h"

#ifdef UART_PHY

#include <avr/interrupt.h>

#define HAL_SIZEOF_UART_RX_FIFO     16      // Should be 2^n
#define HAL_SIZEOF_UART_TX_FIFO     16      // Should be 2^n

#if (!(defined USART_USE_PORT) || (USART_USE_PORT == 0))            // USART0

#define UARTx                       &UCSR0A

#ifndef USART_RX_vect
#define USART_RX_vect               USART0_RX_vect
#endif  //  USART_RX_vect

#ifndef USART_UDRE_vect
#define USART_UDRE_vect             USART0_UDRE_vect
#endif  //  USART_UDRE_vect

#elif ((USART_USE_PORT == 1) && (defined UCSR1A))                   // USART1

#define UARTx                       &UCSR1A

#define USART_RX_vect               USART1_RX_vect
#define USART_UDRE_vect             USART1_UDRE_vect

#elif ((USART_USE_PORT == 2) && (defined UCSR2A))                   // USART2

#define UARTx                       &UCSR2A

#define USART_RX_vect               USART2_RX_vect
#define USART_UDRE_vect             USART2_UDRE_vect

#elif ((USART_USE_PORT == 3) && (defined UCSR3A))                   // USART3

#define UARTx                       &UCSR3A

#define USART_RX_vect               USART3_RX_vect
#define USART_UDRE_vect             USART3_UDRE_vect

#else
#error unknown USART configuration
#endif  //  USART_USE_PORT

#define UCSRB                       *(uint8_t *)(UARTx + 1)
#define UCSRC                       *(uint8_t *)(UARTx + 2)
#define UBRRL                       *(uint8_t *)(UARTx + 4)
#define UBRRH                       *(uint8_t *)(UARTx + 5)
#define UDR                         *(uint8_t *)(UARTx + 6)

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

    UBRRH = (((F_CPU/16/38400) - 1)>>8);
    UBRRL = (((F_CPU/16/38400) - 1) & 0xFF);

    UCSRB = ((1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0));
    UCSRC = (3<<UCSZ00);

    hal_uart_rx_head = 0;
    hal_uart_rx_tail = 0;
    hal_uart_tx_head = 0;
    hal_uart_tx_tail = 0;
}

bool hal_uart_tx_busy(void)
{
    if(hal_uart_tx_head == hal_uart_tx_tail)
        return false;

    if((UCSRB & (1<<UDRIE0)) == 0)
    {
        UDR = hal_uart_tx_fifo[hal_uart_tx_tail];
        hal_uart_tx_tail++;
        hal_uart_tx_tail &= (uint8_t)(HAL_SIZEOF_UART_TX_FIFO - 1);
        UCSRB |= (1<<UDRIE0);
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
    uint8_t data = UDR;
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
        UCSRB &= ~(1<<UDRIE0);
        return;
    }

    UDR = hal_uart_tx_fifo[hal_uart_tx_tail];
    hal_uart_tx_tail++;
    hal_uart_tx_tail &= (uint8_t)(HAL_SIZEOF_UART_TX_FIFO - 1);
}

#endif  //  UART_PHY
