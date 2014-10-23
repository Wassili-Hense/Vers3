#include "../../config.h"

#ifdef UART_PHY

#define HAL_SIZEOF_UART_RX_FIFO     16      // Should be 2^n
#define HAL_SIZEOF_UART_TX_FIFO     16      // Should be 2^n

static uint8_t          hal_uart_rx_fifo[HAL_SIZEOF_UART_RX_FIFO];
static volatile uint8_t hal_uart_rx_head;
static uint8_t          hal_uart_rx_tail;

static uint8_t          hal_uart_tx_fifo[HAL_SIZEOF_UART_TX_FIFO];
static uint8_t          hal_uart_tx_head;
static volatile uint8_t hal_uart_tx_tail;

void hal_uart_init_hw(void)
{
#if (USART_USE_PORT == 1)       // USART1
#define UARTx_IRQn                  USART1_IRQn
#define hal_uart_irq_handler        USART1_IRQHandler
#define UART_BUS_FREQUENCY          USART1CLK_Frequency
    RCC->APB2ENR   |= RCC_APB2ENR_USART1EN;         // Enable UART1 Clock
    GPIOA->MODER   |= GPIO_MODER_MODER9_1;          // PA9  (TX) - Alternate function mode
    GPIOA->MODER   |= GPIO_MODER_MODER10_1;         // PA10 (RX) - Alternate function mode
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;       // PA9  (TX) - High speed
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10;      // PA10 (RX) - High speed
    GPIOA->AFR[1]  |= 0x0110;                       // PA9, PA10 - AF1

    RCC->CFGR3     &= ~RCC_CFGR3_USART1SW;
    RCC->CFGR3     |=  RCC_CFGR3_USART1SW_0;        //System clock (SYSCLK) selected as USART1 clock
#elif (USART_USE_PORT == 2)     // USART2
#define UARTx_IRQn                  USART2_IRQn
#define hal_uart_irq_handler        USART2_IRQHandler
#define UART_BUS_FREQUENCY          PCLK_Frequency
    RCC->APB1ENR   |= RCC_APB1ENR_USART2EN;         // Enable UART2 Clock
    GPIOA->MODER   |= GPIO_MODER_MODER2_1;          // PA2  (TX) - Alternate function mode
    GPIOA->MODER   |= GPIO_MODER_MODER3_1;          // PA3  (RX) - Alternate function mode
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2;       // PA2  (TX) - High speed
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;       // PA3  (RX) - High speed
    GPIOA->AFR[0]  |= 0x1100;                       // PA2, PA3  - AF1
#else   //  USART_USE_PORT
#error unknown usart port
#endif  //  USART_USE_PORT

    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus);

    UARTx->CR1 &= (uint32_t)~((uint32_t)USART_CR1_UE);              // Disable USART1
    UARTx->CR2 &= ~USART_CR2_STOP;                                  // 1 Stop-bit
    UARTx->CR1 &= ~((uint32_t)(USART_CR1_M |                        // Data - 8 bit
                               USART_CR1_PCE | USART_CR1_PS));      // No Parity
    UARTx->CR3 &= ~((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE));   // Without flow control
    UARTx->BRR  = ((RCC_ClocksStatus.UART_BUS_FREQUENCY + UART_BaudRate/2)/UART_BaudRate);  // Speed
    UARTx->CR1 |= USART_CR1_TE |                    // Enable TX
                  USART_CR1_RE |                    // Enable RX
                  USART_CR1_RXNEIE;                 // Enable RX Not Empty IRQ
    NVIC_EnableIRQ(UARTx_IRQn);                     // Enable UASRT IRQ
    UARTx->CR1 |= USART_CR1_UE;                     // Enable USART
}

bool hal_uart_tx_busy(void)
{
    if(hal_uart_tx_head == hal_uart_tx_tail)
        return false;

    if((UARTx->CR1 & USART_CR1_TXEIE) == 0)
    {
        UARTx->TDR = hal_uart_tx_fifo[hal_uart_tx_tail];
        hal_uart_tx_tail++;
        hal_uart_tx_tail &= (uint8_t)(HAL_SIZEOF_UART_TX_FIFO - 1);
        UARTx->CR1 |= USART_CR1_TXEIE;
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

void hal_uart_irq_handler(void)
{
    uint32_t itstat = UARTx->ISR;
    itstat &= UARTx->CR1;               // only for RXNE, TC, TXE

    // Received data is ready to be read
    if(itstat & USART_ISR_RXNE)
    {
        uint8_t data = (UARTx->RDR & 0xFF);
        uint8_t tmp_head = (hal_uart_rx_head + 1) & (uint8_t)(HAL_SIZEOF_UART_RX_FIFO - 1);
        if(tmp_head == hal_uart_rx_tail)        // Overflow
            return;

        hal_uart_rx_fifo[hal_uart_rx_head] = data;
        hal_uart_rx_head = tmp_head;    
    }

    // Transmit data register empty
    if(itstat & USART_ISR_TXE)
    {
        if(hal_uart_tx_head == hal_uart_tx_tail)
        {
            UARTx->CR1 &= ~(uint32_t)USART_CR1_TXEIE;
            return;
        }

        UARTx->TDR = hal_uart_tx_fifo[hal_uart_tx_tail];
        hal_uart_tx_tail++;
        hal_uart_tx_tail &= (uint8_t)(HAL_SIZEOF_UART_TX_FIFO - 1);
    }
}

#endif  //  UART_PHY