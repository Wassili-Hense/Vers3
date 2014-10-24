#include "../../config.h"

#ifdef UART_PHY

#define HAL_SIZEOF_UART_RX_FIFO         16      // Should be 2^n
#define HAL_SIZEOF_UART_TX_FIFO         16      // Should be 2^n

static uint8_t          hal_uart_rx_fifo[HAL_SIZEOF_UART_RX_FIFO];
static volatile uint8_t hal_uart_rx_head;
static uint8_t          hal_uart_rx_tail;

static uint8_t          hal_uart_tx_fifo[HAL_SIZEOF_UART_TX_FIFO];
static uint8_t          hal_uart_tx_head;
static volatile uint8_t hal_uart_tx_tail;

#if   ((USART_USE_PORT == 1) && (defined USART1))   // USART1
    #define UARTx                       USART1
    #define UARTx_IRQn                  USART1_IRQn
    #define hal_uart_irq_handler        USART1_IRQHandler
#elif ((USART_USE_PORT == 2) && (defined USART2))   // USART2
    #define UARTx                       USART2
    #define UARTx_IRQn                  USART2_IRQn
    #define hal_uart_irq_handler        USART2_IRQHandler
#elif ((USART_USE_PORT == 3) && (defined USART3))   // USART3
    #define UARTx                       USART3
    #define UARTx_IRQn                  USART3_IRQn
    #define hal_uart_irq_handler        USART3_IRQHandler
#else
#error unknown usart configuration
#endif

void hal_uart_init_hw(void)
{
#if (defined STM32F0XX_MD)                          // STM32F0
    #define HAL_USART_GET_ITSTAT()      (UARTx->ISR & UARTx->CR1)
    #define HAL_USART_RX_DATA           (UARTx->RDR & 0xFF)
    #define HAL_USART_TX_DATA           UARTx->TDR

#if (USART_USE_PORT == 1)                           // USART1, STM32F0
    #define UART_BUS_FREQUENCY          USART1CLK_Frequency
    RCC->APB2ENR   |= RCC_APB2ENR_USART1EN;         // Enable UART1 Clock
    GPIOA->MODER   |= GPIO_MODER_MODER9_1;          // PA9  (TX) - Alternate function mode
    GPIOA->MODER   |= GPIO_MODER_MODER10_1;         // PA10 (RX) - Alternate function mode
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;       // PA9  (TX) - High speed
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10;      // PA10 (RX) - High speed
    GPIOA->AFR[1]  |= 0x0110;                       // PA9, PA10 - AF1
    RCC->CFGR3     &= ~RCC_CFGR3_USART1SW;
    RCC->CFGR3     |=  RCC_CFGR3_USART1SW_0;        //System clock (SYSCLK) selected as USART1 clock
#elif (USART_USE_PORT == 2)                         // USART2, STM32F0
    #define UART_BUS_FREQUENCY          PCLK_Frequency
    RCC->APB1ENR   |= RCC_APB1ENR_USART2EN;         // Enable UART2 Clock
    GPIOA->MODER   |= GPIO_MODER_MODER2_1;          // PA2  (TX) - Alternate function mode
    GPIOA->MODER   |= GPIO_MODER_MODER3_1;          // PA3  (RX) - Alternate function mode
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2;       // PA2  (TX) - High speed
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;       // PA3  (RX) - High speed
    GPIOA->AFR[0]  |= 0x1100;                       // PA2, PA3  - AF1
#endif  //  USART_USE_PORT

#elif ((defined STM32F10X_MD) || (defined STM32F10X_MD_VL))     // STM32F1
    #define HAL_USART_GET_ITSTAT()      (UARTx->SR & UARTx->CR1)
    #define HAL_USART_RX_DATA           (UARTx->DR & 0xFF)
    #define HAL_USART_TX_DATA           UARTx->DR

#if (USART_USE_PORT == 1)                           // USART1, STM32F1
    #define UART_BUS_FREQUENCY          PCLK2_Frequency
    // Enable Clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;           // Enable Clock on USART1
    // Configure GPIO, Tx on PA9, Rx on PA10
    GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
    GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9; // AF Push-Pull out (TX)
#elif (USART_USE_PORT == 2)                         // USART2, STM32F1
    #define UART_BUS_FREQUENCY          PCLK1_Frequency
    // Enable Clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;           // Enable Clock on USART2
    // Configure GPIO, Tx on PA2, Rx on PA3
    GPIOA->CRL &= ~GPIO_CRL_CNF2_0;
    GPIOA->CRL |= GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2; // AF Push-Pull out (TX)
#elif (USART_USE_PORT == 3)                         // USART3, STM32F1
    #define UART_BUS_FREQUENCY          PCLK1_Frequency
    // Enable Clock
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;           // Enable Clock on USART3
    // Configure GPIO, Tx on PB10, Rx on PB11
    GPIOB->CRH &= ~GPIO_CRH_CNF10_0;
    GPIOA->CRH |= GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10;   // AF Push-Pull out (TX)
#endif  //  USART_USE_PORT

#endif  //  STM32

    RCC_ClocksTypeDef RCC_ClocksStatus;
    RCC_GetClocksFreq(&RCC_ClocksStatus);

    UARTx->CR1 = 0;                                 // Disable USART1
    UARTx->CR2 = 0;                                 // 8N1
    UARTx->CR3 = 0;                                 // Without flow control
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
        HAL_USART_TX_DATA = hal_uart_tx_fifo[hal_uart_tx_tail];
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
    uint32_t itstat = HAL_USART_GET_ITSTAT();   // only for RXNE, TC, TXE

    // Received data is ready to be read
    if(itstat & USART_CR1_RXNEIE)
    {
        uint8_t data = HAL_USART_RX_DATA;
        uint8_t tmp_head = (hal_uart_rx_head + 1) & (uint8_t)(HAL_SIZEOF_UART_RX_FIFO - 1);
        if(tmp_head == hal_uart_rx_tail)        // Overflow
            return;

        hal_uart_rx_fifo[hal_uart_rx_head] = data;
        hal_uart_rx_head = tmp_head;    
    }

    // Transmit data register empty
    if(itstat & USART_CR1_TXEIE)
    {
        if(hal_uart_tx_head == hal_uart_tx_tail)
        {
            UARTx->CR1 &= ~(uint32_t)USART_CR1_TXEIE;
            return;
        }

        HAL_USART_TX_DATA = hal_uart_tx_fifo[hal_uart_tx_tail];
        hal_uart_tx_tail++;
        hal_uart_tx_tail &= (uint8_t)(HAL_SIZEOF_UART_TX_FIFO - 1);
    }
}

#endif  //  UART_PHY