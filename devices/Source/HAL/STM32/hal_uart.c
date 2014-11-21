#include "../../config.h"

#if ((defined UART_PHY) || (defined EXTSER_USED))

#define HAL_SIZEOF_UART_RX_FIFO         32      // Should be 2^n
#define HAL_SIZEOF_UART_TX_FIFO         32      // Should be 2^n

#if (defined STM32F0XX_MD)                      // STM32F0
    #define HAL_USART_RX_DATA           RDR
    #define HAL_USART_TX_DATA           TDR
#elif (defined __STM32F10x_H)                   // STM32F1xx
    #define HAL_USART_RX_DATA           DR
    #define HAL_USART_TX_DATA           DR
#else
    #error unknown uC familie
#endif  //  STM32

typedef struct
{
    uint8_t             rx_fifo[HAL_SIZEOF_UART_RX_FIFO];
    volatile uint8_t    rx_head;
    uint8_t             rx_tail;
    
    uint8_t             tx_fifo[HAL_SIZEOF_UART_TX_FIFO];
    uint8_t             tx_head;
    volatile uint8_t    tx_tail;
}HAL_UART_t;

static HAL_UART_t * hal_UARTv[] = {NULL, NULL};

static const uint16_t hal_baud_list[] = {2400, 4800, 9600, 19200, 38400};

static USART_TypeDef * hal_pUART[] =
            {
            #ifdef USART1
                USART1,
            #else
                NULL,
            #endif  //  USART1
            #ifdef USART2
                USART2
            #else
                NULL
            #endif  //  USART2
            };


void hal_uart_init_hw(uint8_t port, uint8_t nBaud)
{
    assert(nBaud < 5);
    
    IRQn_Type           UARTx_IRQn;
    RCC_ClocksTypeDef   RCC_ClocksStatus;
    
    uint32_t            uart_clock;
    uint16_t            baud = hal_baud_list[nBaud];

    RCC_GetClocksFreq(&RCC_ClocksStatus);
    
    switch(port)
    {
#ifdef USART1
        case 0:
            {
            UARTx_IRQn = USART1_IRQn;
            
#if (defined STM32F0XX_MD)
            RCC->APB2ENR   |= RCC_APB2ENR_USART1EN;         // Enable UART1 Clock
            GPIOA->MODER   |= GPIO_MODER_MODER9_1;          // PA9  (TX) - Alternate function mode
            GPIOA->MODER   |= GPIO_MODER_MODER10_1;         // PA10 (RX) - Alternate function mode
            GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;       // PA9  (TX) - High speed
            GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10;      // PA10 (RX) - High speed
            GPIOA->AFR[1]  |= 0x0110;                       // PA9, PA10 - AF1
//            RCC->CFGR3     &= ~RCC_CFGR3_USART1SW;
//            RCC->CFGR3     |=  RCC_CFGR3_USART1SW_0;        //System clock (SYSCLK) selected as USART1 clock

            uart_clock = RCC_ClocksStatus.USART1CLK_Frequency;
#elif (defined __STM32F10x_H)
            // Enable Clock
            RCC->APB2ENR |= RCC_APB2ENR_USART1EN;           // Enable Clock on USART1
            // Configure GPIO, Tx on PA9, Rx on PA10
            GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
            GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9; // AF Push-Pull out (TX)

            uart_clock = RCC_ClocksStatus.PCLK2_Frequency;
#endif
            }
            break;
#endif  //  USART1
#ifdef USART2
        case 1:
            {
            UARTx_IRQn = USART2_IRQn;
            
#if (defined STM32F0XX_MD)
            RCC->APB1ENR   |= RCC_APB1ENR_USART2EN;         // Enable UART2 Clock
            GPIOA->MODER   |= GPIO_MODER_MODER2_1;          // PA2  (TX) - Alternate function mode
            GPIOA->MODER   |= GPIO_MODER_MODER3_1;          // PA3  (RX) - Alternate function mode
            GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2;       // PA2  (TX) - High speed
            GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;       // PA3  (RX) - High speed
            GPIOA->AFR[0]  |= 0x1100;                       // PA2, PA3  - AF1

            uart_clock = RCC_ClocksStatus.PCLK_Frequency;
#elif (defined __STM32F10x_H)
            // Enable Clock
            RCC->APB1ENR |= RCC_APB1ENR_USART2EN;           // Enable Clock on USART2
            // Configure GPIO, Tx on PA2, Rx on PA3
            GPIOA->CRL &= ~GPIO_CRL_CNF2_0;
            GPIOA->CRL |= GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2; // AF Push-Pull out (TX)

            uart_clock = RCC_ClocksStatus.PCLK1_Frequency;
#endif
            }
            break;
#endif  //  USART2
        default:
            assert(0);
    }

    if(hal_UARTv[port] == NULL)
    {
        hal_UARTv[port] = mqAlloc(sizeof(HAL_UART_t));
        assert(hal_UARTv[port] != NULL);
    }

    hal_UARTv[port]->rx_head = 0;
    hal_UARTv[port]->rx_tail = 0;
    hal_UARTv[port]->tx_head = 0;
    hal_UARTv[port]->tx_tail = 0;

    hal_pUART[port]->CR1 = 0;                               // Disable USART1
    hal_pUART[port]->CR2 = 0;                               // 8N1
    hal_pUART[port]->CR3 = 0;                               // Without flow control
    hal_pUART[port]->BRR  = ((uart_clock + baud/2)/baud);   // Speed
    hal_pUART[port]->CR1 |= USART_CR1_TE |                  // Enable TX
                            USART_CR1_RE |                  // Enable RX
                            USART_CR1_RXNEIE;               // Enable RX Not Empty IRQ
    NVIC_EnableIRQ(UARTx_IRQn);                             // Enable UASRT IRQ
    hal_pUART[port]->CR1 |= USART_CR1_UE;                   // Enable USART
}

bool hal_uart_tx_busy(uint8_t port)
{
    assert(hal_UARTv[port] != NULL);

    if(hal_UARTv[port]->tx_head == hal_UARTv[port]->tx_tail)
        return false;

    if((hal_pUART[port]->CR1 & USART_CR1_TXEIE) == 0)
    {
        hal_pUART[port]->HAL_USART_TX_DATA = hal_UARTv[port]->tx_fifo[hal_UARTv[port]->tx_tail];
        hal_UARTv[port]->tx_tail++;
        hal_UARTv[port]->tx_tail &= (uint8_t)(HAL_SIZEOF_UART_TX_FIFO - 1);
        hal_pUART[port]->CR1 |= USART_CR1_TXEIE;
        return false;
    }

    return (((hal_UARTv[port]->tx_head + 1) & (uint8_t)(HAL_SIZEOF_UART_TX_FIFO - 1)) == hal_UARTv[port]->tx_tail);
}

void hal_uart_send(uint8_t port, uint8_t data)
{
    assert(hal_UARTv[port] != NULL);

    uint8_t tmp_head = (hal_UARTv[port]->tx_head + 1) & (uint8_t)(HAL_SIZEOF_UART_TX_FIFO - 1);
    if(tmp_head == hal_UARTv[port]->tx_tail)        // Overflow
        return;

    hal_UARTv[port]->tx_fifo[hal_UARTv[port]->tx_head] = data;
    hal_UARTv[port]->tx_head = tmp_head;
}

bool hal_uart_datardy(uint8_t port)
{
    assert(hal_UARTv[port] != NULL);
    return (hal_UARTv[port]->rx_head != hal_UARTv[port]->rx_tail);
}

bool hal_uart_get(uint8_t port, uint8_t * pData)
{
    assert(hal_UARTv[port] != NULL);

    if(hal_UARTv[port]->rx_head == hal_UARTv[port]->rx_tail)
        return false;
    
    *pData = hal_UARTv[port]->rx_fifo[hal_UARTv[port]->rx_tail];
    hal_UARTv[port]->rx_tail++;
    hal_UARTv[port]->rx_tail &= (uint8_t)(HAL_SIZEOF_UART_RX_FIFO - 1);

    return true;
}

// IRQ handlers
static inline void hal_uart_irq_handler(uint8_t port)
{
    assert(hal_UARTv[port] != NULL);
    
    uint8_t data;
    uint32_t itstat;
#if (defined STM32F0XX_MD)                      // STM32F0
    itstat = hal_pUART[port]->ISR;
    if(itstat & USART_ISR_ORE)
    {
        hal_pUART[port]->ICR = USART_ICR_ORECF;
        return;
    }
#elif (defined __STM32F10x_H)                   // STM32F1xx    
    itstat = hal_pUART[port]->SR;
    if(itstat & USART_SR_ORE)
    {
        data = hal_pUART[port]->HAL_USART_RX_DATA;
        return;
    }
#endif  //  STM32
    
    // Received data is ready to be read
    if(itstat & USART_CR1_RXNEIE)
    {
        data = hal_pUART[port]->HAL_USART_RX_DATA;
        uint8_t tmp_head = (hal_UARTv[0]->rx_head + 1) & (uint8_t)(HAL_SIZEOF_UART_RX_FIFO - 1);
        if(tmp_head == hal_UARTv[0]->rx_tail)        // Overflow
            return;
            
        hal_UARTv[0]->rx_fifo[hal_UARTv[0]->rx_head] = data;
        hal_UARTv[0]->rx_head = tmp_head;
    }
    
    // Transmit data register empty
    if((itstat & USART_CR1_TXEIE) && (hal_pUART[port]->CR1 & USART_CR1_TXEIE))
    {

        if(hal_UARTv[0]->tx_head == hal_UARTv[0]->tx_tail)
        {
            hal_pUART[port]->CR1 &= ~(uint32_t)USART_CR1_TXEIE;
            return;
        }

        hal_pUART[port]->HAL_USART_TX_DATA = hal_UARTv[0]->tx_fifo[hal_UARTv[0]->tx_tail];
        hal_UARTv[0]->tx_tail++;
        hal_UARTv[0]->tx_tail &= (uint8_t)(HAL_SIZEOF_UART_TX_FIFO - 1);
    }
}

#ifdef USART1
void USART1_IRQHandler(void)
{
    hal_uart_irq_handler(0);
}
#endif  // USART1_IRQHandler

#ifdef USART2
void USART2_IRQHandler(void)
{
    hal_uart_irq_handler(1);
}
#endif  //  USART2_IRQHandler

#endif  //  ((defined UART_PHY) || (defined EXTSER_USED))
