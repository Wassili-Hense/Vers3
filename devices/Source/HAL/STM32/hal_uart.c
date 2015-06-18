#include "../../config.h"

#if ((defined HAL_USE_USART1) || \
     (defined HAL_USE_USART2) || \
     (defined HAL_USE_USART3) || \
     (defined HAL_USE_UART4) || \
     (defined HAL_USE_UART5) || \
     (defined HAL_USE_USART6))

#define HAL_SIZEOF_UART_RX_FIFO         32      // Should be 2^n

#if (defined __STM32F0XX_H)
    #define HAL_USART_RX_DATA           RDR
    #define HAL_USART_TX_DATA           TDR
#else
    #define HAL_USART_RX_DATA           DR
    #define HAL_USART_TX_DATA           DR
#endif  //  STM32

/*
            Config 1        Config 2
U[S]ARTx    RX      TX      RX      TX      APB
USART1      PA10    PA9     PB7     PB6     2
USART2      PA3     PA2                     1
USART3      PB11    PB10    PC11    PC10    1
UART4       PA1     PA0     PC11    PC10    1
UART5       PD2     PC12                    1
USART6      PC7     PC6                     2

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

typedef struct
{
    uint8_t             rx_fifo[HAL_SIZEOF_UART_RX_FIFO];
    volatile uint8_t    rx_head;
    uint8_t             rx_tail;

    uint8_t         *   pTxBuf;
    uint8_t             tx_len;
    uint8_t             tx_pos;
}HAL_UART_t;

static HAL_UART_t * hal_UARTv[6] = {NULL, };

static const USART_TypeDef * hal_pUART[6] =
            {
            #if (defined USART1) && (defined HAL_USE_USART1)
                USART1,
            #else
                NULL,
            #endif  //  USART1
            #if (defined USART2) && (defined HAL_USE_USART2)
                USART2,
            #else
                NULL,
            #endif  //  USART2
            #if (defined USART3) && (defined HAL_USE_USART3)
                USART3,
            #else
                NULL,
            #endif  //  USART3
            #if (defined UART4) && (defined HAL_USE_UART4)
                UART4,
            #else
                NULL,
            #endif  //  UART4
            #if (defined UART5) && (defined HAL_USE_UART5)
                UART5,
            #else
                NULL,
            #endif  //  UART5
            #if (defined USART6) && (defined HAL_USE_USART6)
                USART6,
            #else
                NULL,
            #endif  //  USART6
            };

static const uint16_t hal_baud_list[] = {2400, 4800, 9600, 19200, 38400};

// IRQ handlers
static inline void hal_uart_irq_handler(uint8_t port)
{
    assert(hal_UARTv[port] != NULL);
    
    USART_TypeDef * USARTx = (USART_TypeDef *)hal_pUART[port];
    HAL_UART_t * control = hal_UARTv[port];

    uint8_t data;
    uint32_t itstat;
#if (defined __STM32F0XX_H)                      // STM32F0
    itstat = USARTx->ISR;
    if(itstat & USART_ISR_ORE)
    {
        USARTx->ICR = USART_ICR_ORECF;
        return;
    }
#else                                           // STM32F1xx / STM32F4xx
    itstat = USARTx->SR;
    if(itstat & USART_SR_ORE)
    {
        data = USARTx->HAL_USART_RX_DATA;
        return;
    }
#endif  //  STM32

    // Received data is ready to be read
    if(itstat & USART_CR1_RXNEIE)
    {
        data = USARTx->HAL_USART_RX_DATA;
        uint8_t tmp_head = (control->rx_head + 1) & (uint8_t)(HAL_SIZEOF_UART_RX_FIFO - 1);
        if(tmp_head == control->rx_tail)        // Overflow
            return;
            
        control->rx_fifo[control->rx_head] = data;
        control->rx_head = tmp_head;
    }

    // Transmit data register empty
    if((itstat & USART_CR1_TXEIE) && (USARTx->CR1 & USART_CR1_TXEIE))
    {
        if(control->tx_pos == control->tx_len)
        {
            control->tx_len = 0;
            USARTx->CR1 &= ~(uint32_t)USART_CR1_TXEIE;
            return;
        }

        USARTx->HAL_USART_TX_DATA = control->pTxBuf[control->tx_pos];
        control->tx_pos++;
    }
}

#if (defined USART1) && (defined HAL_USE_USART1)
void USART1_IRQHandler(void)
{
    hal_uart_irq_handler(0);
}
#endif  // USART1

#if (defined USART2) && (defined HAL_USE_USART2)
void USART2_IRQHandler(void)
{
    hal_uart_irq_handler(1);
}
#endif  //  USART2

#if (defined USART3) && (defined HAL_USE_USART3)
void USART3_IRQHandler(void)
{
    hal_uart_irq_handler(2);
}
#endif  //  USART3

#if (defined UART4) && (defined HAL_USE_UART4)
void UART4_IRQHandler(void)
{
    hal_uart_irq_handler(3);
}
#endif  //  UART4

#if (defined UART5) && (defined HAL_USE_UART5)
void UART5_IRQHandler(void)
{
    hal_uart_irq_handler(4);
}
#endif  //  UART4

#if (defined USART6) && (defined HAL_USE_USART6)
void USART6_IRQHandler(void)
{
    hal_uart_irq_handler(5);
}
#endif  //  USART3

void hal_uart_deinit(uint8_t port)
{
    switch(port)
    {
#if (defined USART1) && (defined HAL_USE_USART1)
        case 1:
            {
            USART1->CR1 &= ~USART_CR1_UE;               // Disable USART
            NVIC_DisableIRQ(USART1_IRQn);               // Disable USART IRQ

            RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;    // Reset USART
            RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;

            RCC->APB2ENR  &= ~RCC_APB2ENR_USART1EN;     // Disable UART1 Clock

            hal_dio_gpio_cfg(GPIOA, GPIO_Pin_9 | GPIO_Pin_10, DIO_MODE_IN_FLOAT);
            }
            break;
#endif  //  USART1

#if (defined USART2) && (defined HAL_USE_USART2)
        case 2:
            {
            USART2->CR1 &= ~USART_CR1_UE;               // Disable USART
            NVIC_DisableIRQ(USART2_IRQn);               // Disable USART IRQ

            RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;    // Reset USART
            RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;

            RCC->APB1ENR   &= ~RCC_APB1ENR_USART2EN;    // Disable UART2 Clock
            
            hal_dio_gpio_cfg(GPIOA, GPIO_Pin_2 | GPIO_Pin_3, DIO_MODE_IN_FLOAT);
            }
            break;
#endif  //  USART2

#if (defined USART3) && (defined HAL_USE_USART3)
        case 3:
            {
            USART3->CR1 &= ~USART_CR1_UE;               // Disable USART
            NVIC_DisableIRQ(USART3_IRQn);               // Disable USART IRQ

            RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;    // Reset USART
            RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;

            RCC->APB1ENR   &= ~RCC_APB1ENR_USART3EN;    // Disable UART2 Clock
            
            hal_dio_gpio_cfg(GPIOB, GPIO_Pin_10 | GPIO_Pin_11, DIO_MODE_IN_FLOAT);
            }
            break;
#endif  //  USART3

#if (defined UART4) && (defined HAL_USE_UART4)
        case 4:
            {
            UART4->CR1 &= ~USART_CR1_UE;                // Disable USART
            NVIC_DisableIRQ(UART4_IRQn);                // Disable USART IRQ

            RCC->APB1RSTR |= RCC_APB1RSTR_UART4RST;     // Reset USART
            RCC->APB1RSTR &= ~RCC_APB1RSTR_UART4RST;

            RCC->APB1ENR   &= ~RCC_APB1ENR_UART4EN;     // Disable UART2 Clock
            
            hal_dio_gpio_cfg(GPIOA, GPIO_Pin_0 | GPIO_Pin_1, DIO_MODE_IN_FLOAT);
            }
            break;
#endif  //  UART4

#if ((defined HAL_USE_UART5) || \
     (defined HAL_USE_USART6))
#error hal_uart_deinit, ports not implemented
#endif
        default:
            assert(0);
    }
    
    port--;

    if(hal_UARTv[port] != NULL)
    {
        mqFree(hal_UARTv[port]);
        hal_UARTv[port] = NULL;
    }
}

void hal_uart_init_hw(uint8_t port, uint8_t nBaud)
{
    assert(nBaud < 5);
    
    IRQn_Type           UARTx_IRQn;
    RCC_ClocksTypeDef   RCC_ClocksStatus;
    
    uint32_t            uart_clock;
    uint16_t            baud = hal_baud_list[nBaud];

    RCC_GetClocksFreq(&RCC_ClocksStatus);

    if((port == 1) || (port == 6))  // USART1, USART6
    {
#if (defined __STM32F0XX_H)
        uart_clock = RCC_ClocksStatus.USART1CLK_Frequency;
#else
        uart_clock = RCC_ClocksStatus.PCLK2_Frequency;
#endif
    }
    else                            // USART2/3 UART4/5
    {
#if (defined __STM32F0XX_H)
        uart_clock = RCC_ClocksStatus.PCLK_Frequency;
#else
        uart_clock = RCC_ClocksStatus.PCLK1_Frequency;
#endif
    }

    switch(port)
    {

#if (defined USART1) && (defined HAL_USE_USART1)
        case 1:
            {
            UARTx_IRQn = USART1_IRQn;
            RCC->APB2ENR   |= RCC_APB2ENR_USART1EN;                             // Enable UART1 Clock

#if (defined __STM32F0XX_H)
            hal_dio_gpio_cfg(GPIOA, GPIO_Pin_9 | GPIO_Pin_10, (1<<DIO_AF_OFFS) | DIO_MODE_AF_PP);   // PA9, PA10 - AF1
#elif (defined __STM32F10x_H)
            hal_dio_gpio_cfg(GPIOA, GPIO_Pin_9, DIO_MODE_UART);                                     // PA9 - AF, Tx
#elif (defined STM32F4XX)
            hal_dio_gpio_cfg(GPIOA, GPIO_Pin_9 | GPIO_Pin_10, (7<<DIO_AF_OFFS) | DIO_MODE_AF_PP);   // PA9, PA10 - AF7
#endif  // uC
            }
            break;
#endif  //  USART1

#if (defined USART2) && (defined HAL_USE_USART2)
        case 2:
            {
            UARTx_IRQn = USART2_IRQn;
            RCC->APB1ENR   |= RCC_APB1ENR_USART2EN;                             // Enable UART2 Clock

#if (defined __STM32F0XX_H)
            hal_dio_gpio_cfg(GPIOA, GPIO_Pin_2 | GPIO_Pin_3, (1<<DIO_AF_OFFS) | DIO_MODE_AF_PP);    // PA2, PA3 - AF1
#elif (defined __STM32F10x_H)
            hal_dio_gpio_cfg(GPIOA, GPIO_Pin_2, DIO_MODE_UART);                                     // PA2 - AF, Tx
#elif (defined STM32F4XX)
            hal_dio_gpio_cfg(GPIOA, GPIO_Pin_2 | GPIO_Pin_3, (7<<DIO_AF_OFFS) | DIO_MODE_AF_PP);    // PA2, PA3 - AF7
#endif
            }
            break;
#endif  //  USART2

#if (defined USART3) && (defined HAL_USE_USART3)
        case 3:
            {
            UARTx_IRQn = USART3_IRQn;
            RCC->APB1ENR   |= RCC_APB1ENR_USART3EN;                             // Enable UART3 Clock

#if (defined __STM32F10x_H)
            hal_dio_gpio_cfg(GPIOB, GPIO_Pin_10, DIO_MODE_UART);                                    // PB10 - AF, Tx
#elif (defined STM32F4XX)
            hal_dio_gpio_cfg(GPIOB, GPIO_Pin_10 | GPIO_Pin_11, (7<<DIO_AF_OFFS) | DIO_MODE_AF_PP);  // PB10, PB11 - AF7
#endif
            }
            break;
#endif  //  USART3


#if (defined UART4) && (defined HAL_USE_UART4)
        case 4:
            {
            UARTx_IRQn = UART4_IRQn;
            RCC->APB1ENR   |= RCC_APB1ENR_UART4EN;                              // Enable UART4 Clock

            hal_dio_gpio_cfg(GPIOA, GPIO_Pin_0 | GPIO_Pin_1, (8<<DIO_AF_OFFS) | DIO_MODE_AF_PP);    // PA0, PA1 - AF8
            }
            break;
#endif  //  UART4

#if ((defined HAL_USE_UART5) || \
     (defined HAL_USE_USART6))
#error hal_uart_init_hw, ports not implemented
#endif

        default:
            assert(0);
    }
    
    port--;

    if(hal_UARTv[port] == NULL)
    {
        hal_UARTv[port] = mqAlloc(sizeof(HAL_UART_t));
        assert(hal_UARTv[port] != NULL);
    }

    hal_UARTv[port]->rx_head = 0;
    hal_UARTv[port]->rx_tail = 0;
    
    hal_UARTv[port]->pTxBuf = NULL;
    hal_UARTv[port]->tx_len = 0;
    hal_UARTv[port]->tx_pos = 0;
    
    USART_TypeDef * USARTx = (USART_TypeDef *)hal_pUART[port];

    USARTx->CR1 = 0;                                // Disable USART1
    USARTx->CR2 = 0;                                // 8N1
    USARTx->CR3 = 0;                                // Without flow control
    USARTx->BRR  = ((uart_clock + baud/2)/baud);    // Speed
    USARTx->CR1 |= USART_CR1_TE |                   // Enable TX
                   USART_CR1_RE |                   // Enable RX
                   USART_CR1_RXNEIE;                // Enable RX Not Empty IRQ
    NVIC_EnableIRQ(UARTx_IRQn);                     // Enable UASRT IRQ
    USARTx->CR1 |= USART_CR1_UE;                    // Enable USART
}

bool hal_uart_datardy(uint8_t port)
{
    port--;
    
    assert(hal_UARTv[port] != NULL);
    return (hal_UARTv[port]->rx_head != hal_UARTv[port]->rx_tail);
}

uint8_t hal_uart_get(uint8_t port)
{
    port--;
    
    assert(hal_UARTv[port] != NULL);

    if(hal_UARTv[port]->rx_head == hal_UARTv[port]->rx_tail)
        return 0;
    
    uint8_t data = hal_UARTv[port]->rx_fifo[hal_UARTv[port]->rx_tail];
    hal_UARTv[port]->rx_tail++;
    hal_UARTv[port]->rx_tail &= (uint8_t)(HAL_SIZEOF_UART_RX_FIFO - 1);

    return data;
}

// Tx free
bool hal_uart_free(uint8_t port)
{
    return (hal_UARTv[port - 1]->tx_len == 0);
}

void hal_uart_send(uint8_t port, uint8_t len, uint8_t * pBuf)
{
    port--;
    
    hal_UARTv[port]->tx_len = len;
    hal_UARTv[port]->tx_pos = 1;
    hal_UARTv[port]->pTxBuf = pBuf;
    
    USART_TypeDef * USARTx = (USART_TypeDef *)hal_pUART[port];

    USARTx->HAL_USART_TX_DATA = *pBuf;
    USARTx->CR1 |= USART_CR1_TXEIE;
}

#endif  //  (defined HAL_USE_U[S]ARTx)
