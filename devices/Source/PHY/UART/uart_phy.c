/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

// UART interface

#include "../../config.h"

#ifdef UART_PHY
/*
#if (UART_PHY == 1)
#define UART_ADDR           phy1addr
#define UART_ADDR_t         PHY1_ADDR_t
#elif (UART_PHY == 2)
#define UART_ADDR           phy2addr
#define UART_ADDR_t         PHY2_ADDR_t
#endif  //  UART_PHY

static QueueHandle_t        uart_in_queue = NULL;
static QueueHandle_t        uart_out_queue;
static QueueHandle_t        uart_isr_queue;

static void uart_task(void *pvParameters)
{
    MQ_t * pMqBuf;
    UBaseType_t qIsrSize;

    hal_uart_init_hw();

    while(1)
    {
        qIsrSize = uxQueueMessagesWaiting(uart_isr_queue);
        if(qIsrSize < 2)
        {
            pMqBuf = pvPortMalloc(sizeof(MQ_t));
            if(pMqBuf != NULL)
                xQueueSend(uart_isr_queue, &pMqBuf, 0);
        }
        else if(qIsrSize > 4)
        {
            if(xQueueReceive(uart_isr_queue, &pMqBuf, 0) == pdTRUE)
                vPortFree(pMqBuf);
        }
    
        if(xQueueReceive(uart_out_queue, &pMqBuf, 0) == pdTRUE)
        {
#if (UART_PHY == 1)
            mqttsn_parser_phy1(pMqBuf);
#else   //  UART_PHY == 2
            mqttsn_parser_phy2(pMqBuf);
#endif  //  UART_PHY
        }
        taskYIELD();
    }
    vTaskDelete(NULL);
}

void UART_Init(void)
{
    if(uart_in_queue == NULL)
    {
        uart_in_queue  = xQueueCreate(8, sizeof(void *));
        uart_out_queue = xQueueCreate(4, sizeof(void *));
        uart_isr_queue = xQueueCreate(8, sizeof(void *));

        xTaskCreate(uart_task, "uart", configMINIMAL_STACK_SIZE + 16, NULL, tskIDLE_PRIORITY, NULL );
    }
}

void UART_Send(void *pBuf)
{
    if(xQueueSend(uart_in_queue, &pBuf, 0) != pdTRUE)
        vPortFree(pBuf);
    else if(IS_UART_TX_INT_ENABLED() == 0)
    {
        hal_uart_send(0xC0);                                // Send End Of Frame - Flush buffers
        UART_TX_ENABLE_INT();                               // Enable the UARTx Transmit interrupt
    }
}

void uart_rx_handler(BaseType_t * pxHigherPriorityTaskWoken, uint8_t data)
{
    static uint8_t  rx_pos = 0;
    static uint8_t  rx_len = 0;
    static MQ_t   * pRx_buf = NULL;
    static bool     rx_db = false;

    // Convert from SLEEP to RAW data
    if(data == 0xC0)
    {
        if(rx_pos > 1)
        {
            if(rx_len == (rx_pos - 1))
            {
                UART_ADDR_t s_addr = (UART_ADDR_t)UART_PHY;
                memcpy(pRx_buf->UART_ADDR, &s_addr, sizeof(UART_ADDR_t));
                if(xQueueSendFromISR(uart_out_queue, &pRx_buf, pxHigherPriorityTaskWoken) == pdTRUE)
                    pRx_buf = NULL;
            }
        }

        rx_len = 0;
        rx_pos = 0;
        rx_db = false;
        return;
    }
    else if(rx_db)
    {
        rx_db = false;
        data ^= 0x20;
    }
    else if(data == 0xDB)
    {
        rx_db = true;
        return;
    }

    if(rx_pos == 0)  // Get Length
    {
        if((data < 2) || (data > sizeof(MQTTSN_MESSAGE_t)))     // Bad Message Length
        {
            rx_len = 0;
            rx_pos = 0xFF;
            return;
        }
        
        if(pRx_buf == NULL)
        {
            if(xQueueReceiveFromISR(uart_isr_queue, &pRx_buf, pxHigherPriorityTaskWoken) != pdTRUE)
                pRx_buf = NULL;
        }

        if(pRx_buf != NULL)
        {
            rx_len = data;
            pRx_buf->Length = data;
            rx_pos = 1;
        }
        else
        {
            rx_len = 0;
            rx_pos = 0xFF;
        }
    }
    else if(rx_pos <= rx_len)
    {
        pRx_buf->raw[rx_pos - 1] = data;
        rx_pos++;
    }
    else  // overflow
    {
        rx_len = 0;
        rx_pos = 0xFF;
    }
}

void uart_tx_handler(BaseType_t * pxHigherPriorityTaskWoken)
{
    static uint8_t  tx_pos = 0xFF;
    static uint8_t  tx_len = 0;
    static bool     tx_db = false;
    static MQ_t   * pTx_buf;

    uint8_t data;

start_tx_handler:

    if(tx_pos == 0)             // Send Length
    {
        data = pTx_buf->Length;
        tx_len = data;
    }
    else if(tx_pos <= tx_len)
        data = pTx_buf->raw[tx_pos - 1];
    else if(tx_pos == (tx_len + 1))
    {
        hal_uart_send(0xC0);      // Send End Of Frame
        tx_pos = 0xFF;
        xQueueSendFromISR(uart_isr_queue, &pTx_buf, pxHigherPriorityTaskWoken);
        return;
    }
    else
    {
        if(xQueueReceiveFromISR(uart_in_queue, &pTx_buf, pxHigherPriorityTaskWoken) == pdFALSE)  // Queue is empty
        {
            // Disable the UARTx Transmit interrupt
            UART_TX_DISABLE_INT();
            return;
        }
        tx_pos = 0;
        goto start_tx_handler;
    }
  
    // Convert data from RAW to SLEEP format
    if((data == 0xC0) || (data == 0xDB))
    {
        if(tx_db)
        {
            tx_db = false;
            data ^= 0x20;
            tx_pos++;
        }
        else
        {
            tx_db = true;
            data = 0xDB;
        }
    }
    else
        tx_pos++;

    hal_uart_send(data);
}
*/
#endif  //  UART_PHY
