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

#define UART_SIZEOF_RX_FIFO     16
#define UART_SIZEOF_TX_FIFO     16

#if (UART_PHY == 1)
#define UART_ADDR               phy1addr
#define UART_ADDR_t             PHY1_ADDR_t
#elif (UART_PHY == 2)
#define UART_ADDR               phy2addr
#define UART_ADDR_t             PHY2_ADDR_t
#endif  //  UART_PHY

static Queue_t  uart_tx_queue;

static void uart_tx_task(void)
{
    static uint8_t  tx_pos = 0xFF;
    static uint8_t  tx_len = 0;
    static bool     tx_db = false;
    static MQ_t   * pTx_buf = NULL;

    uint8_t data;

uart_tx_task_lbl1:

    if(tx_pos < tx_len)
    {
        data = pTx_buf->raw[tx_pos];
    }
    else if(tx_pos == tx_len)
    {
        if(pTx_buf != NULL)
        {
            MEM_Free(pTx_buf);
            pTx_buf = NULL;
        }

        if(hal_uart_send(0xC0))
        {
            tx_pos = 0xFF;
        }

        return;
    }
    else
    {
        if(pTx_buf == NULL)
        {
            if(!MEM_Dequeue(&uart_tx_queue, &pTx_buf))
                return;
        }

        // Send Length
        tx_pos = 0;
        data = pTx_buf->Length;
        tx_len = data;
    }

    // Convert data from RAW to SLEEP format
    if((data == 0xC0) || (data == 0xDB))
    {
        if(tx_db)
        {
            tx_db = false;
            data ^= 0x20;
        }
        else
        {
            if(!hal_uart_send(0xDB))
                return;
        
            tx_db = true;
            goto uart_tx_task_lbl1;
        }
    }

    if(!hal_uart_send(data))
        return;
    
    tx_pos++;
    goto uart_tx_task_lbl1;
}

void UART_Init(void)
{
    uart_tx_queue.pHead = NULL;
    uart_tx_queue.pTail = NULL;
    
    hal_uart_init_hw();
}

void UART_Send(void *pBuf)
{
    if(!MEM_Enqueue(&uart_tx_queue, pBuf))
        MEM_Free(pBuf);
    else
        uart_tx_task();
}

void * UART_Get(void)
{
    uart_tx_task();
    
    // Rx Task
    static uint8_t  rx_pos = 0;
    static uint8_t  rx_len = 0;
    static MQ_t   * pRx_buf = NULL;
    static bool     rx_db = false;

    uint8_t data;
    MQ_t * pRetVal = NULL;

    while(hal_uart_get(&data))
    {
        // Convert from SLEEP to RAW data
        if(data == 0xC0)
        {
            if((rx_pos > 1) && (rx_len == (rx_pos - 1)))
            {
                UART_ADDR_t s_addr = (UART_ADDR_t)UART_PHY;
                memcpy(pRx_buf->UART_ADDR, &s_addr, sizeof(UART_ADDR_t));

                pRetVal = pRx_buf;
                pRx_buf = NULL;
            }

            rx_len = 0;
            rx_pos = 0;
            rx_db = false;
            break;
        }
        else if(rx_db)
        {
            rx_db = false;
            data ^= 0x20;
        }
        else if(data == 0xDB)
        {
            rx_db = true;
            continue;
        }
        
        if(rx_pos == 0)  // Get Length
        {
            if((data < 2) || (data > sizeof(MQTTSN_MESSAGE_t)))     // Bad Message Length
            {
                rx_len = 0;
                rx_pos = 0xFF;
                continue;
            }
        
            if(pRx_buf == NULL)
            {
                pRx_buf = MEM_Malloc(sizeof(MQ_t));
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
    
    return pRetVal;
}


/*
static void uart_task(void *pvParameters)
{
    MQ_t * pMqBuf;
    UBaseType_t qIsrSize;

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
*/

#endif  //  UART_PHY
