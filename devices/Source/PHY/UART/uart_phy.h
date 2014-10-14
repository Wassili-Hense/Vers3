/*
Copyright (c) 2011-2014 <comparator@gmx.de>

This file is part of the X13.Home project.
http://X13home.org
http://X13home.net
http://X13home.github.io/

BSD New License
See LICENSE file for license details.
*/

#ifndef _UART_PHY_H
#define _UART_PHY_H

// HAL Section
//void hal_uart_init_hw(void);
//void hal_uart_send(uint8_t data);

// IRQ handlers
//void uart_rx_handler(BaseType_t * pxHigherPriorityTaskWoken, uint8_t data);
//void uart_tx_handler(BaseType_t * pxHigherPriorityTaskWoken);

// API Section
void UART_Init(void);
void UART_Send(void *pBuf);

#endif  //  _UART_PHY_H
