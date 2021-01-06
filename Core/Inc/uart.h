/*
 * uart.h
 *
 *  Created on: 2021¦~1¤ë5¤é
 *      Author: user
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32l4xx_hal.h"

extern UART_HandleTypeDef *DEF_UART;

void USART_printf(UART_HandleTypeDef *usart, const char *fmt, ...);

#endif /* INC_UART_H_ */
