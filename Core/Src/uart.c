/*
 * uart.c
 *
 *  Created on: 2021¦~1¤ë5¤é
 *      Author: user
 */

#include "uart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef *DEF_UART = NULL;

void USART_printf(UART_HandleTypeDef *usart, const char *fmt, ...)
{
	va_list ap;
	char buf[1024];

	va_start(ap, fmt);

	vsnprintf(buf, sizeof(buf), fmt, ap);

	HAL_UART_Transmit(usart, (uint8_t *)buf, (uint16_t)strlen(buf), 10);

	va_end(ap);
}
