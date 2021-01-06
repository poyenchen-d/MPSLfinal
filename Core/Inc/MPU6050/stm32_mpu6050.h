#ifndef _STM32_MPU6050_H_
#define _STM32_MPU6050_H_
/* The following functions must be defined for this platform:
 * i2c_write(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char const *data)
 * i2c_read(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char *data)
 * delay_ms(unsigned long num_ms)
 * get_ms(unsigned long *count)
 * reg_int_cb(void (*cb)(void), unsigned char port, unsigned char pin)
 * labs(long x)
 * fabsf(float x)
 * min(int a, int b)
 */

#include "stm32l4xx_hal.h"
#include "i2c.h"
#include "uart.h"

#define delay_ms    	HAL_Delay
#define get_ms(x)     	do { *x = HAL_GetTick(); } while(0)
#define min(a,b) 	    ((a<b)?a:b)

//#define fabs        	fabsf
#define log_e(fmt, ...) USART_printf(DEF_UART, fmt, ##__VA_ARGS__)
#define log_i(fmt, ...) USART_printf(DEF_UART, fmt, ##__VA_ARGS__)

#endif

