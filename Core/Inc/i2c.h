/*
 * i2c.h
 *
 *  Created on: 2021¦~1¤ë6¤é
 *      Author: user
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "stm32l4xx_hal.h"

extern I2C_HandleTypeDef *mpu6050_i2c;

int i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);
int i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length,  const uint8_t *data);

#endif /* INC_I2C_H_ */
