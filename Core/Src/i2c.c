/*
 * i2c.c
 *
 *  Created on: 2021¦~1¤ë6¤é
 *      Author: user
 */
#include "i2c.h"

I2C_HandleTypeDef *mpu6050_i2c = NULL;

int i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(mpu6050_i2c, (uint16_t)slave_addr << 1, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
	return (status == HAL_OK ? 0 : -1);
}

int i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length,  const uint8_t *data)
{
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(mpu6050_i2c, (uint16_t)slave_addr << 1, (uint16_t)reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
	return (status == HAL_OK ? 0 : -1);
}

