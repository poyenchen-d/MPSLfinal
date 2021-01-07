/*
 * app.h
 *
 *  Created on: 2021¦~1¤ë7¤é
 *      Author: user
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include "keypad.h"
#include "stm32l4xx_hal.h"
#include "MPU6050/mpu6050.h"

int process_key(KeypadState_t cur_state, const enum KeypadEvent events[16], const struct YPR *y);

#endif /* INC_APP_H_ */
