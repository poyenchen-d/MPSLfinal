/*
 * keypad.h
 *
 *  Created on: 2021¦~1¤ë7¤é
 *      Author: user
 */

#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_

#include "stm32l4xx_hal.h"

enum KeypadEvent {
	KPEV_None = 0,
	KPEV_Down,
	KPEV_Up,
};

typedef uint16_t KeypadState_t;

void keypad_init();
KeypadState_t keypad_scan_multi();
KeypadState_t keypad_scan_diff(enum KeypadEvent events[16], KeypadState_t orig);

#endif /* INC_KEYPAD_H_ */
