/*
 * keypad.c
 *
 *  Created on: 2021¦~1¤ë7¤é
 *      Author: user
 */
#include "keypad.h"
#include "main.h"


const static GPIO_TypeDef *KEYPAD_ROW_PORT[] = {
	X0_GPIO_Port,
	X1_GPIO_Port,
	X2_GPIO_Port,
	X3_GPIO_Port,
};

const static uint16_t KEYPAD_ROW_PIN[] = {
	X0_Pin,
	X1_Pin,
	X2_Pin,
	X3_Pin,
};

const static GPIO_TypeDef *KEYPAD_COL_PORT[] = {
	Y0_GPIO_Port,
	Y1_GPIO_Port,
	Y2_GPIO_Port,
	Y3_GPIO_Port,
};

const static uint16_t KEYPAD_COL_PIN[] = {
	Y0_Pin,
	Y1_Pin,
	Y2_Pin,
	Y3_Pin,
};


const static char KEYPAD_CODE[4][4] = {
    { 0x1, 0x2, 0x3, 0xA },
    { 0x4, 0x5, 0x6, 0xB },
    { 0x7, 0x8, 0x9, 0xC },
    { 0xF, 0x0, 0xE, 0xD },
};

void keypad_init()
{
	for (int i = 0; i < 4; i++) {
		HAL_GPIO_WritePin(KEYPAD_ROW_PORT[i], KEYPAD_ROW_PIN[i], GPIO_PIN_SET);
	}
}

KeypadState_t keypad_scan_multi()
{
	KeypadState_t ret = 0;
    for (int i = 0; i < 4; i++) {
        // Set Xi output as Hi-Z
    	HAL_GPIO_WritePin(KEYPAD_ROW_PORT[i], KEYPAD_ROW_PIN[i], GPIO_PIN_RESET);

    	int delay = 10;
    	while (delay--);

        for (int j = 0; j < 4; j++) {
            if (HAL_GPIO_ReadPin(KEYPAD_COL_PORT[j], KEYPAD_COL_PIN[j]) == GPIO_PIN_RESET) {
                ret |= (1U << KEYPAD_CODE[i][j]);
            }
        }
        // Set Xi output to HIGH
        HAL_GPIO_WritePin(KEYPAD_ROW_PORT[i], KEYPAD_ROW_PIN[i], GPIO_PIN_SET);
    }
    return ret;
}

KeypadState_t keypad_scan_diff(enum KeypadEvent events[16], KeypadState_t orig)
{
	KeypadState_t new = keypad_scan_multi();
	uint16_t keyup = orig & (~new);
	uint16_t keydown = (~orig) & new;

	for (int i = 0; i < 16; i++) {
		uint16_t mask = (1U << i);
		if (keyup & mask) {
			events[i] = KPEV_Up;
		} else if (keydown & mask) {
			events[i] = KPEV_Down;
		} else {
			events[i] = KPEV_None;
		}
	}

	return new;
}
