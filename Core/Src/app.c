/*
 * app.c
 *
 *  Created on: 2021¦~1¤ë7¤é
 *      Author: user
 */
#include "app.h"
#include "main.h"
#include "MY_NRF24.h"
#include "uart.h"
#include <string.h>
#include <stdio.h>

const short MOUSE_MOVE_STEP = 8;
static uint32_t last_action_ts = 0;

int send_command(const char *cmd)
{
	if(NRF24_write(cmd, strlen(cmd))) {
		USART_printf(DEF_UART, "Transmitted Successfully\r\n");
	}
}



int process_key(KeypadState_t cur_state, const enum KeypadEvent events[16], const struct YPR *y)
{
	short mouse_x = 0, mouse_y = 0, mouse_wheel = 0;
	char cmdbuf[32 + 1];
	uint32_t cur_action_ts = HAL_Get_Tick();
	// KeyDown/Up action
	for (int i = 0; i < 16; i++) {
		if (events[i] == KPEV_Up) {
			switch (i) {
			case 15:
				// Mouse LBtn
				send_command("mu 1");
				break;
			case 0:
				// Mouse MBtn
				send_command("mu 2");
				break;
			case 14:
				// Mouse RBtn
				send_command("mu 4");
				break;
			case 13:
				// Laser
				HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
				break;
			case 12:
				// Reconfigure
				// TODO
				break;
			}
			//USART_printf(DEF_UART, "Key %d Up!\r\n", i);
		} else if (events[i] == KPEV_Down) {
			switch (i) {
			case 1:
				// Wheel Up
				mouse_wheel -= 1;
				last_action_ts = cur_action_ts;
				break;
			case 7:
				// Wheel Down
				mouse_wheel += 1;
				last_action_ts = cur_action_ts;
				break;
			case 2:
				// Mouse Up
				mouse_y -= MOUSE_MOVE_STEP;
				last_action_ts = cur_action_ts;
				break;
			case 8:
				// Mouse Down
				mouse_y += MOUSE_MOVE_STEP;
				last_action_ts = cur_action_ts;
				break;
			case 4:
				// Mouse Left
				mouse_x -= MOUSE_MOVE_STEP;
				last_action_ts = cur_action_ts;
				break;
			case 6:
				// Mouse Right
				mouse_x += MOUSE_MOVE_STEP;
				last_action_ts = cur_action_ts;
				break;
			case 5:
				// Abs Move
				// TODO
				break;
			case 15:
				// Mouse LBtn
				send_command("md 1");
				break;
			case 0:
				// Mouse MBtn
				send_command("md 2");
				break;
			case 14:
				// Mouse RBtn
				send_command("md 4");
				break;
			case 13:
				// Laser
				HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
				break;
			case 12:
				// Reconfigure
				// TODO
				break;
			}
			//USART_printf(DEF_UART, "Key %d Down!\r\n", i);
		}
	}

	// KeyPressed action
	if ((cur_action_ts - last_action_ts) > 500) {
		for (int i = 0; i < 16; i++) {
			uint16_t mask = (1U << i);
			if (cur_state & mask) {
				switch (i) {
				case 1:
					// Wheel Up
					mouse_wheel -= 1;
					break;
				case 7:
					// Wheel Down
					mouse_wheel += 1;
					break;
				case 2:
					// Mouse Up
					mouse_y -= MOUSE_MOVE_STEP;
					break;
				case 8:
					// Mouse Down
					mouse_y += MOUSE_MOVE_STEP;
					break;
				case 4:
					// Mouse Left
					mouse_x -= MOUSE_MOVE_STEP;
					break;
				case 6:
					// Mouse Right
					mouse_x += MOUSE_MOVE_STEP;
					break;
				case 5:
					// Abs Move
					// TODO
					break;
				}
			}
		}

		last_action_ts = cur_action_ts;
	}

	if (mouse_x != 0 || mouse_y != 0) {
		snprintf(cmdbuf, sizeof(cmdbuf), "mr %d %d", mouse_x, mouse_y);
		send_command(cmdbuf);
	}

	if (mouse_wheel != 0) {
		snprintf(cmdbuf, sizeof(cmdbuf), "wh %d", mouse_wheel);
		send_command(cmdbuf);
	}

	return 0;
}

