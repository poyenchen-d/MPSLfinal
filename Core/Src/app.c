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
#include <math.h>
#include <limits.h>
#include <string.h>
#include <stdio.h>

const short MOUSE_MOVE_STEP = 8;
static uint32_t last_action_ts = 0;

enum cursor_cfg_state {
	CFG_NONE = 0,
	CFG_0,
	CFG_1,
	CFG_2,
	CFG_3,
	CFG_DONE,
};
enum cursor_cfg_state cfg_state = CFG_NONE;
float normal_vec[2];	// [0]: yaw; [1]: roll
float tan_x0, tan_x1, tan_y0, tan_y1;

int send_command(const char *cmd)
{
	if(NRF24_write(cmd, strlen(cmd))) {
		USART_printf(DEF_UART, "Transmitted Successfully\r\n");
	}
}

void configure_angle(const struct YPR *y)
{
	switch (cfg_state) {
	case CFG_0:
		normal_vec[0] = y->yaw;
		normal_vec[1] = y->roll;
		USART_printf(DEF_UART, "Normal vector: %f, %f\r\n", normal_vec[0], normal_vec[1]);
		cfg_state = CFG_1;
		break;
	case CFG_1:
		tan_x0 = tan(y->roll - normal_vec[1]);
		tan_y0 = tan(y->yaw - normal_vec[0]);
		cfg_state = CFG_2;
		USART_printf(DEF_UART, "tan_x0: %f\r\n", tan_x0);
		USART_printf(DEF_UART, "tan_y0: %f\r\n", tan_y0);
		break;
	case CFG_2:
		tan_x1 = tan(y->roll - normal_vec[1]);
		USART_printf(DEF_UART, "tan_x1: %f\r\n", tan_x1);
		cfg_state = CFG_3;
		break;
	case CFG_3:
		tan_y1 = tan(y->yaw - normal_vec[0]);
		USART_printf(DEF_UART, "tan_y1: %f\r\n", tan_y1);
		cfg_state = CFG_DONE;
		break;
	}
}

void calc_cursor_pos(uint16_t pos[2], const struct YPR *y)
{
	if (cfg_state != CFG_DONE) {
		return;
	}

	float tan_xx = tan(y->roll - normal_vec[1]);
	float tan_yx = tan(y->yaw - normal_vec[0]);
	float x_p = (tan_xx - tan_x0) / (tan_x1 - tan_x0);
	float y_p = (tan_yx - tan_y0) / (tan_y1 - tan_y0);

	pos[0] = (uint16_t)(x_p * UINT16_MAX);
	pos[1] = (uint16_t)(y_p * UINT16_MAX);
	USART_printf(DEF_UART, "Cursor: (%u, %u)\r\n", pos[0], pos[1]);
}

int process_key(KeypadState_t cur_state, const enum KeypadEvent events[16], const struct YPR *y)
{
	short mouse_x = 0, mouse_y = 0, mouse_wheel = 0;
	char cmdbuf[32 + 1] = {0};
	uint16_t cur_pos[2];
	uint32_t cur_action_ts = HAL_GetTick();
	// KeyDown/Up action
	for (int i = 0; i < 16; i++) {
		if (events[i] == KPEV_Up) {
			switch (i) {
			case 5:
				// Configure Abs Move
				switch (cfg_state) {
				case CFG_0:
				case CFG_1:
				case CFG_2:
				case CFG_3:
					configure_angle(y);
					break;
				default:
					break;
				}
				break;
			case 15:
				// Mouse LBtn
				send_command("mu 1");
				break;
			case 0:
				// Mouse MBtn
				send_command("mu 4");
				break;
			case 14:
				// Mouse RBtn
				send_command("mu 2");
				break;
			case 13:
				// Laser
				if (cfg_state == CFG_NONE || cfg_state == CFG_DONE) {
					HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
				}
				break;
			case 12:
				// Reconfigure
				cfg_state = CFG_0;
				HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
				break;
			}
			//USART_printf(DEF_UART, "Key %d Up!\r\n", i);
		} else if (events[i] == KPEV_Down) {
			switch (i) {
			case 1:
				// Wheel Up
				mouse_wheel += 1;
				last_action_ts = cur_action_ts;
				break;
			case 7:
				// Wheel Down
				mouse_wheel -= 1;
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
				switch (cfg_state) {
				case CFG_DONE:
					calc_cursor_pos(cur_pos, y);
					snprintf(cmdbuf, 32, "ma %u %u", cur_pos[0], cur_pos[1]);
					send_command(cmdbuf);
					break;
				default:
					break;
				}
				break;
			case 15:
				// Mouse LBtn
				send_command("md 1");
				break;
			case 0:
				// Mouse MBtn
				send_command("md 4");
				break;
			case 14:
				// Mouse RBtn
				send_command("md 2");
				break;
			case 13:
				// Laser
				HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
				break;
			}
			//USART_printf(DEF_UART, "Key %d Down!\r\n", i);
		}
	}

	// KeyPressed action
	if ((cur_action_ts - last_action_ts) > 200) {
		for (int i = 0; i < 16; i++) {
			uint16_t mask = (1U << i);
			if (cur_state & mask) {
				switch (i) {
				case 1:
					// Wheel Up
					mouse_wheel += 1;
					break;
				case 7:
					// Wheel Down
					mouse_wheel -= 1;
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
					switch (cfg_state) {
					case CFG_DONE:
						calc_cursor_pos(cur_pos, y);
						snprintf(cmdbuf, 32, "ma %u %u", cur_pos[0], cur_pos[1]);
						send_command(cmdbuf);
						break;
					default:
						break;
					}
					break;
				}
			}
		}

		last_action_ts = cur_action_ts;
	}

	if (mouse_x != 0 || mouse_y != 0) {
		snprintf(cmdbuf, 32, "mr %d %d", mouse_x, mouse_y);
		send_command(cmdbuf);
	}

	if (mouse_wheel != 0) {
		snprintf(cmdbuf, 32, "wh %d", mouse_wheel);
		send_command(cmdbuf);
	}

	return 0;
}
