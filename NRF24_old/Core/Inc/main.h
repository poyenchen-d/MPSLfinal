/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Y1_Pin GPIO_PIN_13
#define Y1_GPIO_Port GPIOC
#define Y2_Pin GPIO_PIN_14
#define Y2_GPIO_Port GPIOC
#define Y3_Pin GPIO_PIN_15
#define Y3_GPIO_Port GPIOC
#define X0_Pin GPIO_PIN_0
#define X0_GPIO_Port GPIOC
#define X1_Pin GPIO_PIN_1
#define X1_GPIO_Port GPIOC
#define X2_Pin GPIO_PIN_2
#define X2_GPIO_Port GPIOC
#define X3_Pin GPIO_PIN_3
#define X3_GPIO_Port GPIOC
#define Laser_Pin GPIO_PIN_3
#define Laser_GPIO_Port GPIOA
#define Y0_Pin GPIO_PIN_12
#define Y0_GPIO_Port GPIOC
#define CSNpin_Pin GPIO_PIN_8
#define CSNpin_GPIO_Port GPIOB
#define CEpin_Pin GPIO_PIN_9
#define CEpin_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
