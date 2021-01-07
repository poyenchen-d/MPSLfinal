/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

#include "i2c.h"
#include "keypad.h"
#include "uart.h"
#include "nrf24.h"
#include "MY_NRF24.h"
#include "kalman.h"
#include "MPU6050/inv_mpu.h"
#include "MPU6050/mpu6050.h"

#include "app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD_TO_DEG (57.295779513082320876798154814105)
#define DEG_TO_RAD (0.01745329251994329576923690768489)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t TxpipeAddrs[5] = {0x2f,0xa6,0x37,0x89,0x73};
char myTxData[32] = "Hello World!";
char AckPayload[32];

//RawData_Def myAccelRaw, myGyroRaw;
//ScaledData_Def myAccelScaled, myGyroScaled;
/*
void avgMPU6050_2000()
{
	RawData_Def avgAccelRaw, avgGyroRaw;

	long accel_x = 0;
	long accel_y = 0;
	long accel_z = 0;
	long gyro_x = 0;
	long gyro_y = 0;
	long gyro_z = 0;
	for (int i = 0; i < 2000; i++) {
		MPU6050_Get_Accel_RawData(&avgAccelRaw);
		MPU6050_Get_Gyro_RawData(&avgGyroRaw);
		accel_x += avgAccelRaw.x;
		accel_y += avgAccelRaw.y;
		accel_z += avgAccelRaw.z;
		gyro_x += avgGyroRaw.x;
		gyro_y += avgGyroRaw.y;
		gyro_z += avgGyroRaw.z;

		HAL_Delay(20);
	}

	USART_printf(&huart3, "Accel:\r\n");
	USART_printf(&huart3, "\tx: %ld:\r\n", (accel_x / 2000)-140);
	USART_printf(&huart3, "\ty: %ld:\r\n", (accel_y / 2000)-137);
	USART_printf(&huart3, "\tz: %ld:\r\n", (accel_z / 2000)+38);
	USART_printf(&huart3, "Gyro:\r\n");
	USART_printf(&huart3, "\tx: %ld:\r\n", (gyro_x / 2000)+56);
	USART_printf(&huart3, "\ty: %ld:\r\n", (gyro_y / 2000)-30);
	USART_printf(&huart3, "\tz: %ld:\r\n", (gyro_z / 2000)+44);
	struct Euler p;
	struct Quaternion q;

	float Euler_x = 0;
	float Euler_y = 0;
	float Euler_z = 0;
	MPU6050_ToEuler(&p,  Quaternion *q);

	USART_printf(&huart3, "Euler:\r\n");
	USART_printf(&huart3, "\tx: %f:\r\n", Euler_x);
	USART_printf(&huart3, "\ty: %f:\r\n", Euler_y);
	USART_printf(&huart3, "\tz: %f:\r\n", Euler_z);
}

*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	MPU_ConfigTypeDef myMpuConfig;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
	DEF_UART = &huart3;
	nrf24_DebugUART_Init(huart3);
	NRF24_begin(GPIOB, CSNpin_Pin, GPIO_PIN_9, hspi1);
	NRF24_powerUp();

	NRF24_stopListening();
	NRF24_openWritingPipe(TxpipeAddrs);
	NRF24_setAutoAck(true);
	NRF24_setChannel(82);
	NRF24_setPayloadSize(32);
	NRF24_setPALevel(RF24_PA_0dB);
	NRF24_setDataRate(RF24_2MBPS);
	NRF24_setRetries(0x3, 3);
	NRF24_setCRCLength(RF24_CRC_8);

	NRF24_enableDynamicPayloads();
	// NRF24_enableAckPayload();
	printRadioSettings();
	//  nRF24_DumpConfig();

    mpu6050_i2c = &hi2c2;
    USART_printf(&huart3, "Connected: %u\r\n", MPU6050_isConnected());
    if (MPU6050_Init()) {
    	USART_printf(&huart3, "Failed to initialize MPU6050.\r\n");
    }

    struct Quaternion q;
	q.w = 1;
	q.x = 0;
	q.y = 0;
	q.z = 0;

	keypad_init();
	KeypadState_t kpstate = 0;

	struct Kalman kalmanX, kalmanZ;
	Kalman_Init(&kalmanX);
	Kalman_Init(&kalmanZ);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t gyro_ts = HAL_GetTick();
	uint32_t last_print = gyro_ts;

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		struct Euler p;
		struct Gravity g;
		struct YPR y;

		int16_t gyro_raw[3], accel_raw[3];
		mpu_get_gyro_reg(gyro_raw, NULL);
		mpu_get_accel_reg(accel_raw, NULL);
		uint32_t ts = HAL_GetTick();

		struct ScaledData accel, gyro;
		MPU6050_ToAccelScaled(&accel, accel_raw);
		MPU6050_ToGyroScaled(&gyro, gyro_raw);

		MPU6050_ToQuaternion(&q, &accel, &gyro, (ts - gyro_ts) / 1000.0);

		MPU6050_ToEuler(&p,  &q);
		MPU6050_ToGravity(&g, &q);
		MPU6050_ToYPR(&y, &q, &g);

		// Kalman filter
		float kalman_x = Kalman_getAngle(&kalmanX, y.roll * RAD_TO_DEG, gyro_raw[0] / gyroRateFactor, (ts - gyro_ts) / 1000.0);
		float kalman_z = Kalman_getAngle(&kalmanZ, y.yaw * RAD_TO_DEG, gyro_raw[2] / gyroRateFactor, (ts - gyro_ts) / 1000.0);
		gyro_ts = ts;

		struct YPR kalman_ypr;
		kalman_ypr.yaw = kalman_z * DEG_TO_RAD;
		kalman_ypr.pitch = y.pitch;
		kalman_ypr.roll = kalman_x * DEG_TO_RAD;

		enum KeypadEvent events[16];
		kpstate = keypad_scan_diff(events, kpstate);
		process_key(kpstate, events, &kalman_ypr);


		uint32_t print_ts = HAL_GetTick();
		if (print_ts - last_print > 1000) {
			USART_printf(&huart3, "Gyro raw:\r\n");
			USART_printf(&huart3, "\tx: %d:\r\n", gyro_raw[0]);
			USART_printf(&huart3, "\ty: %d:\r\n", gyro_raw[1]);
			USART_printf(&huart3, "\tz: %d:\r\n", gyro_raw[2]);
			USART_printf(&huart3, "Accel raw:\r\n");
			USART_printf(&huart3, "\tx: %d:\r\n", accel_raw[0]);
			USART_printf(&huart3, "\ty: %d:\r\n", accel_raw[1]);
			USART_printf(&huart3, "\tz: %d:\r\n", accel_raw[2]);
			USART_printf(&huart3, "Quaternion:\r\n");
			USART_printf(&huart3, "\tw: %f:\r\n", q.w);
			USART_printf(&huart3, "\tx: %f:\r\n", q.x);
			USART_printf(&huart3, "\ty: %f:\r\n", q.y);
			USART_printf(&huart3, "\tz: %f:\r\n", q.z);
			USART_printf(&huart3, "Euler:\r\n");
			USART_printf(&huart3, "\tx: %f:\r\n", p.x);
			USART_printf(&huart3, "\ty: %f:\r\n", p.y);
			USART_printf(&huart3, "\tz: %f:\r\n", p.z);
			USART_printf(&huart3, "YPR:\r\n");
			USART_printf(&huart3, "\tyaw: %f:\r\n", y.yaw);
			USART_printf(&huart3, "\tpitch: %f:\r\n", y.pitch);
			USART_printf(&huart3, "\troll: %f:\r\n", y.roll);
			USART_printf(&huart3, "Kalman YPR:\r\n");
			USART_printf(&huart3, "\tyaw: %f:\r\n", kalman_ypr.yaw);
			USART_printf(&huart3, "\tpitch: %f:\r\n", kalman_ypr.pitch);
			USART_printf(&huart3, "\troll: %f:\r\n", kalman_ypr.roll);

			last_print = print_ts;
		}
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00909BEB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, X0_Pin|X1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, X2_Pin|X3_Pin|CSNpin_Pin|CEpin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : X0_Pin X1_Pin */
  GPIO_InitStruct.Pin = X0_Pin|X1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Y2_Pin Y1_Pin Y3_Pin */
  GPIO_InitStruct.Pin = Y2_Pin|Y1_Pin|Y3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : X2_Pin X3_Pin */
  GPIO_InitStruct.Pin = X2_Pin|X3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Laser_Pin */
  GPIO_InitStruct.Pin = Laser_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Laser_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Y0_Pin */
  GPIO_InitStruct.Pin = Y0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Y0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CSNpin_Pin CEpin_Pin */
  GPIO_InitStruct.Pin = CSNpin_Pin|CEpin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
