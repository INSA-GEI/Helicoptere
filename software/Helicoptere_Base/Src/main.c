/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "adc.h"

#include "spidac.h"
#include "led.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint8_t SYSTICK_1msEvent=0;
void ReceptionCallback(char* data, uint16_t size);

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#define FRONT_CMD	0
#define REAR_CMD	1

#define ANGLE_PITCH	0
#define MDPS_PITCH	1
#define ANGLE_YAW	2
#define MDPS_YAW 	3

/* USER CODE BEGIN PV */
uint32_t InputValues[2]={0};
uint32_t OutputValues[4]={0};

char SendBuffer[20];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

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

	ADC_Init();
	SPIDAC_Init();
	LED_Init();
	UART_Init();
	/* USER CODE BEGIN 2 */
	LED_SetMode(LED_MODE_OFF);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	LED_SetMode(LED_MODE_RUN);
	UART_AddReceptionCallback(ReceptionCallback);
	UART_StartReception();

	while (1)
	{
		/* USER CODE END WHILE */
		if (SYSTICK_1msEvent==1)
		{
			SYSTICK_1msEvent=0;

			InputValues[FRONT_CMD]=ADC_GetChannelVoltage(ADC_CHANNEL_FRONT);
			InputValues[REAR_CMD]=ADC_GetChannelVoltage(ADC_CHANNEL_REAR);

			sprintf (SendBuffer,">%lu,%lu\n",InputValues[FRONT_CMD],InputValues[REAR_CMD]);
			UART_SendData(SendBuffer, strlen(SendBuffer));

			SPIDAC_SetValue(SPIDAC_CHANNEL_ANGLE_PITCH, (uint16_t)OutputValues[ANGLE_PITCH]);
			SPIDAC_SetValue(SPIDAC_CHANNEL_MDPS_PITCH, (uint16_t)OutputValues[MDPS_PITCH]);
			SPIDAC_SetValue(SPIDAC_CHANNEL_ANGLE_YAW, (uint16_t)OutputValues[ANGLE_YAW]);
			SPIDAC_SetValue(SPIDAC_CHANNEL_MDPS_YAW, (uint16_t)OutputValues[MDPS_YAW]);
			SPIDAC_LatchInputs();

		}
		/* USER CODE BEGIN 3 */
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

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void ReceptionCallback(char* data, uint16_t size)
{
	uint32_t angle_pitch, angle_yaw, mdps_pitch, mdps_yaw=0;

	if (data[0]=='<') // frame correctly written
	{
		sscanf (data, "<%lu,%lu,%lu,%lu\n",
				&angle_pitch,&mdps_pitch,
				&angle_yaw,&mdps_yaw);

		// todo: prevoir des convertion d'echelle et de format
		OutputValues[ANGLE_PITCH]=(uint16_t)angle_pitch;
		OutputValues[MDPS_PITCH]=(uint16_t)mdps_pitch;
		OutputValues[ANGLE_YAW]=(uint16_t)angle_yaw;
		OutputValues[MDPS_YAW]=(uint16_t)mdps_yaw;
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
