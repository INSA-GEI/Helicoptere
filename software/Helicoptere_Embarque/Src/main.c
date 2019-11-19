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
#include "stm32l4xx_hal.h"

#include "stdio.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void RxXBEEData(char* data, uint16_t size);
void GetAndUpdateSensors(void);
void SystickRequestAHRSUpdate(void);

//const char *testStr1="Ca a l'air de marcher\n";
char printfBuffer[100];

acceleration_t acceleration;
angularRate_t angular_rate;

//int acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, temp=0;
//float int_gyro_x,  int_gyro_y, int_gyro_z=0.0;
//int int_gyro_x_int,  int_gyro_y_int, int_gyro_z_int=0;

typedef struct
{
	int32_t pitch;
	int32_t roll;
	int32_t yaw;
} eulerAnglesInt_t;

eulerAnglesInt_t EulerAnglesInt;

eulerAngles_t EulerAngles;
eulerAnglesInt_t EulerAnglesIntTmp[5];
int flagSensors;
int systickRequestFlag;

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
	int counter=0;
	int counter2=0;
	/* USER CODE BEGIN 1 */
	HAL_DeInit();
	/* USER CODE END 1 */


	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	SystemCoreClockUpdate();
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();

	/* USER CODE BEGIN 2 */

	/* Demarre le timer de la led */
	LED_Init();

	/* Demarre le timer des moteurs */
	MOTORS_Init();
	MOTORS_SetHead(0);
	MOTORS_SetTail(0);

	/* Demarre l'USART1 (XBEE) */
	XBEE_Init();
	XBEE_AddReceptionCallback(RxXBEEData);
	XBEE_StartReception();

	/* Demarre l'USART2 (BASECOM) */
	BASECOM_Init();

	/* Demarre l'accelerometre et gyroscope */
	flagSensors=0;
	systickRequestFlag=0;

	if (ACC_GYRO_Init() != ACC_OK) {
		LED_SetMode(LED_MODE_ERROR);
		sprintf (printfBuffer, "Error initializing sensors: Stop\n");
		XBEE_SendData((char*)printfBuffer, strlen(printfBuffer));

		while (1);
	}

	AHRS_Init();
	HAL_GPIO_EXTI_Callback(GPIO_PIN_1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//HAL_Delay(10);

		//GYRO_ReadValues(&angular_rate);

		if (systickRequestFlag==1)
		{
			systickRequestFlag=0;

			SystickRequestAHRSUpdate();
		}

		if (flagSensors==1)
		{
			flagSensors=0;
			AHRS_UpdateQuaternions();

			AHRS_GetEulerAngles(&EulerAngles);
			EulerAnglesIntTmp[counter].pitch = (int32_t)(EulerAngles.pitch*100.0);
			EulerAnglesIntTmp[counter].roll = (int32_t)(EulerAngles.roll*100.0);
			EulerAnglesIntTmp[counter].yaw = (int32_t)(EulerAngles.yaw*100.0);

			counter ++;
			counter2++;

			if (counter>=5)
			{
				int i;
				counter=0;

				for (i=0; i<5; i++)
				{
					EulerAngles.pitch += EulerAnglesIntTmp[i].pitch;
					EulerAngles.roll += EulerAnglesIntTmp[i].roll;
					EulerAngles.yaw += EulerAnglesIntTmp[i].yaw;
				}

				EulerAngles.pitch = EulerAngles.pitch/5;
				EulerAngles.roll = EulerAngles.roll/5;
				EulerAngles.yaw = EulerAngles.yaw/5;
			}

			if (counter2>=100)
			{
				counter2 =0;

				sprintf (printfBuffer, "Angles [%d,\t %d,\t %d]\n",
						(int)(EulerAngles.pitch/100),
						(int)(EulerAngles.roll/100),
						(int)(EulerAngles.yaw/100));

				XBEE_SendData((char*)printfBuffer, strlen(printfBuffer));
			}
		}

		//		MOTORS_SetTail(250);
		//		LED_SetMode(LED_MODE_IDLE);
		//		HAL_Delay(5000);
		//
		//		MOTORS_SetTail(500);
		//		LED_SetMode(LED_MODE_RUN);
		//		HAL_Delay(5000);
		//
		//		MOTORS_SetTail(750);
		//		LED_SetMode(LED_MODE_ERROR);
		//		HAL_Delay(5000);
		//
		//		MOTORS_SetTail(1000);
		//		LED_SetMode(LED_MODE_ON);
		//		HAL_Delay(5000);
		//__WFI();
	}
	/* USER CODE END 3 */
}

/**
 * @brief Xbee RX Callback
 * @retval None
 */
void RxXBEEData(char* data, uint16_t size)
{
	static uint32_t cnt=0;
	volatile char c;

	c= data[0];

	cnt++;
	if (cnt==2) XBEE_StopReception();
}

void SystickRequestAHRSUpdate(void)
{
	static int counter=0;
	counter++;

	if (counter>=2)
	{
		counter=0;

		if (AHRS_Status == AHRS_RUN)
			GetAndUpdateSensors();
	}
}

void GetAndUpdateSensors(void)
{
	//	static uint8_t counter=0;
	acceleration_t acceleration_loc;
	angularRate_t angular_rate_loc;

	if (GYRO_ReadValues(&angular_rate_loc)==ACC_OK)
	{
		angular_rate.x = angular_rate_loc.x;
		angular_rate.y = angular_rate_loc.y;
		angular_rate.z = angular_rate_loc.z;
	}

	if (ACC_ReadValues(&acceleration_loc)==ACC_OK)
	{
		acceleration.x = acceleration_loc.x;
		acceleration.y = acceleration_loc.y;
		acceleration.z = acceleration_loc.z;
	}

	AHRS_UpdateSensors(&acceleration, &angular_rate);
	flagSensors=1;
}

/**
 * @brief GPIO EXTI Callback
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//	static uint8_t counter=0;
	acceleration_t acceleration_loc;
	angularRate_t angular_rate_loc;

	if (GYRO_ReadValues(&angular_rate_loc)==ACC_OK)
	{
		angular_rate.x = angular_rate_loc.x;
		angular_rate.y = angular_rate_loc.y;
		angular_rate.z = angular_rate_loc.z;
	}

	if (ACC_ReadValues(&acceleration_loc)==ACC_OK)
	{
		acceleration.x = acceleration_loc.x;
		acceleration.y = acceleration_loc.y;
		acceleration.z = acceleration_loc.z;
	}

	AHRS_UpdateSensors(&acceleration, &angular_rate);
	flagSensors=1;

	//	counter++;
	//
	//	if(counter>=4)
	//	{
	//		counter =0;
	//		flagSensors=1;
	//	}
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

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 20;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
			|RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
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
void assert_failed(char *file, uint32_t line)
{ 
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
