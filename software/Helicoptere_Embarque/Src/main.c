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
#include "stdlib.h"

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
void RxBASECOMData(char* data, uint16_t size);
void GetAndUpdateSensors(void);
void SystickRequestAHRSUpdate(void);
void CmdProcessor(char org, char * data, uint16_t size);

void TASK_1msPeriodic(void);
void TASK_10msPeriodic(void);
void TASK_100msPeriodic(void);
void TASK_UpdateSensorPeriodic(void);

acceleration_t acceleration;
angularRate_t angular_rate;

typedef struct
{
	int32_t pitch;
	int32_t roll;
	int32_t yaw;
} eulerAnglesInt_t;

typedef struct
{
	float pitch;
	float roll;
	float yaw;
} eulerAngles_t;

eulerAnglesInt_t EulerAnglesInt;

eulerAngles_t EulerAngles;
eulerAnglesInt_t EulerAnglesIntTmp[5];
volatile uint8_t SYSTICK_1msEvent;

char SendXBEEBuffer[100]={0};
char SendBASECOMBuffer[100]={0};
char RcvXBEEBuffer[100]={0};
char RcvBASECOMBuffer[100]={0};

#define ORG_XBEE 		0
#define ORG_BASECOM 	1

volatile char WirelessMode;
volatile char ResetFlag;
volatile char usartBasecomFlag;
volatile char usartXbeeFlag;

volatile int messageCounter=0;
volatile char messageReceived=0;

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
	int counterTask_100ms=0;
	int counterTask_10ms=0;
	int counterTask_Sensor=0;

	/* USER CODE BEGIN 1 */
	HAL_DeInit();
	/* USER CODE END 1 */


	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_DeInit();
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
	DEBUG_Init();
	LED_Init();
	LED_SetMode(LED_MODE_ON);

	/* Demarre le timer des moteurs */
	MOTORS_Init();
	MOTORS_SetHead(0);
	MOTORS_SetTail(0);

	usartBasecomFlag=0;
	usartXbeeFlag=0;
	/* Demarre l'USART1 (XBEE) */
	XBEE_Init();
	XBEE_AddReceptionCallback(RxXBEEData);
	XBEE_StartReception();

	/* Demarre l'USART2 (BASECOM) */
	BASECOM_Init();
	BASECOM_AddReceptionCallback(RxBASECOMData);
	BASECOM_StartReception();

	/* attente de 5s, que l'helicopetere se stabilise */
	sprintf (SendXBEEBuffer, "Stabilizing system: Do not touch\n\rPlease wait 3s ...\n\r");
	XBEE_SendData((char*)SendXBEEBuffer, strlen(SendXBEEBuffer));
	HAL_Delay(3000);

	sprintf (SendXBEEBuffer, "Calibrating sensors\n\rDo not touch and wait 2s ...\n\r");
	XBEE_SendData((char*)SendXBEEBuffer, strlen(SendXBEEBuffer));
	/* Demarre l'accelerometre et gyroscope et calibre le gyroscope (le systeme ne doit plus bouger) */
	if (ACC_GYRO_Init() != ACC_OK) {
		LED_SetMode(LED_MODE_ERROR);
		sprintf (SendXBEEBuffer, "Error initializing sensors: Stop\n\r");
		XBEE_SendData((char*)SendXBEEBuffer, strlen(SendXBEEBuffer));

		while (1);
	}

	//AHRS_Init();
	MAHONY_Init(500);
	//HAL_GPIO_EXTI_Callback(GPIO_PIN_1);

	sprintf (SendXBEEBuffer, "System ready ...\n\r");
	XBEE_SendData((char*)SendXBEEBuffer, strlen(SendXBEEBuffer));

	// initialize variables, flags and counters
	SYSTICK_1msEvent=0;
	WirelessMode=0;
	ResetFlag = 0;

	LED_SetMode(LED_MODE_ERROR);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		if (ResetFlag==1)
		{
			MOTORS_SetHead(0);
			MOTORS_SetTail(0);
			LED_SetMode(LED_MODE_OFF);
			__NVIC_SystemReset();

			while(1);
		}

		if (usartBasecomFlag==1)
		{
			usartBasecomFlag=0;
			CmdProcessor(ORG_BASECOM, RcvBASECOMBuffer, strlen(RcvBASECOMBuffer));
		}

		if (usartXbeeFlag==1)
		{
			usartXbeeFlag=0;
			CmdProcessor(ORG_XBEE, RcvXBEEBuffer, strlen(RcvXBEEBuffer));
		}

		if (SYSTICK_1msEvent==1)
		{
			SYSTICK_1msEvent=0;
			counterTask_10ms++;
			counterTask_100ms++;
			counterTask_Sensor++;
			if (counterTask_Sensor>=2)
			{
				// throw every 2ms periodic task
				counterTask_Sensor=0;

				TASK_UpdateSensorPeriodic();
			}

			// Throw 1 ms periodic task
			TASK_1msPeriodic();

			if (counterTask_10ms>=10)
			{
				// throw every 10ms periodic task
				counterTask_10ms=0;
				TASK_10msPeriodic();
			}

			if (counterTask_100ms>=100)
			{
				// throw every 100ms periodic task
				counterTask_100ms=0;
				TASK_100msPeriodic();
			}

			messageCounter++;

			if (messageCounter>=1000) // pas de message valable recu depuis 1 seconde
			{
				messageCounter=0;
				messageReceived=0;

				MOTORS_SetHead(0);
				MOTORS_SetTail(0);
				LED_SetMode(LED_MODE_ERROR);
			}
		}
	}
	/* USER CODE END 3 */
}

void TASK_10msPeriodic(void)
{
	int32_t pitch, yaw, mdpsPitch, mdpsYaw;

	pitch = (int32_t)(-EulerAngles.pitch); // Already in 100's of degree, and invert sign
	yaw = (int32_t)(EulerAngles.yaw); // Already in 100's of degree
	mdpsYaw= (int32_t)(angular_rate.z/10.0); // angular_rate are in mdps, convert them in 100's of dps
	mdpsPitch= (int32_t)(angular_rate.x/10.0); // angular_rate are in mdps, convert them in 100's of dps

	// envoyer les angles en centieme de degré et les vitesses en centieme de dps
	sprintf (SendXBEEBuffer, "<%li,%li,%li,%li\n\r",
			pitch, mdpsPitch, yaw, mdpsYaw);

	//	sprintf (SendXBEEBuffer, "<%li,%li\n\r",
	//			pitch/100, yaw/100 );

	XBEE_SendData((char*)SendXBEEBuffer, strlen(SendXBEEBuffer));

	sprintf (SendBASECOMBuffer, "<%li,%li,%li,%li\n\r",
			pitch, mdpsPitch, yaw, mdpsYaw);

	BASECOM_SendData((char*)SendBASECOMBuffer, strlen(SendBASECOMBuffer));
}

void TASK_100msPeriodic(void)
{

}

void TASK_1msPeriodic(void)
{

}

void TASK_UpdateSensorPeriodic(void) // every 2 ms
{
	acceleration_t acceleration_loc={0.0,0.0,0.0};
	angularRate_t angular_rate_loc={0.0,0.0,0.0};
	static int counter =0;

	if (AHRS_Status == AHRS_RUN)
	{

		if (GYRO_ReadValues(&angular_rate_loc)==ACC_OK)
		{
			//			angular_rate.x = angular_rate_loc.x/1.63;
			//			angular_rate.y = angular_rate_loc.y/1.63;
			//			angular_rate.z = angular_rate_loc.z/1.63;

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

		//		AHRS_UpdateSensors(&acceleration, &angular_rate);
		//
		//		AHRS_UpdateQuaternions();
		//
		//		AHRS_GetEulerAngles(&EulerAngles);
		//		EulerAnglesIntTmp[counter].pitch = (int32_t)(EulerAngles.pitch*100.0);
		//		EulerAnglesIntTmp[counter].roll = (int32_t)(EulerAngles.roll*100.0);
		//		EulerAnglesIntTmp[counter].yaw = (int32_t)(EulerAngles.yaw*100.0);

		// Gyro is given in mdps, MAHONY requires it in dps
		// Accelero is given in mg, MAHONY requires it in g
		MAHONY_UpdateWithoutMag(angular_rate.x/1000.0, angular_rate.y/1000.0, angular_rate.z/1000.0,
				acceleration.x/1000.0, acceleration.y/1000.0, acceleration.z/1000.0);

		EulerAnglesIntTmp[counter].roll = (int32_t)(MAHONY_GetPitch()*100.0);
		EulerAnglesIntTmp[counter].pitch = (int32_t)(MAHONY_GetRoll()*100.0);
		EulerAnglesIntTmp[counter].yaw = (int32_t)(MAHONY_GetYaw()*100.0);
		counter++;

		if (counter>=5) //every 10ms
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
	}
}

int SearchStr(char *str, char *pattern)
{
	int length, lengthPattern;
	int index =0;
	int status=1;

	length = strlen(str);
	lengthPattern = strlen(pattern);

	if ((length == 0) || (lengthPattern==0))
		return 0;

	if (length != lengthPattern)
		return 0;

	for (index=0; index <length; index ++)
	{
		if (str[index] != pattern[index])
		{
			status=0;
			break;
		}
	}

	return status;
}

void CmdProcessor(char org, char *data, uint16_t size)
{
	char *ptr=NULL;
	char *dummy=NULL;
	uint32_t valHead,valTail=0;

	if (data[0]=='A')
	{
		if (SearchStr(data, "ATRS")!=0)
		{
			ResetFlag=1;
		}
		else if (SearchStr(data, "ATWL=1")!=0)
		{
			WirelessMode=1;
			MOTORS_SetHead(0);
			MOTORS_SetTail(0);
		}
		else if (SearchStr(data, "ATWL=0")!=0)
		{
			WirelessMode=0;
			MOTORS_SetHead(0);
			MOTORS_SetTail(0);
		}
		else if (SearchStr(data, "ATWL?")!=0)
		{
			if (WirelessMode==0)
				sprintf(SendXBEEBuffer, "0\n\r");
			else
				sprintf(SendXBEEBuffer, "1\n\r");
			XBEE_SendData(SendXBEEBuffer, strlen(SendXBEEBuffer));
		}
		else if (SearchStr(data, "AT")!=0)
		{
			sprintf(SendXBEEBuffer, "OK\n\r");
			XBEE_SendData(SendXBEEBuffer, strlen(SendXBEEBuffer));
		}

		messageCounter=0;

		if (messageReceived==0)
		{
			messageReceived=1;
			LED_SetMode(LED_MODE_RUN);
		}
	}
	else if (data[0] == '>')
	{
		if (((WirelessMode==0)&&(org==ORG_BASECOM)) || ((WirelessMode==1)&&(org==ORG_XBEE)))
		{
			ptr = &data[1];
			char delim[]=",";
			char *split[2]={NULL};

			split[0] = strtok(ptr, delim);
			split[1] = strtok(NULL, delim);

			valHead=strtol(split[0],&dummy,10); // Convert first part (head) from string to long int in base 10
			valTail=strtol(split[1],&dummy,10); // Convert first part (head) from string to long int in base 10

			// update motors
			// Maximum string values are 10000 (10000 millivolts), maximum motor command is 1000: factor 10
			// Under 1v, motor is off

			if (valHead>=1000)
				MOTORS_SetHead(valHead/10);
			else
				MOTORS_SetHead(0);

			if (valTail>=1000)
				MOTORS_SetTail(valTail/10);
			else
				MOTORS_SetTail(0);

			messageCounter=0;

			if (messageReceived==0)
			{
				messageReceived=1;
				LED_SetMode(LED_MODE_RUN);
			}
		}
	}
	else //unknown frame, drop it
	{
		// nothing to do
	}
}
/**
 * @brief Xbee RX Callback
 * @retval None
 */
void RxXBEEData(char* data, uint16_t size)
{
	usartXbeeFlag=1;

	strcpy(RcvXBEEBuffer,data);
}

/**
 * @brief BASECOM RX Callback
 * @retval None
 */
void RxBASECOMData(char* data, uint16_t size)
{
	usartBasecomFlag=1;

	strcpy(RcvBASECOMBuffer,data);
}

/**
 * @brief GPIO EXTI Callback
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//	static uint8_t counter=0;
	//	acceleration_t acceleration_loc;
	//	angularRate_t angular_rate_loc;
	//
	//	if (GYRO_ReadValues(&angular_rate_loc)==ACC_OK)
	//	{
	//		angular_rate.x = angular_rate_loc.x;
	//		angular_rate.y = angular_rate_loc.y;
	//		angular_rate.z = angular_rate_loc.z;
	//	}
	//
	//	if (ACC_ReadValues(&acceleration_loc)==ACC_OK)
	//	{
	//		acceleration.x = acceleration_loc.x;
	//		acceleration.y = acceleration_loc.y;
	//		acceleration.z = acceleration_loc.z;
	//	}
	//
	//	AHRS_UpdateSensors(&acceleration, &angular_rate);

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
