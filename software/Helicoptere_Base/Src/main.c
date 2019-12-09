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
volatile uint8_t SYSTICK_1msEvent=0;
void ReceptionCallback(char* data, uint16_t size);
void ConvertOutputValues (int32_t valAnglePitch,int32_t valAngleYaw,int32_t valMdpsPitch,int32_t valMdpsYaw);

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
const uint32_t DAC_CALIBRATION[]={3845,3900,3890,3885}; // DAC values for 10V output, per channel
const uint32_t ADC_CALIBRATION[]={0,0}; //ADC values for 10V input, per channel

char SendBuffer[100];
char ResetString[] = "ATRS\n\r\0";

volatile int32_t testAnglePitch, testAngleYaw, testMdpsPitch, testMdpsYaw=0;
volatile int32_t valAnglePitch,valAngleYaw,valMdpsPitch,valMdpsYaw=0;

volatile int messageCounter=0;
volatile char messageReceived=0;
volatile int counter_10ms=0;
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
	DEBUG_Init();
	LED_Init();
	LED_SetMode(LED_MODE_ON);

	ADC_Init();
	SPIDAC_Init();

	UART_Init();

	/* Envoi de la commande Reset à la carte embarqué (arret moteurs + recalibration AHRS) */
	UART_SendData(ResetString, strlen(ResetString));

	/* USER CODE BEGIN 2 */


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	UART_AddReceptionCallback(ReceptionCallback);
	UART_StartReception();

	LED_SetMode(LED_MODE_ERROR);

	while (1)
	{
		/* USER CODE END WHILE */
		if (SYSTICK_1msEvent==1)
		{
			SYSTICK_1msEvent=0;

			counter_10ms++;
			if (counter_10ms>=10)
			{
				counter_10ms=0;
				// Get command value (input), convert them to millivolt (from 0 to 10000 millivolt) and send them to embedded board
				InputValues[FRONT_CMD]=ADC_GetChannelVoltage(ADC_CHANNEL_FRONT);
				InputValues[REAR_CMD]=ADC_GetChannelVoltage(ADC_CHANNEL_REAR);

				DEBUG_ENTERSECTION(DEBUG_SECTION_1);
				sprintf (SendBuffer,">%lu,%lu\n\r",InputValues[FRONT_CMD],InputValues[REAR_CMD]);
				UART_SendData(SendBuffer, strlen(SendBuffer));
				DEBUG_LEAVESECTION(DEBUG_SECTION_1);
			}

			// Send output values to dac
			SPIDAC_SetValue(SPIDAC_CHANNEL_ANGLE_PITCH, (uint16_t)OutputValues[ANGLE_PITCH]);
			SPIDAC_SetValue(SPIDAC_CHANNEL_MDPS_PITCH, (uint16_t)OutputValues[MDPS_PITCH]);
			SPIDAC_SetValue(SPIDAC_CHANNEL_ANGLE_YAW, (uint16_t)OutputValues[ANGLE_YAW]);
			SPIDAC_SetValue(SPIDAC_CHANNEL_MDPS_YAW, (uint16_t)OutputValues[MDPS_YAW]);
			SPIDAC_LatchInputs();

			messageCounter++;

			if (messageCounter>=1000) // pas de message valable recu depuis 1 seconde
			{
				messageCounter=0;
				messageReceived=0;

				LED_SetMode(LED_MODE_ERROR);
			}
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
void ConvertOutputValues (int32_t valAnglePitch,int32_t valAngleYaw,int32_t valMdpsPitch,int32_t valMdpsYaw)
{
	// les angles sont donnés en centieme de degree et les vitesses en centieme de dps

	// Saturation de anglePitch (-45<=anglePitch<=45)
	if (valAnglePitch<-4500)
		valAnglePitch=-4500;
	else if (valAnglePitch>4500)
		valAnglePitch=4500;

	// ramener la sortie dans l'intervalle [0,10V] pour la plage d'angle [-45°,45°]
	OutputValues[ANGLE_PITCH] = ((valAnglePitch+4500)*10000l/9000l);

	// Utiliser la calibration du DAC pour rester dans l'interval [0,10V]
	OutputValues[ANGLE_PITCH] = OutputValues[ANGLE_PITCH]* DAC_CALIBRATION[ANGLE_PITCH]/10000l;


	// Saturation de mdpsPitch (-2000<=valMdpsPitch<=2000)
	if (valMdpsPitch<-2000)
		valMdpsPitch=-2000;
	else if (valMdpsPitch>2000)
		valMdpsPitch=2000;

	// ramener la sortie dans l'intervalle [0,10V] pour la plage d'angle [-20000 mdps,20000 mdps]
	OutputValues[MDPS_PITCH] = ((valMdpsPitch+2000)*10000l/4000l);

	// Utiliser la calibration du DAC pour rester dans l'interval [0,10V]
	OutputValues[MDPS_PITCH] = OutputValues[MDPS_PITCH]* DAC_CALIBRATION[MDPS_PITCH]/10000l;


	// Saturation de angleYaw (-180<=valAngleYaw<=180)
	if (valAngleYaw<-18000)
		valAngleYaw=-18000;
	else if (valAngleYaw>18000)
		valAngleYaw=18000;

	// ramener la sortie dans l'intervalle [0,10V] pour la plage d'angle [-180°,180°]
	OutputValues[ANGLE_YAW] = ((valAngleYaw+18000)*10000l/36000l);

	// Utiliser la calibration du DAC pour rester dans l'interval [0,10V]
	OutputValues[ANGLE_YAW] = OutputValues[ANGLE_YAW]* DAC_CALIBRATION[ANGLE_YAW]/10000l;


	// Saturation de mdpsYaw (-2000<=valMdpsYaw<=2000)
	if (valMdpsYaw<-2000)
		valMdpsYaw=-2000;
	else if (valMdpsYaw>2000)
		valMdpsYaw=2000;

	// ramener la sortie dans l'intervalle [0,10V] pour la plage d'angle [-20000 mdps,20000 mdps]
	OutputValues[MDPS_YAW] = ((valMdpsYaw+2000)*10000l/4000l);

	// Utiliser la calibration du DAC pour rester dans l'interval [0,10V]
	OutputValues[MDPS_YAW] = OutputValues[MDPS_YAW]* DAC_CALIBRATION[MDPS_YAW]/10000l;
}

void ReceptionCallback(char* data, uint16_t size)
{
	char *ptr=NULL;
	char *dummy=NULL;

	if (data[0] == '<')
	{
		ptr = &data[1];
		char delim[]=",";
		char *split[4]={NULL};

		split[0] = strtok(ptr, delim);
		split[1] = strtok(NULL, delim);
		split[2] = strtok(NULL, delim);
		split[3] = strtok(NULL, delim);

		valAnglePitch=strtol(split[0],&dummy,10); // Convert first part (Pitch angle) from string to long int in base 10
		valMdpsPitch=strtol(split[1],&dummy,10); // Convert second part (Pitch mdps) from string to long int in base 10
		valAngleYaw=strtol(split[2],&dummy,10); // Convert third part (Yaw angle) from string to long int in base 10
		valMdpsYaw=strtol(split[3],&dummy,10); // Convert fourth part (yaw mdps) from string to long int in base 10

		ConvertOutputValues (valAnglePitch,valAngleYaw,valMdpsPitch,valMdpsYaw);

		messageCounter=0;

		if (messageReceived==0)
		{
			messageReceived=1;
			LED_SetMode(LED_MODE_RUN);
		}
	}
	else //unknown frame, drop it
	{
		// nothing to do
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
