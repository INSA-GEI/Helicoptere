/**
 ******************************************************************************
 * File Name          : TIM.c
 * Description        : This file provides code for the configuration
 *                      of the TIM instances.
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

/* Includes ------------------------------------------------------------------*/
#include <led.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
extern void Error_Handler(void);

#define LED_PERIOD 1000

/* LED init function */
void LED_Init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 32000;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = LED_PERIOD;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}

	LED_SetMode(LED_MODE_IDLE);
	LED_MspPostInit(&htim2);

	HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_Base_Start(&htim2);

}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

	if(tim_pwmHandle->Instance==TIM2)
	{
		/* USER CODE BEGIN TIM2_MspInit 0 */

		/* USER CODE END TIM2_MspInit 0 */
		/* TIM2 clock enable */
		__HAL_RCC_TIM2_CLK_ENABLE();
		/* USER CODE BEGIN TIM2_MspInit 1 */

		/* USER CODE END TIM2_MspInit 1 */
	}
}
void LED_MspPostInit(TIM_HandleTypeDef* timHandle)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(timHandle->Instance==TIM2)
	{
		/* USER CODE BEGIN TIM2_MspPostInit 0 */

		/* USER CODE END TIM2_MspPostInit 0 */

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**TIM2 GPIO Configuration
    PB3     ------> TIM2_CH2 
		 */
		GPIO_InitStruct.Pin = LED_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
		HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

		/* USER CODE BEGIN TIM2_MspPostInit 1 */

		/* USER CODE END TIM2_MspPostInit 1 */
	}

}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

	if(tim_pwmHandle->Instance==TIM2)
	{
		/* USER CODE BEGIN TIM2_MspDeInit 0 */

		/* USER CODE END TIM2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM2_CLK_DISABLE();
		/* USER CODE BEGIN TIM2_MspDeInit 1 */

		/* USER CODE END TIM2_MspDeInit 1 */
	}
} 

/* USER CODE BEGIN 1 */

/**
 * @brief LED Set
 * @param None
 * @retval None
 */
void LED_SetMode(int mode)
{
	switch (mode)
	{
	case LED_MODE_IDLE:
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, LED_PERIOD/10);
		__HAL_TIM_SET_AUTORELOAD(&htim2, LED_PERIOD);
		break;
	case LED_MODE_RUN:
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, LED_PERIOD/2);
		__HAL_TIM_SET_AUTORELOAD(&htim2, LED_PERIOD);
		break;
	case LED_MODE_ERROR:
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, LED_PERIOD/20);
		__HAL_TIM_SET_AUTORELOAD(&htim2, LED_PERIOD/10);
		break;
	case LED_MODE_OFF:
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_AUTORELOAD(&htim2, LED_PERIOD);
		break;
	case LED_MODE_ON:
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, LED_PERIOD);
		__HAL_TIM_SET_AUTORELOAD(&htim2, LED_PERIOD);
		break;
	default:
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, LED_PERIOD/10);
		__HAL_TIM_SET_AUTORELOAD(&htim2, LED_PERIOD);
	}

	__HAL_TIM_SET_COUNTER(&htim2,0);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
