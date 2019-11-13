/**
 ******************************************************************************
 * @file           : led.c
 * @brief          : Led program body
 * @author         : dimercur
 * @date           : Nov 13, 2019
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 INSA-GEI.
 * All rights reserved.</center></h2>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************
 */

#include "led.h"

TIM_HandleTypeDef htim2;
extern void Error_Handler(void);
static void LED_MspPostInit();
static void LED_MspDeInit();

#define LED_PERIOD 1220

/**** Support Functions ****/
static void LED_MspPostInit()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**TIM2 GPIO Configuration
    PB3 (JTDO-TRACESWO)     ------> TIM2_CH2
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;

	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
static void LED_MspDeInit()
{
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();
}

/**
  * @brief LED Initialization Function
  * @param None
  * @retval None
  */
void LED_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 65535;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = LED_PERIOD;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

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
  sConfigOC.Pulse = 610;
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
	default:
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, LED_PERIOD/10);
		__HAL_TIM_SET_AUTORELOAD(&htim2, LED_PERIOD);
	}

	__HAL_TIM_SET_COUNTER(&htim2,0);
}




