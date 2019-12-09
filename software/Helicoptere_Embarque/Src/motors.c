/**
 ******************************************************************************
 * @file           : motors.c
 * @brief          : Motors program body
 * @author         : dimercur
 * @date           : Nov 14, 2019
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

#include "motors.h"

TIM_HandleTypeDef htim1;
extern void Error_Handler(void);
static void MOTORS_MspPostInit();

#define MOTORS_PWM_PERIOD 		800

/* HAL_TIM_Base_MspInit and HAL_TIM_Base_MspDeInit are defined in stm32l4xx_hal_msp.c and used by HAL_TIM_Base_Init
 * Both call LED_MspInit (or LED_MspDeInit) if timer is tim2
 */

/**** Private Functions ****/
void MOTORS_MspInit()
{
	__HAL_RCC_TIM1_CLK_ENABLE();
}

void MOTORS_MspDeInit()
{
	__HAL_RCC_TIM1_CLK_DISABLE();
}

void MOTORS_MspPostInit(TIM_HandleTypeDef* htim)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1
    PA11     ------> TIM1_CH4
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/*** Public Functions ****/

void MOTORS_Init()
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 800;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}

	MOTORS_MspPostInit(&htim1);


	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
}

void MOTORS_SetHead(uint32_t speed)
{
	uint16_t tmp;

	if (speed > MOTORS_CMD_MAX_RANGE) speed = MOTORS_CMD_MAX_RANGE;
	tmp = speed*MOTORS_PWM_PERIOD / MOTORS_CMD_MAX_RANGE;

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, tmp);
	//__HAL_TIM_SET_AUTORELOAD(&htim1, MOTORS_PWM_PERIOD);
	__HAL_TIM_SET_COUNTER(&htim1,0);
}

void MOTORS_SetTail(uint32_t speed)
{
	uint16_t tmp;

	if (speed > MOTORS_CMD_MAX_RANGE) speed = MOTORS_CMD_MAX_RANGE;
	tmp = speed*MOTORS_PWM_PERIOD / MOTORS_CMD_MAX_RANGE;

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, tmp);
	//__HAL_TIM_SET_AUTORELOAD(&htim1, MOTORS_PWM_PERIOD);
	__HAL_TIM_SET_COUNTER(&htim1,0);
}
