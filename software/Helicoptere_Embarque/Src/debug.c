/**
 ******************************************************************************
 * File Name          : debug.c
 * Description        : This file provides code for the configuration
 *                      of the debug services.
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
#include <debug.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


/* DEBUG init function */
void DEBUG_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStruct.Pin = DEBUG_SECTION_1_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(DEBUG_SECTION_1_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DEBUG_SECTION_2_PIN;
	HAL_GPIO_Init(DEBUG_SECTION_2_PORT, &GPIO_InitStruct);

	DEBUG_LEAVESECTION(DEBUG_SECTION_1);
	DEBUG_LEAVESECTION(DEBUG_SECTION_2);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
