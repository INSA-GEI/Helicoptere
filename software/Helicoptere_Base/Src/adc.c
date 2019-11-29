/**
 ******************************************************************************
 * File Name          : ADC.c
 * Description        : This file provides code for the configuration
 *                      of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

#define ADC_BUFFER_SIZE 16
#define ADC_CHANNEL_0_MAX 0x300
#define ADC_CHANNEL_1_MAX 0x300

uint16_t ADC_RawBuffer[ADC_BUFFER_SIZE][2]={0};
uint16_t ADC_NormalizedBuffer[2];

void ADC_DMACptCallback(DMA_HandleTypeDef *hdma);

/* ADC init function */
void ADC_Init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	hadc.Instance = ADC1;
	if (HAL_ADC_DeInit(&hadc) != HAL_OK)
	{
		/* ADC de-initialization Error */
		Error_Handler();
	}

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.OversamplingMode = DISABLE;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.SamplingTime= ADC_SAMPLETIME_12CYCLES_5;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ContinuousConvMode = ENABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.DMAContinuousRequests = ENABLE;
	hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerFrequencyMode = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	if (HAL_ADC_Init(&hadc) != HAL_OK)
	{
		Error_Handler();
	}

	/* ### - 2 - Start calibration ############################################ */
	if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED)!= HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADC_RawBuffer, 2*ADC_BUFFER_SIZE) != HAL_OK)
	{
		/* ADC de-initialization Error */
		Error_Handler();
	}
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(adcHandle->Instance==ADC1)
	{
		/* USER CODE BEGIN ADC1_MspInit 0 */

		/* USER CODE END ADC1_MspInit 0 */
		/* ADC1 clock enable */
		__HAL_RCC_ADC1_CLK_ENABLE();
		__HAL_RCC_DMA1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**ADC GPIO Configuration
    PA0-CK_IN     ------> ADC_IN0
    PA1     ------> ADC_IN1 
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* ADC1 DMA Init */
		/* ADC Init */
		hdma_adc.Instance = DMA1_Channel1;
		hdma_adc.Init.Request = DMA_REQUEST_0;
		hdma_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_adc.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_adc.Init.MemInc = DMA_MINC_ENABLE;
		hdma_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		hdma_adc.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		hdma_adc.Init.Mode = DMA_CIRCULAR;
		hdma_adc.Init.Priority = DMA_PRIORITY_LOW;
		if (HAL_DMA_Init(&hdma_adc) != HAL_OK)
		{
			Error_Handler();
		}

		__HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc);

		/* USER CODE BEGIN ADC1_MspInit 1 */
		/* DMA interrupt init */
		/* DMA1_Channel1_IRQn interrupt configuration */


		HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
		/* USER CODE END ADC1_MspInit 1 */
	}
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

	if(adcHandle->Instance==ADC1)
	{
		/* USER CODE BEGIN ADC1_MspDeInit 0 */

		/* USER CODE END ADC1_MspDeInit 0 */
		/* Peripheral clock disable */

		__HAL_RCC_ADC1_CLK_DISABLE();

		/**ADC GPIO Configuration
    PA0-CK_IN     ------> ADC_IN0
    PA1     ------> ADC_IN1 
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

		/* ADC1 DMA DeInit */
		HAL_DMA_DeInit(adcHandle->DMA_Handle);
		/* USER CODE BEGIN ADC1_MspDeInit 1 */

		/* USER CODE END ADC1_MspDeInit 1 */
	}
} 

/* USER CODE BEGIN 1 */
uint16_t ADC_GetChannelRaw(uint8_t channel)
{
	if ((channel!=ADC_CHANNEL_FRONT) && (channel !=ADC_CHANNEL_REAR ))
	{
		Error_Handler();
	}

	return ADC_NormalizedBuffer[channel];
}

uint16_t ADC_GetChannelVoltage(uint8_t channel)
{
	uint32_t tmp=0;

	if ((channel!=ADC_CHANNEL_FRONT) && (channel !=ADC_CHANNEL_REAR ))
	{
		Error_Handler();
	}

	if (channel == ADC_CHANNEL_FRONT)
	{
		tmp = ((uint32_t)ADC_NormalizedBuffer[0] * 10000)/ (uint32_t)ADC_CHANNEL_0_MAX;
	}
	else
	{
		tmp = ((uint32_t)ADC_NormalizedBuffer[1] * 10000)/ (uint32_t)ADC_CHANNEL_1_MAX;
	}

	if (tmp>10000) tmp=10000;

	return (uint16_t)tmp;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint32_t acc0, acc1=0;

	for (int i=0; i<ADC_BUFFER_SIZE; i++)
	{
		acc0 += ADC_RawBuffer[i][0];
		acc1 += ADC_RawBuffer[i][1];
	}

	ADC_NormalizedBuffer[0] = acc0/ADC_BUFFER_SIZE;
	ADC_NormalizedBuffer[1] = acc1/ADC_BUFFER_SIZE;

	//HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADC_RawBuffer, 2*ADC_BUFFER_SIZE);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
