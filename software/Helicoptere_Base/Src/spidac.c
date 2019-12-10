/**
 ******************************************************************************
 * File Name          : SPI.c
 * Description        : This file provides code for the configuration
 *                      of the SPI instances.
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
#include <spidac.h>

/* USER CODE BEGIN 0 */
extern void Error_Handler(void);

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

//#define SPIDAC_OPTIMIZED_TRANSMIT

/* SPI1 init function */
void SPIDAC_Init(void)
{
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_1LINE;
	hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(spiHandle->Instance==SPI1)
	{
		/* USER CODE BEGIN SPI1_MspInit 0 */

		/* USER CODE END SPI1_MspInit 0 */
		/* SPI1 clock enable */
		__HAL_RCC_SPI1_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA7     ------> SPI1_MOSI 
    PA4     ------> DAC_SYNC, active low
		 */
		GPIO_InitStruct.Pin = DAC_CLK_Pin|DAC_DATA_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/*Configure GPIO pin Output Level */
		HAL_GPIO_WritePin(DAC_SYNC_GPIO_Port, DAC_SYNC_Pin, GPIO_PIN_SET);

		/*Configure GPIO pin : PtPin */
		GPIO_InitStruct.Pin = DAC_SYNC_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
		HAL_GPIO_Init(DAC_SYNC_GPIO_Port, &GPIO_InitStruct);

		/* USER CODE BEGIN SPI1_MspInit 1 */

		/* USER CODE END SPI1_MspInit 1 */
	}
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{
	if(spiHandle->Instance==SPI1)
	{
		/* USER CODE BEGIN SPI1_MspDeInit 0 */

		/* USER CODE END SPI1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_SPI1_CLK_DISABLE();

		/**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA7     ------> SPI1_MOSI 
		 */
		HAL_GPIO_DeInit(GPIOA, DAC_CLK_Pin|DAC_DATA_Pin | DAC_SYNC_Pin);

		/* SPI1 DMA DeInit */
		//HAL_DMA_DeInit(spiHandle->hdmatx);
		/* USER CODE BEGIN SPI1_MspDeInit 1 */

		/* USER CODE END SPI1_MspDeInit 1 */
	}
} 

/* USER CODE BEGIN 1 */

void SPIDAC_SetValue(uint8_t channel, uint16_t val)
{
	uint8_t DACSPI_Data[2] __attribute__((aligned (16))) = {0};

	if (channel>3)
		Error_Handler();
	else
	{
		DACSPI_Data[1] = (channel<<4) | (val>>8);
		DACSPI_Data[0] = val & 0xFF;

#if !defined (SPIDAC_OPTIMIZED_TRANSMIT)
		HAL_GPIO_WritePin(GPIOA, DAC_SYNC_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, DACSPI_Data, 1, 1000);
		HAL_GPIO_WritePin(GPIOA, DAC_SYNC_Pin, GPIO_PIN_SET);
#else
		GPIOA->BRR = DAC_SYNC_Pin;
		//HAL_SPI_Transmit(&hspi1, DACSPI_Data, 1, 1000);
		SPI_1LINE_TX(&hspi1);
		__HAL_SPI_ENABLE(&hspi1);
		hspi1.Instance->DR = *((uint16_t *)DACSPI_Data);

		while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY)!= RESET) {}

		__HAL_SPI_CLEAR_OVRFLAG(&hspi1);
		GPIOA->BSRR = DAC_SYNC_Pin;
#endif /* SPIDAC_OPTIMIZED_TRANSMIT */
	}
}

void SPIDAC_LatchInputs(void)
{
	uint8_t DACSPI_Data[2] __attribute__((aligned (16))) = {0};
	DACSPI_Data[1] = 0xD0; /* update DAC from input register */
	DACSPI_Data[0] = 0x00;

#if !defined (SPIDAC_OPTIMIZED_TRANSMIT)
	HAL_GPIO_WritePin(GPIOA, DAC_SYNC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, DACSPI_Data, 1, 1000);
	HAL_GPIO_WritePin(GPIOA, DAC_SYNC_Pin, GPIO_PIN_SET);
#else
	GPIOA->BRR = DAC_SYNC_Pin;
	GPIOA->BRR = DAC_SYNC_Pin;
	//HAL_SPI_Transmit(&hspi1, DACSPI_Data, 1, 1000);
	SPI_1LINE_TX(&hspi1);
	__HAL_SPI_ENABLE(&hspi1);
	hspi1.Instance->DR = *((uint16_t *)DACSPI_Data);

	while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY)!= RESET) {}

	__HAL_SPI_CLEAR_OVRFLAG(&hspi1);
	GPIOA->BSRR = DAC_SYNC_Pin;
#endif /* SPIDAC_OPTIMIZED_TRANSMIT */
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
