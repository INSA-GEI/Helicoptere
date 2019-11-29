/**
 ******************************************************************************
 * File Name          : USART.c
 * Description        : This file provides code for the configuration
 *                      of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
UART_ReceptionCallbackTypeDef UART_ReceptionCallback = 0;
extern void Error_Handler(void);
static void UART_RxISR(UART_HandleTypeDef *huart);
static void UART_ClearBuffer(char *buf);
static void UART_CopyBuffer(uint32_t size);

static char UART_RawBuffer[100];
static char UART_FilteredBuffer[100];
static uint32_t UART_RawBufferIndex;

void UART_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 250000;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
}

void UART_MspInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* USER CODE BEGIN USART2_MspInit 0 */

	/* USER CODE END USART2_MspInit 0 */
	/* USART2 clock enable */
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**USART2 GPIO Configuration
	    PA2     ------> USART2_TX
	    PA3     ------> USART2_RX
	 */
	GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USART2 DMA Init */
	/* USART2_TX Init */
	hdma_usart2_tx.Instance = DMA1_Channel4;
	hdma_usart2_tx.Init.Request = DMA_REQUEST_4;
	hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_tx.Init.Mode = DMA_NORMAL;
	hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
	if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_LINKDMA(&huart2,hdmatx,hdma_usart2_tx);

	/* USART2 interrupt Init */
	HAL_NVIC_SetPriority(USART2_IRQn, 0x04, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

	/* USER CODE BEGIN USART2_MspInit 1 */
	/* DMA1_Channel4_5_6_7_IRQn interrupt configuration */

	HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0x05, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
	/* USER CODE END USART2_MspInit 1 */

}

void UART_MspDeInit(void)
{
	__HAL_RCC_USART2_CLK_DISABLE();

	/**USART2 GPIO Configuration
	    PA2     ------> USART2_TX
	    PA3     ------> USART2_RX
	 */
	HAL_GPIO_DeInit(GPIOA, USART_TX_Pin|USART_RX_Pin);

	/* USART2 DMA DeInit */
	HAL_DMA_DeInit(huart2.hdmatx);

	/* USART2 interrupt Deinit */
	HAL_NVIC_DisableIRQ(USART2_IRQn);
}

void UART_ClearBuffer(char *buf)
{
	int i;

	for (i=0; i<100; i++)
	{
		buf[i]=0;
	}
}

static void UART_CopyBuffer(uint32_t size)
{
	int i;

	for (i=0; i<size+1; i++)
	{
		UART_FilteredBuffer[i]=UART_RawBuffer[i];
	}
}

void UART_AddReceptionCallback (UART_ReceptionCallbackTypeDef callback)
{
	if (callback==0)
	{
		Error_Handler();
	}

	UART_ReceptionCallback = callback;
}

void UART_SendData (char* data, uint16_t size)
{
	if (HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data, size) != HAL_OK)
	{
		Error_Handler();
	}
}

void UART_StartReception(void)
{
	if (UART_ReceptionCallback==0)
	{
		Error_Handler();
	}

	UART_ClearBuffer(UART_RawBuffer);
	UART_RawBufferIndex=0;
	huart2.RxISR = UART_RxISR;

	/* enable RX flag */
	/* Clear RXNE interrupt flag */
	__HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
	SET_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE);
}

void UART_StopReception(void)
{
	huart2.RxISR = 0;

	/* Clear RXNE interrupt flag */
	__HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
	CLEAR_BIT(huart2.Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
	CLEAR_BIT(huart2.Instance->CR3, USART_CR3_EIE);
}

static void UART_RxISR(UART_HandleTypeDef *huart)
{
	uint16_t  uhdata;

	uhdata = (uint16_t) READ_REG(huart->Instance->RDR);
	/* Clear RXNE interrupt flag */
	__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
	uhdata = (uint8_t)uhdata;

	if (uhdata!=0x0D)
	{
		UART_RawBuffer[UART_RawBufferIndex]=(char)uhdata;
		UART_RawBufferIndex++;
	}
	else
	{
		UART_RawBuffer[UART_RawBufferIndex]=0;
		UART_RawBufferIndex++;

		UART_CopyBuffer(UART_RawBufferIndex);
		UART_ReceptionCallback(UART_FilteredBuffer, UART_RawBufferIndex);

		UART_ClearBuffer(UART_RawBuffer);
		UART_RawBufferIndex=0;
	}
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
