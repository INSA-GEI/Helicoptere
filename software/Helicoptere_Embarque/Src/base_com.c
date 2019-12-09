/**
 ******************************************************************************
 * @file           : base_com.c
 * @brief          : <Description to come here>
 * @author         : dimercur
 * @date           : 15 nov. 2019
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

#include "base_com.h"

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
BASECOM_ReceptionCallbackTypeDef BASECOM_ReceptionCallback = 0;
extern void Error_Handler(void);
static void BASECOM_RxISR(UART_HandleTypeDef *huart);
static void BASECOM_ClearBuffer(char *buf);
static void BASECOM_CopyBuffer(uint32_t size);

static char BASECOM_RawBuffer[100];
static char BASECOM_FilteredBuffer[100];
static uint32_t BASECOM_RawBufferIndex;

void BASECOM_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
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

void BASECOM_MspInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**USART2 GPIO Configuration
	    PA2     ------> USART2_TX
	    PA3     ------> USART2_RX
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USART2 DMA Init */
	/* USART2_TX Init */
	hdma_usart2_tx.Instance = DMA1_Channel7;
	hdma_usart2_tx.Init.Request = DMA_REQUEST_2;
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
	HAL_NVIC_SetPriority(USART2_IRQn, 0x0A, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

	/* DMA1_Channel7 interrupt init */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0x0F, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

void BASECOM_MspDeInit(void)
{
	__HAL_RCC_USART2_CLK_DISABLE();

	/**USART2 GPIO Configuration
	    PA2     ------> USART2_TX
	    PA3     ------> USART2_RX
	 */
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

	/* USART2 DMA DeInit */
	HAL_DMA_DeInit(huart2.hdmatx);

	/* USART2 interrupt DeInit */
	HAL_NVIC_DisableIRQ(USART2_IRQn);
}

void BASECOM_ClearBuffer(char *buf)
{
	int i;

	for (i=0; i<100; i++)
	{
		buf[i]=0;
	}
}

static void BASECOM_CopyBuffer(uint32_t size)
{
	int i;

	for (i=0; i<size+1; i++)
	{
		BASECOM_FilteredBuffer[i]=BASECOM_RawBuffer[i];
	}
}

void BASECOM_AddReceptionCallback (BASECOM_ReceptionCallbackTypeDef callback)
{
	if (callback==0)
	{
		Error_Handler();
	}

	BASECOM_ReceptionCallback = callback;
}

void BASECOM_SendData (char* data, uint16_t size)
{
	if (HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data, size) != HAL_OK)
	{
		Error_Handler();
	}
}

void BASECOM_StartReception(void)
{
	if (BASECOM_ReceptionCallback==0)
	{
		Error_Handler();
	}

	BASECOM_ClearBuffer(BASECOM_RawBuffer);
	BASECOM_RawBufferIndex=0;
	huart2.RxISR = BASECOM_RxISR;

	/* enable RX flag */
	/* Clear RXNE interrupt flag */
	__HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
	SET_BIT(huart2.Instance->CR1, USART_CR1_RXNEIE);
}

void BASECOM_StopReception(void)
{
	huart2.RxISR = 0;

	/* Clear RXNE interrupt flag */
	__HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
	CLEAR_BIT(huart2.Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
	CLEAR_BIT(huart2.Instance->CR3, USART_CR3_EIE);
}

static void BASECOM_RxISR(UART_HandleTypeDef *huart)
{
	uint16_t  uhdata;

	uhdata = (uint16_t) READ_REG(huart->Instance->RDR);
	/* Clear RXNE interrupt flag */
	__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
	uhdata = (uint8_t)uhdata;

	if ((uhdata!='\r')&&(uhdata!='\n'))
	{
		BASECOM_RawBuffer[BASECOM_RawBufferIndex]=(char)uhdata;
		BASECOM_RawBufferIndex++;
	}
	else if (uhdata!='\n')
	{
		BASECOM_RawBuffer[BASECOM_RawBufferIndex]=0;
		BASECOM_RawBufferIndex++;

		BASECOM_CopyBuffer(BASECOM_RawBufferIndex);
		BASECOM_ReceptionCallback(BASECOM_FilteredBuffer, BASECOM_RawBufferIndex);

		BASECOM_ClearBuffer(BASECOM_RawBuffer);
		BASECOM_RawBufferIndex=0;
	}
}

