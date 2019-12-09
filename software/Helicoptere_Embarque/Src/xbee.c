/**
 ******************************************************************************
 * @file           : xbee.c
 * @brief          : Communication through xbee or Bluetooth
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

#include "xbee.h"

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
XBEE_ReceptionCallbackTypeDef XBEE_ReceptionCallback = 0;
extern void Error_Handler(void);
static void XBEE_RxISR(UART_HandleTypeDef *huart);
static void XBEE_ClearBuffer(char *buf);
static void XBEE_CopyBuffer(uint32_t size);

static char XBEE_RawBuffer[100];
static char XBEE_FilteredBuffer[100];
static uint32_t XBEE_RawBufferIndex;

void XBEE_Init(void)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}

	XBEE_ClearBuffer(XBEE_RawBuffer);
	XBEE_ClearBuffer(XBEE_FilteredBuffer);
	XBEE_RawBufferIndex=0;
}

void XBEE_MspInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**USART1 GPIO Configuration
	    PA9     ------> USART1_TX
	    PA10     ------> USART1_RX
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USART1 DMA Init */
	/* USART1_TX Init */
	hdma_usart1_tx.Instance = DMA1_Channel4;
	hdma_usart1_tx.Init.Request = DMA_REQUEST_2;
	hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart1_tx.Init.Mode = DMA_NORMAL;
	hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
	if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_LINKDMA(&huart1,hdmatx,hdma_usart1_tx);

	/* USART1 interrupt Init */
	HAL_NVIC_SetPriority(USART1_IRQn, 0x0B, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);

	/* DMA1_Channel4 interrupt init */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0x03, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

void XBEE_MspDeInit(void)
{
	__HAL_RCC_USART1_CLK_DISABLE();

	/**USART1 GPIO Configuration
	    PA9     ------> USART1_TX
	    PA10     ------> USART1_RX
	 */
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

	/* USART1 DMA DeInit */
	HAL_DMA_DeInit(huart1.hdmatx);

	/* USART1 interrupt DeInit */
	HAL_NVIC_DisableIRQ(USART1_IRQn);
}

void XBEE_ClearBuffer(char *buf)
{
	int i;

	for (i=0; i<100; i++)
	{
		buf[i]=0;
	}
}

static void XBEE_CopyBuffer(uint32_t size)
{
	int i;

	for (i=0; i<size+1; i++)
	{
		XBEE_FilteredBuffer[i]=XBEE_RawBuffer[i];
	}
}

void XBEE_AddReceptionCallback (XBEE_ReceptionCallbackTypeDef callback)
{
	if (callback==0)
	{
		Error_Handler();
	}

	XBEE_ReceptionCallback = callback;
}

void XBEE_SendData (char* data, uint16_t size)
{
	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t*)data, size) != HAL_OK)
	{
		Error_Handler();
	}
}


void XBEE_StartReception(void)
{
	if (XBEE_ReceptionCallback==0)
	{
		Error_Handler();
	}

	XBEE_ClearBuffer(XBEE_RawBuffer);
	XBEE_RawBufferIndex=0;
	huart1.RxISR = XBEE_RxISR;

	/* enable RX flag */
	/* Clear RXNE interrupt flag */
	__HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
	SET_BIT(huart1.Instance->CR1, USART_CR1_RXNEIE);
}

void XBEE_StopReception(void)
{
	huart1.RxISR = 0;

	/* Clear RXNE interrupt flag */
	__HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
	CLEAR_BIT(huart1.Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
	CLEAR_BIT(huart1.Instance->CR3, USART_CR3_EIE);
}

static void XBEE_RxISR(UART_HandleTypeDef *huart)
{
	uint16_t  uhdata;

	uhdata = (uint16_t) READ_REG(huart->Instance->RDR);
	/* Clear RXNE interrupt flag */
	__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
	uhdata = (uint8_t)uhdata;


	if ((uhdata!='\r')&&(uhdata!='\n'))
	{
		XBEE_RawBuffer[XBEE_RawBufferIndex]=(char)uhdata;
		XBEE_RawBufferIndex++;
	}
	else if (uhdata!='\n')
	{
		XBEE_RawBuffer[XBEE_RawBufferIndex]=0;
		XBEE_RawBufferIndex++;

		XBEE_CopyBuffer(XBEE_RawBufferIndex);
		XBEE_ReceptionCallback(XBEE_FilteredBuffer, XBEE_RawBufferIndex);

		XBEE_ClearBuffer(XBEE_RawBuffer);
		XBEE_RawBufferIndex=0;
	}
}

