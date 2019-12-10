/**
 ******************************************************************************
 * File Name          : USART.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

typedef void (*UART_ReceptionCallbackTypeDef)(char* data, uint16_t size);

void UART_Init(void);
void UART_DeInit(void);
void UART_MspInit(void);
void UART_MspDeInit(void);

void UART_AddReceptionCallback (UART_ReceptionCallbackTypeDef callback);
void UART_SendData (char* data, uint16_t size);
void UART_StartReception(void);
void UART_StopReception(void);

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
