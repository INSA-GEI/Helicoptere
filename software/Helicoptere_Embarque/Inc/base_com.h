/**
 ******************************************************************************
 * @file           : base_com.h
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

#ifndef BASE_COM_H_
#define BASE_COM_H_

#include "stm32l4xx_hal.h"

typedef void (*BASECOM_ReceptionCallbackTypeDef)(char* data, uint16_t size);

void BASECOM_Init(void);
void BASECOM_MspInit(void);
void BASECOM_MspDeInit(void);

void BASECOM_AddReceptionCallback (BASECOM_ReceptionCallbackTypeDef callback);
void BASECOM_SendData (char* data, uint16_t size);
void BASECOM_StartReception(void);
void BASECOM_StopReception(void);

#endif /* BASE_COM_H_ */
