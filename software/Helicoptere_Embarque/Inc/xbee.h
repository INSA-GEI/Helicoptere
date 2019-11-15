/**
 ******************************************************************************
 * @file           : xbee.h
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

#ifndef XBEE_H_
#define XBEE_H_

#include "stm32l4xx_hal.h"

typedef void (*XBEE_ReceptionCallbackTypeDef)(char* data, uint16_t size);

void XBEE_Init(void);
void XBEE_MspInit(void);
void XBEE_MspDeInit(void);

void XBEE_AddReceptionCallback (XBEE_ReceptionCallbackTypeDef callback);
void XBEE_SendData (char* data, uint16_t size);
void XBEE_StartReception(void);
void XBEE_StopReception(void);

#endif /* XBEE_H_ */
