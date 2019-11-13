/**
 ******************************************************************************
 * @file           : led.h
 * @brief          : Led program body
 * @author         : dimercur
 * @date           : Nov 13, 2019
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

#ifndef LED_H_
#define LED_H_

#include "stm32l4xx_hal.h"

#define LED_MODE_IDLE 0
#define LED_MODE_RUN 1
#define LED_MODE_ERROR 2

void LED_Init(void);
void LED_SetMode(int mode);

#endif /* LED_H_ */
