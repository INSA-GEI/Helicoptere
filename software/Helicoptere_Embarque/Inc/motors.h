/**
 ******************************************************************************
 * @file           : motors.h
 * @brief          : Motors program header
 * @author         : dimercur
 * @date           : Nov 14, 2019
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

#ifndef MOTORS_H_
#define MOTORS_H_

#include "stm32l4xx_hal.h"

#define MOTORS_CMD_MAX_RANGE	1000

void MOTORS_Init();
void MOTORS_MspInit();
void MOTORS_MspDeInit();

void MOTORS_SetHead(uint32_t speed);
void MOTORS_SetTail(uint32_t speed);

#endif /* MOTORS_H_ */
