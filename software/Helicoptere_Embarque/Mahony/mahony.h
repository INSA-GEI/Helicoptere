/**
 ******************************************************************************
 * @file           : mahony.h
 * @brief          : <Description to come here>
 * @author         : dimercur
 * @date           : 10 déc. 2019
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

#ifndef MAHONY_H_
#define MAHONY_H_

#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
typedef enum
{
	AHRS_STOP=0,
	AHRS_RUN=1,
	AHRS_ERROR=2
} ahrsStatus_t;

extern ahrsStatus_t AHRS_Status;

void MAHONY_Init(float sampleFreq);
void MAHONY_UpdateWithMag(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MAHONY_UpdateWithoutMag(float gx, float gy, float gz, float ax, float ay, float az);
void MAHONY_ComputeAngles();
float MAHONY_GetRoll();
float MAHONY_GetPitch();
float MAHONY_GetYaw();

#endif /* MAHONY_H_ */
