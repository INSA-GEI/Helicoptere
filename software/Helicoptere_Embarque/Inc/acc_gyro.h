/**
 ******************************************************************************
 * @file    stm32746g_discovery_mag.h
 * @author  dimercur
 * @brief   This file contains the common defines and functions prototypes for
 *          the stm32746g_discovery_mag.c driver.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/** @addtogroup BSP
 * @{
 */

/** @addtogroup STM32746G_DISCOVERY
 * @{
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ACC_GYRO_H
#define __ACC_GYRO_H

#ifdef __cplusplus
extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "lsm6ds3_reg.h"
#include <string.h>
/** @addtogroup STM32746G_DISCOVERY_ACC_GYRO
 * @{
 */


/* Exported constants --------------------------------------------------------*/ 
/** @defgroup STM32746G_DISCOVERY_ACC_GYRO_Exported_Constants STM32746G_DISCOVERY_ACC_GYRO Exported Constants
 * @{
 */
/* ACCelerometre Error codes */
#define ACC_OK            ((uint8_t)0x00)
#define ACC_ERROR         ((uint8_t)0x01)
#define ACC_BUSY          ((uint8_t)0x02)
#define ACC_NO_DATA       ((uint8_t)0x03)

/* Driver settings */
// If LSM6DS3_ADDRESS_HIGH is set, LSM6DS3 I2C Address High will be used instead of Adress Low (depending off SDO state)
//#define LSM6DS3_ADDRESS_HIGH

// Acceleration full scale
// LSM6DS3_ACC_FULL_SCALE can take values off:
// * LSM6DS3_2g
// * LSM6DS3_4g
// * LSM6DS3_8g
// * LSM6DS3_16g

#define LSM6DS3_ACC_FULL_SCALE LSM6DS3_2g

// Acceleration Output Data Rate (update frequency)
// LSM6DS3_ACC_ODR can take values off:
// * LSM6DS3_XL_ODR_OFF
// * LSM6DS3_XL_ODR_12Hz5  = 1,
// * LSM6DS3_XL_ODR_26Hz   = 2,
// * LSM6DS3_XL_ODR_52Hz   = 3,
// * LSM6DS3_XL_ODR_104Hz  = 4,
// * LSM6DS3_XL_ODR_208Hz  = 5,
// * LSM6DS3_XL_ODR_416Hz  = 6,
// * LSM6DS3_XL_ODR_833Hz  = 7,
// * LSM6DS3_XL_ODR_1k66Hz = 8,
// * LSM6DS3_XL_ODR_3k33Hz = 9,
// * LSM6DS3_XL_ODR_6k66Hz = 10,

#define LSM6DS3_ACC_ODR LSM6DS3_XL_ODR_833Hz

// Gyroscope full scale
// LSM6DS3_GYRO_FULL_SCALE can take values off:
// * LSM6DS3_125dps
// * LSM6DS3_250dps
// * LSM6DS3_500dps
// * LSM6DS3_1000dps
// * LSM6DS3_2000dps

#define LSM6DS3_GYRO_FULL_SCALE LSM6DS3_2000dps

// Gyroscope Output Data Rate (update frequency)
// LSM6DS3_GYRO_ODR can take values off:
// * LSM6DS3_GY_ODR_OFF    = 0,
// * LSM6DS3_GY_ODR_12Hz5  = 1,
// * LSM6DS3_GY_ODR_26Hz   = 2,
// * LSM6DS3_GY_ODR_52Hz   = 3,
// * LSM6DS3_GY_ODR_104Hz  = 4,
// * LSM6DS3_GY_ODR_208Hz  = 5,
// * LSM6DS3_GY_ODR_416Hz  = 6,
// * LSM6DS3_GY_ODR_833Hz  = 7,
// * LSM6DS3_GY_ODR_1k66Hz = 8,

#define LSM6DS3_GYRO_ODR LSM6DS3_GY_ODR_833Hz
/**
 * @}
 */

/* Exported types ------------------------------------------------------------*/
/** @defgroup STM32746G_DISCOVERY_ACC_GYRO_Exported_Types STM32746G_DISCOVERY_ACC_GYRO Exported Types
 * @{
 */
typedef struct
{
	float x;
	float y;
	float z;
} acceleration_t ;

typedef struct
{
	float x;
	float y;
	float z;
} angularRate_t;
/**
 * @}
 */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup STM32746G_DISCOVERY_ACC_GYRO_Exported_Functions
 * @{
 */
uint8_t ACC_GYRO_Init       (void);
uint8_t ACC_GYRO_DeInit     (void);
uint8_t ACC_GYRO_CheckID	(void);
uint8_t ACC_IsDataReady		(void);
uint8_t GYRO_IsDataReady	(void);
uint8_t ACC_ReadRawValues	(axis3bit16_t *data_raw_acceleration);
uint8_t GYRO_ReadRawValues	(axis3bit16_t *data_raw_angular_rate);
uint8_t ACC_ReadValues		(acceleration_t *acceleration);
uint8_t GYRO_ReadValues		(angularRate_t *angular_rate);
uint8_t GYRO_UpdateGyroBias	(void);
uint8_t ACC_ReadTemperature	(float *temperature_degC);

/* These functions can be modified in case the current settings
   need to be changed for specific application needs */
void ACC_GYRO_MspInit		(void);
void ACC_GYRO_MspDeInit		(void);

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __ACC_GYRO_H */
/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
