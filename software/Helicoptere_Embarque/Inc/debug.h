/**
 ******************************************************************************
 * File Name          : debug.h
 * Description        : This file provides code for the configuration
 *                      of the debug services.
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
#ifndef __DEBUG_H
#define __DEBUG_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define DEBUG_SECTION_1_PORT	GPIOB
#define DEBUG_SECTION_2_PORT	GPIOA
#define DEBUG_SECTION_1_PIN		GPIO_PIN_0
#define DEBUG_SECTION_2_PIN		GPIO_PIN_12
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void DEBUG_Init(void);

/* USER CODE BEGIN Prototypes */
#define DEBUG_ENTERSECTION(section) section##_PORT->BSRR = section##_PIN
#define DEBUG_LEAVESECTION(section) section##_PORT->BRR = section##_PIN

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ DEBUG_H */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
