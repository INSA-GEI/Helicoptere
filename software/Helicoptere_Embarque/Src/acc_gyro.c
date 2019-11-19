/**
 ******************************************************************************
 * @file    stm32746g_discovery_acc_gyro.c
 * @author  dimercur
 * @brief   This file includes a standard driver for lsm6ds3 sensor
  @verbatim
  ==============================================================================
                     ##### How to use this driver #####
  ==============================================================================  
  [..] 
   (#) This driver is used to drive the N25Q128A QSPI external
       memory mounted on STM32746G-Discovery board.

   (#) This driver need a specific component driver (N25Q128A) to be included with.

   (#) Initialization steps:
       (++) Initialize the QPSI external memory using the BSP_QSPI_Init() function. This 
            function includes the MSP layer hardware resources initialization and the
            QSPI interface with the external memory.

   (#) QSPI memory operations
       (++) QSPI memory can be accessed with read/write operations once it is
            initialized.
            Read/write operation can be performed with AHB access using the functions
            BSP_QSPI_Read()/BSP_QSPI_Write(). 
       (++) The function BSP_QSPI_GetInfo() returns the configuration of the QSPI memory. 
            (see the QSPI memory data sheet)
       (++) Perform erase block operation using the function BSP_QSPI_Erase_Block() and by
            specifying the block address. You can perform an erase operation of the whole 
            chip by calling the function BSP_QSPI_Erase_Chip(). 
       (++) The function BSP_QSPI_GetStatus() returns the current status of the QSPI memory. 
            (see the QSPI memory data sheet)
  @endverbatim
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

/* Dependencies
- stm32f7xx_hal_spi.c
- stm32f7xx_hal_gpio.c
- stm32f7xx_hal_cortex.c
- stm32f7xx_hal_rcc_ex.h
EndDependencies */

/* Includes ------------------------------------------------------------------*/
#include "acc_gyro.h"

/** @addtogroup BSP
 * @{
 */

/** @addtogroup STM32746G_DISCOVERY
 * @{
 */

/** @defgroup STM32746G_DISCOVERY_ACC_GYRO STM32746G-Discovery ACC_GYRO
 * @{
 */


/* Private variables ---------------------------------------------------------*/

/** @defgroup STM32746G_DISCOVERY_ACC_GYRO_Private_Variables STM32746G_DISCOVERY ACC_GYRO Private Variables
 * @{
 */
static I2C_HandleTypeDef hi2c1;
extern void Error_Handler(void);

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);

lsm6ds3_ctx_t sensorCtx;
static char accSensorEnabled=0;

static int32_t GyroCorrectionCoeffs[3] = {0};

#if LSM6DS3_ADDRESS_HIGH
#define LSM6DS3_I2C_ADD LSM6DS3_I2C_ADD_H
#else
#define LSM6DS3_I2C_ADD LSM6DS3_I2C_ADD_L
#endif /* LSM6DS3_ADDRESS_HIGH */

#define SENSOR_INT_Pin GPIO_PIN_1
#define SENSOR_INT_GPIO_Port GPIOA
#define SENSOR_INT_EXTI_IRQn EXTI1_IRQn
/**
 * @}
 */



/* Private functions ---------------------------------------------------------*/

/** @defgroup STM32746G_DISCOVERY_ACC_GYRO_Private_Functions STM32746G_DISCOVERY ACC_GYRO Private Functions
 * @{
 */


/**
 * @}
 */

/** @defgroup STM32746G_DISCOVERY_ACC_GYRO_Exported_Functions STM32746G_DISCOVERY ACC_GYRO Exported Functions
 * @{
 */

/**
 * @brief  Initializes interface for accelerometer sensor.
 * @retval Initialization status
 */
uint8_t ACC_GYRO_Init(void)
{ 
	uint8_t whoamI,rst;
//	lsm6ds3_int1_route_t int_1_reg;
//	axis3bit16_t data;

	hi2c1.Instance = I2C1;
//	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Call the DeInit function to reset the driver */
	if (HAL_I2C_DeInit(&hi2c1) != HAL_OK)
	{
		return ACC_ERROR;
	}

	hi2c1.Init.Timing = 0x10909CEC;
	//hi2c1.Init.Timing = 0x40912732;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}

	sensorCtx.write_reg = platform_write;
	sensorCtx.read_reg = platform_read;
	sensorCtx.handle = &hi2c1;

	/*
	 *  Check device ID
	 */
	lsm6ds3_device_id_get(&sensorCtx, &whoamI);
	if (whoamI != LSM6DS3_ID)
	{
		return ACC_ERROR;
	}

	/*
	 * Restore default configuration
	 */
	lsm6ds3_reset_set(&sensorCtx, PROPERTY_ENABLE);
	do {
		lsm6ds3_reset_get(&sensorCtx, &rst);
	} while (rst);

	/*
	 * Enable Block Data Update
	 */
	lsm6ds3_block_data_update_set(&sensorCtx, PROPERTY_ENABLE);

	/*
	 * Set full scale
	 */
	lsm6ds3_xl_full_scale_set(&sensorCtx, LSM6DS3_ACC_FULL_SCALE);
	lsm6ds3_gy_full_scale_set(&sensorCtx, LSM6DS3_GYRO_FULL_SCALE);

	/*
	 * Set Output Data Rate
	 */
	lsm6ds3_xl_data_rate_set(&sensorCtx, LSM6DS3_ACC_ODR);
	lsm6ds3_gy_data_rate_set(&sensorCtx, LSM6DS3_GYRO_ODR);

	/*
	 * Set Accelerometer and gyroscope in High performance mode
	 */
	lsm6ds3_xl_power_mode_set(&sensorCtx, LSM6DS3_XL_HIGH_PERFORMANCE);
	lsm6ds3_gy_power_mode_set(&sensorCtx, LSM6DS3_GY_HIGH_PERFORMANCE);

	accSensorEnabled = 1;

	/*
	 * Compute gyro bias
	 */
	GYRO_UpdateGyroBias();

	/*
	 * Enable interrupt generation on DRDY INT1 pin
	 */
//	lsm6ds3_pin_int1_route_get(&sensorCtx, &int_1_reg);
//	int_1_reg.int1_drdy_g = PROPERTY_ENABLE;
//	int_1_reg.int1_drdy_xl = PROPERTY_ENABLE;
//	lsm6ds3_pin_int1_route_set(&sensorCtx, &int_1_reg);
//
//	/* Configure GPIO pin : SENSOR_INT_Pin */
//	GPIO_InitStruct.Pin = SENSOR_INT_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(SENSOR_INT_GPIO_Port, &GPIO_InitStruct);
//
//	/* EXTI interrupt init*/
//	HAL_NVIC_SetPriority(EXTI1_IRQn, 0x09, 0);
//	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
//	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	/* Do some dummy reads to flush It */
//	lsm6ds3_acceleration_raw_get(&sensorCtx, data.u8bit);
//	lsm6ds3_angular_rate_raw_get(&sensorCtx, data.u8bit);

	return ACC_OK;
}

/**
 * @brief  De-Initializes the accelerometer sensor interface.
 * @retval De-init status
 */
uint8_t ACC_GYRO_DeInit(void)
{ 
	ACC_GYRO_MspDeInit();
	accSensorEnabled =0;

	return ACC_OK;
}

/**
 * @brief  Request ID of the chip.
 * @retval If ID is 105, return ACC_OK, ACC_ERROR otherwise
 */
uint8_t ACC_GYRO_CheckID(void)
{
	uint8_t whoamI;

	lsm6ds3_device_id_get(&sensorCtx, &whoamI);
	if (whoamI != LSM6DS3_ID)
	{
		return ACC_ERROR;
	}

	return ACC_OK;
}

uint8_t ACC_IsDataReady(void)
{
	uint8_t reg;

	lsm6ds3_xl_flag_data_ready_get(&sensorCtx, &reg);
	return reg;
}

uint8_t GYRO_IsDataReady(void)
{
	uint8_t reg;

	lsm6ds3_gy_flag_data_ready_get(&sensorCtx, &reg);
	return reg;
}

/**
 * @brief  Reads acceleration values
 * @retval Read status
 */
uint8_t ACC_ReadRawValues(axis3bit16_t *data_raw_acceleration)
{
	uint8_t status= ACC_OK;
	uint8_t reg;

	if (accSensorEnabled)
	{
		/* TODO: Supprimer apres test */
		__disable_irq(); // Set PRIMASK

		/*
		 * Read status register
		 */
		lsm6ds3_xl_flag_data_ready_get(&sensorCtx, &reg);

		if (reg)
		{
			/*
			 * Read accelerometer field data
			 */
			memset(data_raw_acceleration->u8bit, 0, 3 * sizeof(int16_t));
			lsm6ds3_acceleration_raw_get(&sensorCtx, data_raw_acceleration->u8bit);
		}
		else status = ACC_BUSY;

		/* TODO: Supprimer apres test */
		__enable_irq(); // Clear PRIMASK
	} else status = ACC_ERROR;

	return status;
}

/**
 * @brief  Reads acceleration values
 * @retval Read status
 */
uint8_t GYRO_ReadRawValues(axis3bit16_t *data_raw_angular_rate)
{
	uint8_t status= ACC_OK;
	uint8_t reg;

	if (accSensorEnabled)
	{
		/* TODO: Supprimer apres test */
		__disable_irq(); // Set PRIMASK

		lsm6ds3_gy_flag_data_ready_get(&sensorCtx, &reg);

		if (reg)
		{
			/*
			 * Read gyroscope field data
			 */
			memset(data_raw_angular_rate->u8bit, 0, 3 * sizeof(int16_t));
			lsm6ds3_angular_rate_raw_get(&sensorCtx, data_raw_angular_rate->u8bit);
		}
		else status = ACC_BUSY;

		/* TODO: Supprimer apres test */
		__enable_irq(); // Clear PRIMASK
	} else status = ACC_ERROR;

	return status;
}

/**
 * @brief  Reads acceleration values
 * @retval Read status
 */
uint8_t ACC_ReadValues(acceleration_t *acceleration)
{
	axis3bit16_t data_raw_acceleration;
	uint8_t status= ACC_OK;

	if (accSensorEnabled)
	{
		/* TODO: Supprimer apres test */
		__disable_irq(); // Set PRIMASK

		status = ACC_ReadRawValues(&data_raw_acceleration);

		if (status == ACC_OK)
		{
			if (LSM6DS3_ACC_FULL_SCALE == LSM6DS3_2g)
			{
				acceleration->x =
						lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[0]);
				acceleration->y =
						lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[1]);
				acceleration->z =
						lsm6ds3_from_fs2g_to_mg(data_raw_acceleration.i16bit[2]);
			}
			else if (LSM6DS3_ACC_FULL_SCALE == LSM6DS3_4g)
			{
				acceleration->x =
						lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[0]);
				acceleration->y =
						lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[1]);
				acceleration->z =
						lsm6ds3_from_fs4g_to_mg(data_raw_acceleration.i16bit[2]);
			}
			else if (LSM6DS3_ACC_FULL_SCALE == LSM6DS3_8g)
			{
				acceleration->x =
						lsm6ds3_from_fs8g_to_mg(data_raw_acceleration.i16bit[0]);
				acceleration->y =
						lsm6ds3_from_fs8g_to_mg(data_raw_acceleration.i16bit[1]);
				acceleration->z =
						lsm6ds3_from_fs8g_to_mg(data_raw_acceleration.i16bit[2]);
			}
			else /* LSM6DS3_ACC_FULL_SCALE == LSM6DS3_16g */
			{
				acceleration->x =
						lsm6ds3_from_fs16g_to_mg(data_raw_acceleration.i16bit[0]);
				acceleration->y =
						lsm6ds3_from_fs16g_to_mg(data_raw_acceleration.i16bit[1]);
				acceleration->z =
						lsm6ds3_from_fs16g_to_mg(data_raw_acceleration.i16bit[2]);
			}
		}

		/* TODO: Supprimer apres test */
		__enable_irq(); // Clear PRIMASK
	} else status = ACC_ERROR;

	return status;
}

/**
 * @brief  Reads acceleration values
 * @retval Read status
 */
uint8_t GYRO_ReadValues(angularRate_t *angular_rate)
{
	axis3bit16_t data_raw_angular_rate;
	uint8_t status= ACC_OK;

	if (accSensorEnabled)
	{
		/* TODO: Supprimer apres test */
		__disable_irq(); // Set PRIMASK

		status = GYRO_ReadRawValues(&data_raw_angular_rate);

		if (status == ACC_OK)
		{
			/*
			 * Read gyroscope field data
			 */
			if (LSM6DS3_GYRO_FULL_SCALE == LSM6DS3_125dps)
			{
				angular_rate->x =
						lsm6ds3_from_fs125dps_to_mdps(data_raw_angular_rate.i16bit[0]-GyroCorrectionCoeffs[0]);
				angular_rate->y =
						lsm6ds3_from_fs125dps_to_mdps(data_raw_angular_rate.i16bit[1]-GyroCorrectionCoeffs[1]);
				angular_rate->z =
						lsm6ds3_from_fs125dps_to_mdps(data_raw_angular_rate.i16bit[2]-GyroCorrectionCoeffs[2]);
			}
			else if (LSM6DS3_GYRO_FULL_SCALE == LSM6DS3_250dps)
			{
				angular_rate->x =
						lsm6ds3_from_fs250dps_to_mdps(data_raw_angular_rate.i16bit[0]-GyroCorrectionCoeffs[0]);
				angular_rate->y =
						lsm6ds3_from_fs250dps_to_mdps(data_raw_angular_rate.i16bit[1]-GyroCorrectionCoeffs[1]);
				angular_rate->z =
						lsm6ds3_from_fs250dps_to_mdps(data_raw_angular_rate.i16bit[2]-GyroCorrectionCoeffs[2]);
			}
			else if (LSM6DS3_GYRO_FULL_SCALE == LSM6DS3_500dps)
			{
				angular_rate->x =
						lsm6ds3_from_fs500dps_to_mdps(data_raw_angular_rate.i16bit[0]-GyroCorrectionCoeffs[0]);
				angular_rate->y =
						lsm6ds3_from_fs500dps_to_mdps(data_raw_angular_rate.i16bit[1]-GyroCorrectionCoeffs[1]);
				angular_rate->z =
						lsm6ds3_from_fs500dps_to_mdps(data_raw_angular_rate.i16bit[2]-GyroCorrectionCoeffs[2]);
			}
			else if (LSM6DS3_GYRO_FULL_SCALE == LSM6DS3_1000dps)
			{
				angular_rate->x =
						lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[0]-GyroCorrectionCoeffs[0]);
				angular_rate->y =
						lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[1]-GyroCorrectionCoeffs[1]);
				angular_rate->z =
						lsm6ds3_from_fs1000dps_to_mdps(data_raw_angular_rate.i16bit[2]-GyroCorrectionCoeffs[2]);
			}
			else /* (LSM6DS3_GYRO_FULL_SCALE == LSM6DS3_2000dps) */
			{
				angular_rate->x =
						lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]-GyroCorrectionCoeffs[0]);
				angular_rate->y =
						lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]-GyroCorrectionCoeffs[1]);
				angular_rate->z =
						lsm6ds3_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]-GyroCorrectionCoeffs[2]);
			}
		}

		/* TODO: Supprimer apres test */
		__enable_irq(); // Clear PRIMASK
	} else status = ACC_ERROR;

	return status;
}

/**
 * @brief  Compute gyroscopic bias
 * @retval Read status
 */
uint8_t GYRO_UpdateGyroBias(void)
{
	int i = 0;
	axis3bit16_t data_raw_angular_rate;
	int32_t TmpGyroCorrectionCoeffs[3] = {0};

	GyroCorrectionCoeffs[0]=0;
	GyroCorrectionCoeffs[1]=0;
	GyroCorrectionCoeffs[2]=0;

	HAL_Delay(1000);

	for (i = 0; i < 100; i++) {
		GYRO_ReadRawValues(&data_raw_angular_rate);

		TmpGyroCorrectionCoeffs[0] += data_raw_angular_rate.i16bit[0];
		TmpGyroCorrectionCoeffs[1] += data_raw_angular_rate.i16bit[1];
		TmpGyroCorrectionCoeffs[2] += data_raw_angular_rate.i16bit[2];

		HAL_Delay(10);
	}

	GyroCorrectionCoeffs[0] = TmpGyroCorrectionCoeffs[0]/100;
	GyroCorrectionCoeffs[1] = TmpGyroCorrectionCoeffs[1]/100;
	GyroCorrectionCoeffs[2] = TmpGyroCorrectionCoeffs[2]/100;

	HAL_Delay(100);
	return ACC_OK;
}
/**
 * @brief  Reads T° values in °C
 * @retval Read status
 */
uint8_t ACC_ReadTemperature(float *temperature_degC)
{
	uint8_t status= ACC_OK;
	uint8_t reg;
	axis1bit16_t data_raw_temperature;

	if (accSensorEnabled)
	{
		/* TODO: Supprimer apres test */
		__disable_irq(); // Set PRIMASK

		/*
		 * Read output only if new value is available
		 */
		lsm6ds3_temp_flag_data_ready_get(&sensorCtx, &reg);

		if (reg)
		{
			/*
			 * Read temperature data
			 */
			memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
			lsm6ds3_temperature_raw_get(&sensorCtx, data_raw_temperature.u8bit);
			*temperature_degC = lsm6ds3_from_lsb_to_celsius(data_raw_temperature.i16bit);

		}
		else
		{
			status =  ACC_NO_DATA;
		}

		/* TODO: Supprimer apres test */
		__enable_irq(); // Clear PRIMASK
	} else status = ACC_ERROR;

	return status;
}

/**
 * @}
 */

/** @addtogroup ACC_GYRO_Private_Functions
 * @{
 */


/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len)
{

	HAL_I2C_Mem_Write(handle, LSM6DS3_I2C_ADD, reg,
			I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);


	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len)
{

	HAL_I2C_Mem_Read(handle, LSM6DS3_I2C_ADD, reg,
			I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);


	return 0;
}

/**
 * @brief ACC Gyro MSP Initialization
 *        This function configures the hardware resources used in this example:
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration
 *           - NVIC configuration for QSPI interrupt
 * @retval None
 */
void ACC_GYRO_MspInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**I2C1 GPIO Configuration
	    PB6     ------> I2C1_SCL
	    PB7     ------> I2C1_SDA
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Peripheral clock enable */
	__HAL_RCC_I2C1_CLK_ENABLE();
	__HAL_RCC_I2C1_FORCE_RESET();
	__HAL_RCC_I2C1_RELEASE_RESET();

	/* Enable and set I2Cx Interrupt to a lower priority */
	HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0x0A, 0);
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

	/* Enable and set I2Cx Interrupt to a lower priority */
	HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0x0B, 0);
	HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

/**
 * @brief ACC GYRO MSP De-Initialization
 *        This function frees the hardware resources used in this example:
 *          - Disable the Peripheral's clock
 *          - Revert GPIO and NVIC configuration to their default state
 * @retval None
 */
void ACC_GYRO_MspDeInit(void)
{
	/*##-1- Disable NVIC for IT_LIS2MDL ###########################################*/
	//	HAL_NVIC_DisableIRQ(SPI2_IRQn);
	__HAL_RCC_I2C1_FORCE_RESET();
	__HAL_RCC_I2C1_RELEASE_RESET();

	/*##-2- Disable peripherals and GPIO Clocks ################################*/
	/* De-Configure QSPI pins */
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

	__HAL_RCC_I2C1_CLK_DISABLE();
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

