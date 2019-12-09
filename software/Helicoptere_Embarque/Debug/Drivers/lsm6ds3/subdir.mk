################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/lsm6ds3/lsm6ds3_reg.c 

OBJS += \
./Drivers/lsm6ds3/lsm6ds3_reg.o 

C_DEPS += \
./Drivers/lsm6ds3/lsm6ds3_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/lsm6ds3/lsm6ds3_reg.o: ../Drivers/lsm6ds3/lsm6ds3_reg.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L432xx -c -I../Inc -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/AHRS" -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Drivers/lsm6ds3" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/lsm6ds3/lsm6ds3_reg.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

