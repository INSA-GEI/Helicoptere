################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/acc_gyro.c \
../Src/base_com.c \
../Src/debug.c \
../Src/led.c \
../Src/main.c \
../Src/motors.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32l4xx.c \
../Src/xbee.c 

OBJS += \
./Src/acc_gyro.o \
./Src/base_com.o \
./Src/debug.o \
./Src/led.o \
./Src/main.o \
./Src/motors.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32l4xx.o \
./Src/xbee.o 

C_DEPS += \
./Src/acc_gyro.d \
./Src/base_com.d \
./Src/debug.d \
./Src/led.d \
./Src/main.d \
./Src/motors.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32l4xx.d \
./Src/xbee.d 


# Each subdirectory must supply rules for building sources it contributes
Src/acc_gyro.o: ../Src/acc_gyro.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L432xx -c -I../Inc -I../Drivers/CMSIS/Include -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Mahony" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Drivers/lsm6ds3" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/acc_gyro.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/base_com.o: ../Src/base_com.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L432xx -c -I../Inc -I../Drivers/CMSIS/Include -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Mahony" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Drivers/lsm6ds3" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/base_com.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/debug.o: ../Src/debug.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L432xx -c -I../Inc -I../Drivers/CMSIS/Include -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Mahony" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Drivers/lsm6ds3" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/debug.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/led.o: ../Src/led.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L432xx -c -I../Inc -I../Drivers/CMSIS/Include -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Mahony" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Drivers/lsm6ds3" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/led.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/main.o: ../Src/main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L432xx -c -I../Inc -I../Drivers/CMSIS/Include -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Mahony" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Drivers/lsm6ds3" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/motors.o: ../Src/motors.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L432xx -c -I../Inc -I../Drivers/CMSIS/Include -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Mahony" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Drivers/lsm6ds3" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/motors.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/stm32l4xx_hal_msp.o: ../Src/stm32l4xx_hal_msp.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L432xx -c -I../Inc -I../Drivers/CMSIS/Include -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Mahony" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Drivers/lsm6ds3" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/stm32l4xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/stm32l4xx_it.o: ../Src/stm32l4xx_it.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L432xx -c -I../Inc -I../Drivers/CMSIS/Include -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Mahony" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Drivers/lsm6ds3" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/stm32l4xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/syscalls.o: ../Src/syscalls.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L432xx -c -I../Inc -I../Drivers/CMSIS/Include -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Mahony" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Drivers/lsm6ds3" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/syscalls.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L432xx -c -I../Inc -I../Drivers/CMSIS/Include -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Mahony" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Drivers/lsm6ds3" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/system_stm32l4xx.o: ../Src/system_stm32l4xx.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L432xx -c -I../Inc -I../Drivers/CMSIS/Include -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Mahony" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Drivers/lsm6ds3" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/system_stm32l4xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/xbee.o: ../Src/xbee.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DDEBUG -DSTM32L432xx -c -I../Inc -I../Drivers/CMSIS/Include -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Mahony" -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/Drivers/lsm6ds3" -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/xbee.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

