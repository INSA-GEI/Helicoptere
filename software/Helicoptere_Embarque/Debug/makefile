################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/lsm6ds3/subdir.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include AHRS/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Helicoptere_Embarque_FW.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Helicoptere_Embarque_FW.list \


# All Target
all: Helicoptere_Embarque_FW.elf secondary-outputs

# Tool invocations
Helicoptere_Embarque_FW.elf: $(OBJS) $(USER_OBJS) /home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/STM32L432KCUX_FLASH.ld
	arm-none-eabi-gcc -o "Helicoptere_Embarque_FW.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/dimercur/Documents/Travail/git/Helicoptere/software/Helicoptere_Embarque/STM32L432KCUX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Helicoptere_Embarque_FW.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Helicoptere_Embarque_FW.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Helicoptere_Embarque_FW.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
