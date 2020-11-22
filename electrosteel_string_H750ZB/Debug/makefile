################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include leaf/leaf/Src/subdir.mk
-include leaf/leaf/Externals/subdir.mk
-include leaf/Examples/subdir.mk
-include Drivers/STM32H7xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
electrosteel_string_H750ZB.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
electrosteel_string_H750ZB.list \

OBJCOPY_BIN += \
electrosteel_string_H750ZB.bin \


# All Target
all: electrosteel_string_H750ZB.elf secondary-outputs

# Tool invocations
electrosteel_string_H750ZB.elf: $(OBJS) $(USER_OBJS) /Users/josnyder/dev/Electrosteel_stringsensor/electrosteel_string_H750ZB/STM32H750ZBTX_FLASH.ld
	arm-none-eabi-gcc -o "electrosteel_string_H750ZB.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m7 -T"/Users/josnyder/dev/Electrosteel_stringsensor/electrosteel_string_H750ZB/STM32H750ZBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="electrosteel_string_H750ZB.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

electrosteel_string_H750ZB.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "electrosteel_string_H750ZB.list"
	@echo 'Finished building: $@'
	@echo ' '

electrosteel_string_H750ZB.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "electrosteel_string_H750ZB.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
