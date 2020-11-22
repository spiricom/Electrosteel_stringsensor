################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../leaf/leaf/Externals/d_fft_mayer.c 

OBJS += \
./leaf/leaf/Externals/d_fft_mayer.o 

C_DEPS += \
./leaf/leaf/Externals/d_fft_mayer.d 


# Each subdirectory must supply rules for building sources it contributes
leaf/leaf/Externals/d_fft_mayer.o: ../leaf/leaf/Externals/d_fft_mayer.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DSTM32H750xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"/Users/josnyder/dev/Electrosteel_stringsensor/electrosteel_string_H750ZB/leaf/leaf" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"leaf/leaf/Externals/d_fft_mayer.d" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

