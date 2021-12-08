################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/delay/delay.c 

OBJS += \
./Core/delay/delay.o 

C_DEPS += \
./Core/delay/delay.d 


# Each subdirectory must supply rules for building sources it contributes
Core/delay/%.o: ../Core/delay/%.c Core/delay/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_DMP -DUSE_HAL_DRIVER -DSTM32F103xE -DEMPL -DEMPL_TARGET_STM32F4 -DMPU9250 -DMPL_LOG_NDEBUG=1 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/STM32CubeIDEWS/NineAxisController/Core/delay" -I"D:/STM32CubeIDEWS/NineAxisController/Core/IIC" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/driver/include" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/mpl" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/mllite" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/eMPL-hal" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/driver" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/driver/eMPL" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/driver/stm32L" -I"D:/STM32CubeIDEWS/NineAxisController/Core/W25QXX" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-delay

clean-Core-2f-delay:
	-$(RM) ./Core/delay/delay.d ./Core/delay/delay.o

.PHONY: clean-Core-2f-delay

