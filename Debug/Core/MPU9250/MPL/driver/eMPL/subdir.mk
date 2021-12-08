################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/MPU9250/MPL/driver/eMPL/inv_mpu.c \
../Core/MPU9250/MPL/driver/eMPL/inv_mpu_dmp_motion_driver.c 

OBJS += \
./Core/MPU9250/MPL/driver/eMPL/inv_mpu.o \
./Core/MPU9250/MPL/driver/eMPL/inv_mpu_dmp_motion_driver.o 

C_DEPS += \
./Core/MPU9250/MPL/driver/eMPL/inv_mpu.d \
./Core/MPU9250/MPL/driver/eMPL/inv_mpu_dmp_motion_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/MPU9250/MPL/driver/eMPL/%.o: ../Core/MPU9250/MPL/driver/eMPL/%.c Core/MPU9250/MPL/driver/eMPL/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_DMP -DUSE_HAL_DRIVER -DSTM32F103xE -DEMPL -DEMPL_TARGET_STM32F4 -DMPU9250 -DMPL_LOG_NDEBUG=1 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/STM32CubeIDEWS/NineAxisController/Core/delay" -I"D:/STM32CubeIDEWS/NineAxisController/Core/IIC" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/driver/include" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/mpl" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/mllite" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/eMPL-hal" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/driver" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/driver/eMPL" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/driver/stm32L" -I"D:/STM32CubeIDEWS/NineAxisController/Core/W25QXX" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-MPU9250-2f-MPL-2f-driver-2f-eMPL

clean-Core-2f-MPU9250-2f-MPL-2f-driver-2f-eMPL:
	-$(RM) ./Core/MPU9250/MPL/driver/eMPL/inv_mpu.d ./Core/MPU9250/MPL/driver/eMPL/inv_mpu.o ./Core/MPU9250/MPL/driver/eMPL/inv_mpu_dmp_motion_driver.d ./Core/MPU9250/MPL/driver/eMPL/inv_mpu_dmp_motion_driver.o

.PHONY: clean-Core-2f-MPU9250-2f-MPL-2f-driver-2f-eMPL

