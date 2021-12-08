################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/MPU9250/MPL/mllite/data_builder.c \
../Core/MPU9250/MPL/mllite/hal_outputs.c \
../Core/MPU9250/MPL/mllite/message_layer.c \
../Core/MPU9250/MPL/mllite/ml_math_func.c \
../Core/MPU9250/MPL/mllite/mlmath.c \
../Core/MPU9250/MPL/mllite/mpl.c \
../Core/MPU9250/MPL/mllite/results_holder.c \
../Core/MPU9250/MPL/mllite/start_manager.c \
../Core/MPU9250/MPL/mllite/storage_manager.c 

OBJS += \
./Core/MPU9250/MPL/mllite/data_builder.o \
./Core/MPU9250/MPL/mllite/hal_outputs.o \
./Core/MPU9250/MPL/mllite/message_layer.o \
./Core/MPU9250/MPL/mllite/ml_math_func.o \
./Core/MPU9250/MPL/mllite/mlmath.o \
./Core/MPU9250/MPL/mllite/mpl.o \
./Core/MPU9250/MPL/mllite/results_holder.o \
./Core/MPU9250/MPL/mllite/start_manager.o \
./Core/MPU9250/MPL/mllite/storage_manager.o 

C_DEPS += \
./Core/MPU9250/MPL/mllite/data_builder.d \
./Core/MPU9250/MPL/mllite/hal_outputs.d \
./Core/MPU9250/MPL/mllite/message_layer.d \
./Core/MPU9250/MPL/mllite/ml_math_func.d \
./Core/MPU9250/MPL/mllite/mlmath.d \
./Core/MPU9250/MPL/mllite/mpl.d \
./Core/MPU9250/MPL/mllite/results_holder.d \
./Core/MPU9250/MPL/mllite/start_manager.d \
./Core/MPU9250/MPL/mllite/storage_manager.d 


# Each subdirectory must supply rules for building sources it contributes
Core/MPU9250/MPL/mllite/%.o: ../Core/MPU9250/MPL/mllite/%.c Core/MPU9250/MPL/mllite/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_DMP -DUSE_HAL_DRIVER -DSTM32F103xE -DEMPL -DEMPL_TARGET_STM32F4 -DMPU9250 -DMPL_LOG_NDEBUG=1 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/STM32CubeIDEWS/NineAxisController/Core/delay" -I"D:/STM32CubeIDEWS/NineAxisController/Core/IIC" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/driver/include" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/mpl" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/mllite" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/eMPL-hal" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/driver" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/driver/eMPL" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/driver/stm32L" -I"D:/STM32CubeIDEWS/NineAxisController/Core/W25QXX" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-MPU9250-2f-MPL-2f-mllite

clean-Core-2f-MPU9250-2f-MPL-2f-mllite:
	-$(RM) ./Core/MPU9250/MPL/mllite/data_builder.d ./Core/MPU9250/MPL/mllite/data_builder.o ./Core/MPU9250/MPL/mllite/hal_outputs.d ./Core/MPU9250/MPL/mllite/hal_outputs.o ./Core/MPU9250/MPL/mllite/message_layer.d ./Core/MPU9250/MPL/mllite/message_layer.o ./Core/MPU9250/MPL/mllite/ml_math_func.d ./Core/MPU9250/MPL/mllite/ml_math_func.o ./Core/MPU9250/MPL/mllite/mlmath.d ./Core/MPU9250/MPL/mllite/mlmath.o ./Core/MPU9250/MPL/mllite/mpl.d ./Core/MPU9250/MPL/mllite/mpl.o ./Core/MPU9250/MPL/mllite/results_holder.d ./Core/MPU9250/MPL/mllite/results_holder.o ./Core/MPU9250/MPL/mllite/start_manager.d ./Core/MPU9250/MPL/mllite/start_manager.o ./Core/MPU9250/MPL/mllite/storage_manager.d ./Core/MPU9250/MPL/mllite/storage_manager.o

.PHONY: clean-Core-2f-MPU9250-2f-MPL-2f-mllite

