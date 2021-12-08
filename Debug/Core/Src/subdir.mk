################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/PID.c \
../Core/Src/contact.c \
../Core/Src/encoder.c \
../Core/Src/gpio.c \
../Core/Src/main.c \
../Core/Src/odometry.c \
../Core/Src/spi.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c 

OBJS += \
./Core/Src/PID.o \
./Core/Src/contact.o \
./Core/Src/encoder.o \
./Core/Src/gpio.o \
./Core/Src/main.o \
./Core/Src/odometry.o \
./Core/Src/spi.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o 

C_DEPS += \
./Core/Src/PID.d \
./Core/Src/contact.d \
./Core/Src/encoder.d \
./Core/Src/gpio.d \
./Core/Src/main.d \
./Core/Src/odometry.d \
./Core/Src/spi.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_DMP -DUSE_HAL_DRIVER -DSTM32F103xE -DEMPL -DEMPL_TARGET_STM32F4 -DMPU9250 -DMPL_LOG_NDEBUG=1 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"D:/STM32CubeIDEWS/NineAxisController/Core/delay" -I"D:/STM32CubeIDEWS/NineAxisController/Core/IIC" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/driver/include" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/mpl" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/mllite" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/eMPL-hal" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/driver" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/driver/eMPL" -I"D:/STM32CubeIDEWS/NineAxisController/Core/MPU9250/MPL/driver/stm32L" -I"D:/STM32CubeIDEWS/NineAxisController/Core/W25QXX" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/PID.d ./Core/Src/PID.o ./Core/Src/contact.d ./Core/Src/contact.o ./Core/Src/encoder.d ./Core/Src/encoder.o ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/odometry.d ./Core/Src/odometry.o ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/usart.d ./Core/Src/usart.o

.PHONY: clean-Core-2f-Src

