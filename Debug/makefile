################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/delay/subdir.mk
-include Core/W25QXX/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Core/MPU9250/MPL/mllite/subdir.mk
-include Core/MPU9250/MPL/eMPL-hal/subdir.mk
-include Core/MPU9250/MPL/driver/stm32L/subdir.mk
-include Core/MPU9250/MPL/driver/eMPL/subdir.mk
-include Core/MPU9250/subdir.mk
-include Core/IIC/subdir.mk
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

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := NineAxisController
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
NineAxisController.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
NineAxisController.list \

OBJCOPY_BIN += \
NineAxisController.bin \


# All Target
all: main-build

# Main-build Target
main-build: NineAxisController.elf secondary-outputs

# Tool invocations
NineAxisController.elf: $(OBJS) $(USER_OBJS) D:\STM32CubeIDEWS\NineAxisController\STM32F103RCTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "NineAxisController.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\STM32CubeIDEWS\NineAxisController\STM32F103RCTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="NineAxisController.map" -Wl,--gc-sections -static -L"D:\STM32CubeIDEWS\NineAxisController\Core\MPU9250\MPL\mpl" --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

NineAxisController.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "NineAxisController.list"
	@echo 'Finished building: $@'
	@echo ' '

NineAxisController.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "NineAxisController.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) NineAxisController.bin NineAxisController.elf NineAxisController.list default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
