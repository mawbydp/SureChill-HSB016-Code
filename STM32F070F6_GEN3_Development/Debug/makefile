################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
STM32F070F6_GEN3_Development.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32F070F6_GEN3_Development.list \


# All Target
all: STM32F070F6_GEN3_Development.elf secondary-outputs

# Tool invocations
STM32F070F6_GEN3_Development.elf: $(OBJS) $(USER_OBJS) C:\Users\davidmawby\STM32CubeIDE\workspace_1.1.0\STM32F070F6_GEN3_Development\STM32F070F6PX_FLASH.ld
	arm-none-eabi-gcc -o "STM32F070F6_GEN3_Development.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"C:\Users\davidmawby\STM32CubeIDE\workspace_1.1.0\STM32F070F6_GEN3_Development\STM32F070F6PX_FLASH.ld" --specs=nosys.specs -Wl,-Map="STM32F070F6_GEN3_Development.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32F070F6_GEN3_Development.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32F070F6_GEN3_Development.list"
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
