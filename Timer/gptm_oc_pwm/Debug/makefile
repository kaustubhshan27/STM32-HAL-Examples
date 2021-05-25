################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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
gptm_oc_pwm.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
gptm_oc_pwm.list \

OBJCOPY_BIN += \
gptm_oc_pwm.bin \


# All Target
all: gptm_oc_pwm.elf secondary-outputs

# Tool invocations
gptm_oc_pwm.elf: $(OBJS) $(USER_OBJS) /home/kaustubh/STM32CubeIDE/workspace_1.3.0/gptm_oc_pwm/STM32F411RETX_FLASH.ld
	arm-none-eabi-gcc -o "gptm_oc_pwm.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/kaustubh/STM32CubeIDE/workspace_1.3.0/gptm_oc_pwm/STM32F411RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="gptm_oc_pwm.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

gptm_oc_pwm.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "gptm_oc_pwm.list"
	@echo 'Finished building: $@'
	@echo ' '

gptm_oc_pwm.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "gptm_oc_pwm.bin"
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
