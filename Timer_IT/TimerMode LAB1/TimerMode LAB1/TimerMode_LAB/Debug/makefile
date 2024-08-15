################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
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
TimerMode_LAB.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
TimerMode_LAB.list \

OBJCOPY_BIN += \
TimerMode_LAB.bin \


# All Target
all: TimerMode_LAB.elf secondary-outputs

# Tool invocations
TimerMode_LAB.elf: $(OBJS) $(USER_OBJS) C:\Users\Khaled\ Magdy\Desktop\TimerMode_LAB\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "TimerMode_LAB.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\Khaled Magdy\Desktop\TimerMode_LAB\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="TimerMode_LAB.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

TimerMode_LAB.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "TimerMode_LAB.list"
	@echo 'Finished building: $@'
	@echo ' '

TimerMode_LAB.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "TimerMode_LAB.bin"
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
