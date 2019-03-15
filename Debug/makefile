################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include src/subdir.mk
-include Utilities/STM32F429I-Discovery/subdir.mk
-include Utilities/Log/subdir.mk
-include Utilities/Components/wm8994/subdir.mk
-include Utilities/Components/ts3510/subdir.mk
-include Utilities/Components/stmpe811/subdir.mk
-include Utilities/Components/stmpe1600/subdir.mk
-include Utilities/Components/st7789h2/subdir.mk
-include Utilities/Components/st7735/subdir.mk
-include Utilities/Components/s5k5cag/subdir.mk
-include Utilities/Components/ov2640/subdir.mk
-include Utilities/Components/otm8009a/subdir.mk
-include Utilities/Components/mfxstm32l152/subdir.mk
-include Utilities/Components/lsm303dlhc/subdir.mk
-include Utilities/Components/ls016b8uy/subdir.mk
-include Utilities/Components/lis3dsh/subdir.mk
-include Utilities/Components/lis302dl/subdir.mk
-include Utilities/Components/l3gd20/subdir.mk
-include Utilities/Components/ili9341/subdir.mk
-include Utilities/Components/ili9325/subdir.mk
-include Utilities/Components/ft6x06/subdir.mk
-include Utilities/Components/exc7200/subdir.mk
-include Utilities/Components/cs43l22/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: filter_bank_RTOS.elf

# Tool invocations
filter_bank_RTOS.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU GCC Linker'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -T"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/LinkerScript.ld" -Wl,-Map=output.map -Wl,--gc-sections -lm -o "filter_bank_RTOS.elf" @"objects.list" $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	-@echo 'Generating binary and Printing size information:'
	arm-none-eabi-objcopy -O binary "filter_bank_RTOS.elf" "filter_bank_RTOS.bin"
	arm-none-eabi-size "filter_bank_RTOS.elf"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
