################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/drivers.c \
../src/filter.c \
../src/main.c \
../src/routing.c \
../src/stm32f4xx_it.c \
../src/syscalls.c \
../src/system_stm32f4xx.c 

OBJS += \
./src/drivers.o \
./src/filter.o \
./src/main.o \
./src/routing.o \
./src/stm32f4xx_it.o \
./src/syscalls.o \
./src/system_stm32f4xx.o 

C_DEPS += \
./src/drivers.d \
./src/filter.d \
./src/main.d \
./src/routing.d \
./src/stm32f4xx_it.d \
./src/syscalls.d \
./src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32F429ZITx -DSTM32 -DSTM32F429I_DISC1 -DSTM32F4 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F429xx -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/inc" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/CMSIS/core" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/CMSIS/device" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/HAL_Driver/Inc/Legacy" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/HAL_Driver/Inc" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Middlewares/Third_Party/FreeRTOS/Source/include" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/ampire480272" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/ampire640480" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/Common" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/cs43l22" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/exc7200" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/ft6x06" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/ili9325" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/ili9341" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/l3gd20" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/lis302dl" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/lis3dsh" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/ls016b8uy" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/lsm303dlhc" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/mfxstm32l152" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/n25q128a" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/n25q256a" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/n25q512a" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/otm8009a" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/ov2640" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/s25fl512s" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/s5k5cag" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/st7735" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/st7789h2" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/stmpe1600" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/stmpe811" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/ts3510" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Components/wm8994" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Fonts" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/Log" -I"/Users/Myren/Documents/STM32/STM32workspace/filter_bank_RTOS/Utilities/STM32F429I-Discovery" -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


