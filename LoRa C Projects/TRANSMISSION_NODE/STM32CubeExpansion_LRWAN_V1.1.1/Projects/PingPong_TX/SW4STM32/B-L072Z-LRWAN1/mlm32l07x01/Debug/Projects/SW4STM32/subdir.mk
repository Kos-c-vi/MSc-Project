################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Projects/Multi/Applications/LoRa/PingPong/SW4STM32/B-L072Z-LRWAN1/startup_stm32l072xx.s 

OBJS += \
./Projects/SW4STM32/startup_stm32l072xx.o 


# Each subdirectory must supply rules for building sources it contributes
Projects/SW4STM32/startup_stm32l072xx.o: C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Projects/Multi/Applications/LoRa/PingPong/SW4STM32/B-L072Z-LRWAN1/startup_stm32l072xx.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


