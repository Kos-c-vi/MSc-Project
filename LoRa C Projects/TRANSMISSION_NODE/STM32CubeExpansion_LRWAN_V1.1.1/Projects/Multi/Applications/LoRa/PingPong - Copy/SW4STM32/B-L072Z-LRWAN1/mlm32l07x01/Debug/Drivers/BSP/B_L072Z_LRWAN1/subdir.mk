################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/BSP/B-L072Z-LRWAN1/b-l072z-lrwan1.c 

OBJS += \
./Drivers/BSP/B_L072Z_LRWAN1/b-l072z-lrwan1.o 

C_DEPS += \
./Drivers/BSP/B_L072Z_LRWAN1/b-l072z-lrwan1.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/B_L072Z_LRWAN1/b-l072z-lrwan1.o: C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/BSP/B-L072Z-LRWAN1/b-l072z-lrwan1.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -DSTM32L072xx -DUSE_B_L072Z_LRWAN1 -DUSE_HAL_DRIVER -DUSE_BAND_868 -DUSE_MODEM_LORA -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Projects/Multi/Applications/LoRa/PingPong/inc" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/BSP/B-L072Z-LRWAN1" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/BSP/MLM32L07X01" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/STM32L0xx_HAL_Driver/Inc" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/CMSIS/Device/ST/STM32L0xx/Include" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/CMSIS/Include" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Middlewares/Third_Party/Lora/Crypto" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Middlewares/Third_Party/Lora/Mac" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Middlewares/Third_Party/Lora/Phy" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Middlewares/Third_Party/Lora/Utilities" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/BSP/X_NUCLEO_IKS01A1" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/BSP/Components/Common" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/BSP/Components/hts221" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/BSP/Components/lps25hb" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/BSP/Components/sx1276" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Middlewares/Third_Party/Lora/Core"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


