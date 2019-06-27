################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Fatfs/SD.c \
../Middlewares/Fatfs/diskio.c \
../Middlewares/Fatfs/ff.c \
../Middlewares/Fatfs/ffsystem.c \
../Middlewares/Fatfs/ffunicode.c 

OBJS += \
./Middlewares/Fatfs/SD.o \
./Middlewares/Fatfs/diskio.o \
./Middlewares/Fatfs/ff.o \
./Middlewares/Fatfs/ffsystem.o \
./Middlewares/Fatfs/ffunicode.o 

C_DEPS += \
./Middlewares/Fatfs/SD.d \
./Middlewares/Fatfs/diskio.d \
./Middlewares/Fatfs/ff.d \
./Middlewares/Fatfs/ffsystem.d \
./Middlewares/Fatfs/ffunicode.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Fatfs/%.o: ../Middlewares/Fatfs/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -DSTM32L072xx -DUSE_B_L072Z_LRWAN1 -DUSE_HAL_DRIVER -DUSE_BAND_868 -DUSE_MODEM_LORA -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Projects/Multi/Applications/LoRa/PingPong/inc" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/BSP/B-L072Z-LRWAN1" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/BSP/MLM32L07X01" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/STM32L0xx_HAL_Driver/Inc" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/CMSIS/Device/ST/STM32L0xx/Include" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/CMSIS/Include" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Middlewares/Third_Party/Lora/Crypto" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Middlewares/Third_Party/Lora/Mac" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Middlewares/Third_Party/Lora/Phy" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Middlewares/Third_Party/Lora/Utilities" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/BSP/X_NUCLEO_IKS01A1" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/BSP/Components/Common" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/BSP/Components/hts221" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/BSP/Components/lps25hb" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Drivers/BSP/Components/sx1276" -I"C:/Users/user/STM32Cube/Repository/STM32CubeExpansion_LRWAN_V1.1.1/Middlewares/Third_Party/Lora/Core" -I"C:/Users/user/Downloads/ff13/source"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


