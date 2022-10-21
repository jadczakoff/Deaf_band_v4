################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/tinyUSB/class/cdc/cdc_device.c \
../Core/tinyUSB/class/cdc/cdc_host.c \
../Core/tinyUSB/class/cdc/cdc_rndis_host.c 

OBJS += \
./Core/tinyUSB/class/cdc/cdc_device.o \
./Core/tinyUSB/class/cdc/cdc_host.o \
./Core/tinyUSB/class/cdc/cdc_rndis_host.o 

C_DEPS += \
./Core/tinyUSB/class/cdc/cdc_device.d \
./Core/tinyUSB/class/cdc/cdc_host.d \
./Core/tinyUSB/class/cdc/cdc_rndis_host.d 


# Each subdirectory must supply rules for building sources it contributes
Core/tinyUSB/class/cdc/%.o Core/tinyUSB/class/cdc/%.su: ../Core/tinyUSB/class/cdc/%.c Core/tinyUSB/class/cdc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../PDM2PCM/App -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/tinyUSB -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-tinyUSB-2f-class-2f-cdc

clean-Core-2f-tinyUSB-2f-class-2f-cdc:
	-$(RM) ./Core/tinyUSB/class/cdc/cdc_device.d ./Core/tinyUSB/class/cdc/cdc_device.o ./Core/tinyUSB/class/cdc/cdc_device.su ./Core/tinyUSB/class/cdc/cdc_host.d ./Core/tinyUSB/class/cdc/cdc_host.o ./Core/tinyUSB/class/cdc/cdc_host.su ./Core/tinyUSB/class/cdc/cdc_rndis_host.d ./Core/tinyUSB/class/cdc/cdc_rndis_host.o ./Core/tinyUSB/class/cdc/cdc_rndis_host.su

.PHONY: clean-Core-2f-tinyUSB-2f-class-2f-cdc

