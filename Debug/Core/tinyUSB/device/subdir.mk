################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/tinyUSB/device/usbd.c \
../Core/tinyUSB/device/usbd_control.c 

OBJS += \
./Core/tinyUSB/device/usbd.o \
./Core/tinyUSB/device/usbd_control.o 

C_DEPS += \
./Core/tinyUSB/device/usbd.d \
./Core/tinyUSB/device/usbd_control.d 


# Each subdirectory must supply rules for building sources it contributes
Core/tinyUSB/device/%.o Core/tinyUSB/device/%.su: ../Core/tinyUSB/device/%.c Core/tinyUSB/device/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../PDM2PCM/App -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Core/tinyUSB -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-tinyUSB-2f-device

clean-Core-2f-tinyUSB-2f-device:
	-$(RM) ./Core/tinyUSB/device/usbd.d ./Core/tinyUSB/device/usbd.o ./Core/tinyUSB/device/usbd.su ./Core/tinyUSB/device/usbd_control.d ./Core/tinyUSB/device/usbd_control.o ./Core/tinyUSB/device/usbd_control.su

.PHONY: clean-Core-2f-tinyUSB-2f-device

