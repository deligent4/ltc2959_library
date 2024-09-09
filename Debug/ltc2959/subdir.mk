################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ltc2959/ltc2959.c 

OBJS += \
./ltc2959/ltc2959.o 

C_DEPS += \
./ltc2959/ltc2959.d 


# Each subdirectory must supply rules for building sources it contributes
ltc2959/%.o ltc2959/%.su ltc2959/%.cyclo: ../ltc2959/%.c ltc2959/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"E:/STM32CubeIDE/ltc2959_lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ltc2959

clean-ltc2959:
	-$(RM) ./ltc2959/ltc2959.cyclo ./ltc2959/ltc2959.d ./ltc2959/ltc2959.o ./ltc2959/ltc2959.su

.PHONY: clean-ltc2959

