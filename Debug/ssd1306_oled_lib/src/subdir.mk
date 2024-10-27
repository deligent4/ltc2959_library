################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ssd1306_oled_lib/src/ssd1306.c \
../ssd1306_oled_lib/src/ssd1306_fonts.c \
../ssd1306_oled_lib/src/ssd1306_tests.c 

OBJS += \
./ssd1306_oled_lib/src/ssd1306.o \
./ssd1306_oled_lib/src/ssd1306_fonts.o \
./ssd1306_oled_lib/src/ssd1306_tests.o 

C_DEPS += \
./ssd1306_oled_lib/src/ssd1306.d \
./ssd1306_oled_lib/src/ssd1306_fonts.d \
./ssd1306_oled_lib/src/ssd1306_tests.d 


# Each subdirectory must supply rules for building sources it contributes
ssd1306_oled_lib/src/%.o ssd1306_oled_lib/src/%.su ssd1306_oled_lib/src/%.cyclo: ../ssd1306_oled_lib/src/%.c ssd1306_oled_lib/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"E:/STM32CubeIDE/ltc2959_lib/ssd1306_oled_lib/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ssd1306_oled_lib-2f-src

clean-ssd1306_oled_lib-2f-src:
	-$(RM) ./ssd1306_oled_lib/src/ssd1306.cyclo ./ssd1306_oled_lib/src/ssd1306.d ./ssd1306_oled_lib/src/ssd1306.o ./ssd1306_oled_lib/src/ssd1306.su ./ssd1306_oled_lib/src/ssd1306_fonts.cyclo ./ssd1306_oled_lib/src/ssd1306_fonts.d ./ssd1306_oled_lib/src/ssd1306_fonts.o ./ssd1306_oled_lib/src/ssd1306_fonts.su ./ssd1306_oled_lib/src/ssd1306_tests.cyclo ./ssd1306_oled_lib/src/ssd1306_tests.d ./ssd1306_oled_lib/src/ssd1306_tests.o ./ssd1306_oled_lib/src/ssd1306_tests.su

.PHONY: clean-ssd1306_oled_lib-2f-src

