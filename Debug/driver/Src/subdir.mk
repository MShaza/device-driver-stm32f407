################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../driver/Src/stm32f407xx_gpio_driver.c \
../driver/Src/stm32f407xx_spi_driver.c 

OBJS += \
./driver/Src/stm32f407xx_gpio_driver.o \
./driver/Src/stm32f407xx_spi_driver.o 

C_DEPS += \
./driver/Src/stm32f407xx_gpio_driver.d \
./driver/Src/stm32f407xx_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
driver/Src/%.o driver/Src/%.su driver/Src/%.cyclo: ../driver/Src/%.c driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"/Users/muhammadshazab/Documents/Workspace/Projects/device-driver-stm32f407/driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-driver-2f-Src

clean-driver-2f-Src:
	-$(RM) ./driver/Src/stm32f407xx_gpio_driver.cyclo ./driver/Src/stm32f407xx_gpio_driver.d ./driver/Src/stm32f407xx_gpio_driver.o ./driver/Src/stm32f407xx_gpio_driver.su ./driver/Src/stm32f407xx_spi_driver.cyclo ./driver/Src/stm32f407xx_spi_driver.d ./driver/Src/stm32f407xx_spi_driver.o ./driver/Src/stm32f407xx_spi_driver.su

.PHONY: clean-driver-2f-Src

