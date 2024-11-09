################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/stm32f407xx_gpio_driver.c \
../drivers/stm32f407xx_i2c_driver.c \
../drivers/stm32f407xx_rcc_driver.c \
../drivers/stm32f407xx_spi_driver.c 

OBJS += \
./drivers/stm32f407xx_gpio_driver.o \
./drivers/stm32f407xx_i2c_driver.o \
./drivers/stm32f407xx_rcc_driver.o \
./drivers/stm32f407xx_spi_driver.o 

C_DEPS += \
./drivers/stm32f407xx_gpio_driver.d \
./drivers/stm32f407xx_i2c_driver.d \
./drivers/stm32f407xx_rcc_driver.d \
./drivers/stm32f407xx_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/%.o drivers/%.su drivers/%.cyclo: ../drivers/%.c drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"/home/athul-muralidhar/stm32f4x-sensors-test/accelerometer/drivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-drivers

clean-drivers:
	-$(RM) ./drivers/stm32f407xx_gpio_driver.cyclo ./drivers/stm32f407xx_gpio_driver.d ./drivers/stm32f407xx_gpio_driver.o ./drivers/stm32f407xx_gpio_driver.su ./drivers/stm32f407xx_i2c_driver.cyclo ./drivers/stm32f407xx_i2c_driver.d ./drivers/stm32f407xx_i2c_driver.o ./drivers/stm32f407xx_i2c_driver.su ./drivers/stm32f407xx_rcc_driver.cyclo ./drivers/stm32f407xx_rcc_driver.d ./drivers/stm32f407xx_rcc_driver.o ./drivers/stm32f407xx_rcc_driver.su ./drivers/stm32f407xx_spi_driver.cyclo ./drivers/stm32f407xx_spi_driver.d ./drivers/stm32f407xx_spi_driver.o ./drivers/stm32f407xx_spi_driver.su

.PHONY: clean-drivers

