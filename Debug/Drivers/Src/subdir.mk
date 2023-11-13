################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/STM32F407xx_GPIO_driver.c \
../Drivers/Src/STM32F407xx_I2C_driver.c \
../Drivers/Src/STM32F407xx_SPI_driver.c 

OBJS += \
./Drivers/Src/STM32F407xx_GPIO_driver.o \
./Drivers/Src/STM32F407xx_I2C_driver.o \
./Drivers/Src/STM32F407xx_SPI_driver.o 

C_DEPS += \
./Drivers/Src/STM32F407xx_GPIO_driver.d \
./Drivers/Src/STM32F407xx_I2C_driver.d \
./Drivers/Src/STM32F407xx_SPI_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"C:/Users/Lenovo/Desktop/MCU DRIVERS/STM32F4xx_Drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/STM32F407xx_GPIO_driver.cyclo ./Drivers/Src/STM32F407xx_GPIO_driver.d ./Drivers/Src/STM32F407xx_GPIO_driver.o ./Drivers/Src/STM32F407xx_GPIO_driver.su ./Drivers/Src/STM32F407xx_I2C_driver.cyclo ./Drivers/Src/STM32F407xx_I2C_driver.d ./Drivers/Src/STM32F407xx_I2C_driver.o ./Drivers/Src/STM32F407xx_I2C_driver.su ./Drivers/Src/STM32F407xx_SPI_driver.cyclo ./Drivers/Src/STM32F407xx_SPI_driver.d ./Drivers/Src/STM32F407xx_SPI_driver.o ./Drivers/Src/STM32F407xx_SPI_driver.su

.PHONY: clean-Drivers-2f-Src

