################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CustomDrivers/Src/core_nvic.c \
../Drivers/CustomDrivers/Src/stm32f103xx_gpio_driver.c 

OBJS += \
./Drivers/CustomDrivers/Src/core_nvic.o \
./Drivers/CustomDrivers/Src/stm32f103xx_gpio_driver.o 

C_DEPS += \
./Drivers/CustomDrivers/Src/core_nvic.d \
./Drivers/CustomDrivers/Src/stm32f103xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CustomDrivers/Src/%.o Drivers/CustomDrivers/Src/%.su Drivers/CustomDrivers/Src/%.cyclo: ../Drivers/CustomDrivers/Src/%.c Drivers/CustomDrivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Inc -I"C:/Users/yusuf/OneDrive/Belgeler/STM32_Projects/stm32-baremetal-driver-dev/stm32-baremetal-lab/Drivers/CustomDrivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-CustomDrivers-2f-Src

clean-Drivers-2f-CustomDrivers-2f-Src:
	-$(RM) ./Drivers/CustomDrivers/Src/core_nvic.cyclo ./Drivers/CustomDrivers/Src/core_nvic.d ./Drivers/CustomDrivers/Src/core_nvic.o ./Drivers/CustomDrivers/Src/core_nvic.su ./Drivers/CustomDrivers/Src/stm32f103xx_gpio_driver.cyclo ./Drivers/CustomDrivers/Src/stm32f103xx_gpio_driver.d ./Drivers/CustomDrivers/Src/stm32f103xx_gpio_driver.o ./Drivers/CustomDrivers/Src/stm32f103xx_gpio_driver.su

.PHONY: clean-Drivers-2f-CustomDrivers-2f-Src

