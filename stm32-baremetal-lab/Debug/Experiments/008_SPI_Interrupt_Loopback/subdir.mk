################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Experiments/008_SPI_Interrupt_Loopback/main.c 

OBJS += \
./Experiments/008_SPI_Interrupt_Loopback/main.o 

C_DEPS += \
./Experiments/008_SPI_Interrupt_Loopback/main.d 


# Each subdirectory must supply rules for building sources it contributes
Experiments/008_SPI_Interrupt_Loopback/%.o Experiments/008_SPI_Interrupt_Loopback/%.su Experiments/008_SPI_Interrupt_Loopback/%.cyclo: ../Experiments/008_SPI_Interrupt_Loopback/%.c Experiments/008_SPI_Interrupt_Loopback/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Inc -I"C:/Users/yusuf/OneDrive/Belgeler/STM32_Projects/stm32-baremetal-driver-dev/stm32-baremetal-lab/Drivers/CustomDrivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Experiments-2f-008_SPI_Interrupt_Loopback

clean-Experiments-2f-008_SPI_Interrupt_Loopback:
	-$(RM) ./Experiments/008_SPI_Interrupt_Loopback/main.cyclo ./Experiments/008_SPI_Interrupt_Loopback/main.d ./Experiments/008_SPI_Interrupt_Loopback/main.o ./Experiments/008_SPI_Interrupt_Loopback/main.su

.PHONY: clean-Experiments-2f-008_SPI_Interrupt_Loopback

