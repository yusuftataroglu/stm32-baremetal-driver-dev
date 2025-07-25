################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Experiments/007_SPI_Loopback_RX/main.c 

OBJS += \
./Experiments/007_SPI_Loopback_RX/main.o 

C_DEPS += \
./Experiments/007_SPI_Loopback_RX/main.d 


# Each subdirectory must supply rules for building sources it contributes
Experiments/007_SPI_Loopback_RX/%.o Experiments/007_SPI_Loopback_RX/%.su Experiments/007_SPI_Loopback_RX/%.cyclo: ../Experiments/007_SPI_Loopback_RX/%.c Experiments/007_SPI_Loopback_RX/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Inc -I"C:/Users/yusuf/OneDrive/Belgeler/STM32_Projects/stm32-baremetal-driver-dev/stm32-baremetal-lab/Drivers/CustomDrivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Experiments-2f-007_SPI_Loopback_RX

clean-Experiments-2f-007_SPI_Loopback_RX:
	-$(RM) ./Experiments/007_SPI_Loopback_RX/main.cyclo ./Experiments/007_SPI_Loopback_RX/main.d ./Experiments/007_SPI_Loopback_RX/main.o ./Experiments/007_SPI_Loopback_RX/main.su

.PHONY: clean-Experiments-2f-007_SPI_Loopback_RX

