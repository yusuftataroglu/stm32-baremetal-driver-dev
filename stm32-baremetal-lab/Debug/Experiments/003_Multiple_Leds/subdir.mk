################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Experiments/003_Multiple_Leds/main.c 

OBJS += \
./Experiments/003_Multiple_Leds/main.o 

C_DEPS += \
./Experiments/003_Multiple_Leds/main.d 


# Each subdirectory must supply rules for building sources it contributes
Experiments/003_Multiple_Leds/%.o Experiments/003_Multiple_Leds/%.su Experiments/003_Multiple_Leds/%.cyclo: ../Experiments/003_Multiple_Leds/%.c Experiments/003_Multiple_Leds/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Inc -I"C:/Users/yusuf/OneDrive/Belgeler/STM32_Projects/stm32-baremetal-driver-dev/stm32-baremetal-lab/Drivers/CustomDrivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Experiments-2f-003_Multiple_Leds

clean-Experiments-2f-003_Multiple_Leds:
	-$(RM) ./Experiments/003_Multiple_Leds/main.cyclo ./Experiments/003_Multiple_Leds/main.d ./Experiments/003_Multiple_Leds/main.o ./Experiments/003_Multiple_Leds/main.su

.PHONY: clean-Experiments-2f-003_Multiple_Leds

