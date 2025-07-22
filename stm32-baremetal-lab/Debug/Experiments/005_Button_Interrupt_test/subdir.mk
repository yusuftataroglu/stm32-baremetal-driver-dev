################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Experiments/005_Button_Interrupt_test/main.c 

OBJS += \
./Experiments/005_Button_Interrupt_test/main.o 

C_DEPS += \
./Experiments/005_Button_Interrupt_test/main.d 


# Each subdirectory must supply rules for building sources it contributes
Experiments/005_Button_Interrupt_test/%.o Experiments/005_Button_Interrupt_test/%.su Experiments/005_Button_Interrupt_test/%.cyclo: ../Experiments/005_Button_Interrupt_test/%.c Experiments/005_Button_Interrupt_test/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Inc -I"C:/Users/yusuf/OneDrive/Belgeler/STM32_Projects/stm32-baremetal-driver-dev/stm32-baremetal-lab/Drivers/CustomDrivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Experiments-2f-005_Button_Interrupt_test

clean-Experiments-2f-005_Button_Interrupt_test:
	-$(RM) ./Experiments/005_Button_Interrupt_test/main.cyclo ./Experiments/005_Button_Interrupt_test/main.d ./Experiments/005_Button_Interrupt_test/main.o ./Experiments/005_Button_Interrupt_test/main.su

.PHONY: clean-Experiments-2f-005_Button_Interrupt_test

