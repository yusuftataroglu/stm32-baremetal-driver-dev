################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Experiments/004_Button_Interrupt/main.c 

OBJS += \
./Experiments/004_Button_Interrupt/main.o 

C_DEPS += \
./Experiments/004_Button_Interrupt/main.d 


# Each subdirectory must supply rules for building sources it contributes
Experiments/004_Button_Interrupt/%.o Experiments/004_Button_Interrupt/%.su Experiments/004_Button_Interrupt/%.cyclo: ../Experiments/004_Button_Interrupt/%.c Experiments/004_Button_Interrupt/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Inc -I"C:/Users/yusuf/OneDrive/Belgeler/STM32_Projects/stm32-baremetal-driver-dev/stm32-baremetal-lab/Drivers/CustomDrivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Experiments-2f-004_Button_Interrupt

clean-Experiments-2f-004_Button_Interrupt:
	-$(RM) ./Experiments/004_Button_Interrupt/main.cyclo ./Experiments/004_Button_Interrupt/main.d ./Experiments/004_Button_Interrupt/main.o ./Experiments/004_Button_Interrupt/main.su

.PHONY: clean-Experiments-2f-004_Button_Interrupt

