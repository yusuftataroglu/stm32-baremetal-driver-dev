################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Experiments/002_Button_Input/main.c 

OBJS += \
./Experiments/002_Button_Input/main.o 

C_DEPS += \
./Experiments/002_Button_Input/main.d 


# Each subdirectory must supply rules for building sources it contributes
Experiments/002_Button_Input/%.o Experiments/002_Button_Input/%.su Experiments/002_Button_Input/%.cyclo: ../Experiments/002_Button_Input/%.c Experiments/002_Button_Input/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -DNUCLEO_F103RB -c -I../Inc -I"C:/Users/yusuf/OneDrive/Belgeler/STM32_Projects/stm32-baremetal-driver-dev/stm32-baremetal-lab/Drivers/CustomDrivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Experiments-2f-002_Button_Input

clean-Experiments-2f-002_Button_Input:
	-$(RM) ./Experiments/002_Button_Input/main.cyclo ./Experiments/002_Button_Input/main.d ./Experiments/002_Button_Input/main.o ./Experiments/002_Button_Input/main.su

.PHONY: clean-Experiments-2f-002_Button_Input

