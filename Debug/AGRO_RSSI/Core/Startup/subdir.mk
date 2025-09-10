################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../AGRO_RSSI/Core/Startup/startup_stm32f103c8tx.s 

OBJS += \
./AGRO_RSSI/Core/Startup/startup_stm32f103c8tx.o 

S_DEPS += \
./AGRO_RSSI/Core/Startup/startup_stm32f103c8tx.d 


# Each subdirectory must supply rules for building sources it contributes
AGRO_RSSI/Core/Startup/%.o: ../AGRO_RSSI/Core/Startup/%.s AGRO_RSSI/Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

clean: clean-AGRO_RSSI-2f-Core-2f-Startup

clean-AGRO_RSSI-2f-Core-2f-Startup:
	-$(RM) ./AGRO_RSSI/Core/Startup/startup_stm32f103c8tx.d ./AGRO_RSSI/Core/Startup/startup_stm32f103c8tx.o

.PHONY: clean-AGRO_RSSI-2f-Core-2f-Startup

