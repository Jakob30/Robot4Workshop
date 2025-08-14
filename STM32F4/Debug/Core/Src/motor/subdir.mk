################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/motor/motor_control.c \
../Core/Src/motor/motor_init.c \
../Core/Src/motor/status_check.c 

C_DEPS += \
./Core/Src/motor/motor_control.d \
./Core/Src/motor/motor_init.d \
./Core/Src/motor/status_check.d 

OBJS += \
./Core/Src/motor/motor_control.o \
./Core/Src/motor/motor_init.o \
./Core/Src/motor/status_check.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/motor/%.o Core/Src/motor/%.su Core/Src/motor/%.cyclo: ../Core/Src/motor/%.c Core/Src/motor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-motor

clean-Core-2f-Src-2f-motor:
	-$(RM) ./Core/Src/motor/motor_control.cyclo ./Core/Src/motor/motor_control.d ./Core/Src/motor/motor_control.o ./Core/Src/motor/motor_control.su ./Core/Src/motor/motor_init.cyclo ./Core/Src/motor/motor_init.d ./Core/Src/motor/motor_init.o ./Core/Src/motor/motor_init.su ./Core/Src/motor/status_check.cyclo ./Core/Src/motor/status_check.d ./Core/Src/motor/status_check.o ./Core/Src/motor/status_check.su

.PHONY: clean-Core-2f-Src-2f-motor

