################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP2/l3gd20/l3gd20.c 

OBJS += \
./Drivers/BSP2/l3gd20/l3gd20.o 

C_DEPS += \
./Drivers/BSP2/l3gd20/l3gd20.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP2/l3gd20/%.o: ../Drivers/BSP2/l3gd20/%.c Drivers/BSP2/l3gd20/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I"C:/Users/Dimitrius/STM32CubeIDE/workspace_1.8.0/disco/Drivers/BSP2" -I"C:/Users/Dimitrius/STM32CubeIDE/workspace_1.8.0/disco/Drivers/BSP2" -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP2-2f-l3gd20

clean-Drivers-2f-BSP2-2f-l3gd20:
	-$(RM) ./Drivers/BSP2/l3gd20/l3gd20.d ./Drivers/BSP2/l3gd20/l3gd20.o

.PHONY: clean-Drivers-2f-BSP2-2f-l3gd20

