################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MS5837-30BA.c \
../Core/Src/MadgwickAHRS.c \
../Core/Src/Thruster.c \
../Core/Src/crc.c \
../Core/Src/imu9dof.c \
../Core/Src/l3gd20.c \
../Core/Src/lsm303dlhc.c \
../Core/Src/main.c \
../Core/Src/max7456.c \
../Core/Src/osdWidgets.c \
../Core/Src/stm32f3xx_hal_msp.c \
../Core/Src/stm32f3xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f3xx.c 

OBJS += \
./Core/Src/MS5837-30BA.o \
./Core/Src/MadgwickAHRS.o \
./Core/Src/Thruster.o \
./Core/Src/crc.o \
./Core/Src/imu9dof.o \
./Core/Src/l3gd20.o \
./Core/Src/lsm303dlhc.o \
./Core/Src/main.o \
./Core/Src/max7456.o \
./Core/Src/osdWidgets.o \
./Core/Src/stm32f3xx_hal_msp.o \
./Core/Src/stm32f3xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f3xx.o 

C_DEPS += \
./Core/Src/MS5837-30BA.d \
./Core/Src/MadgwickAHRS.d \
./Core/Src/Thruster.d \
./Core/Src/crc.d \
./Core/Src/imu9dof.d \
./Core/Src/l3gd20.d \
./Core/Src/lsm303dlhc.d \
./Core/Src/main.d \
./Core/Src/max7456.d \
./Core/Src/osdWidgets.d \
./Core/Src/stm32f3xx_hal_msp.d \
./Core/Src/stm32f3xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xC -c -I../Core/Inc -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I"C:/Users/Workstation/Desktop/SEVEROV_MC-master-2/Core" -I"C:/Users/Workstation/Desktop/SEVEROV_MC-master-2/Core/Src" -I"C:/Users/Workstation/Desktop/SEVEROV_MC-master-2/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/Workstation/Desktop/SEVEROV_MC-master-2/Drivers" -I"C:/Users/Workstation/Desktop/SEVEROV_MC-master-2" -I"C:/Users/Workstation/Desktop/SEVEROV_MC-master-2/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/Workstation/Desktop/SEVEROV_MC-master-2/Drivers/CMSIS/Include" -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/CMSIS/Include -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/MS5837-30BA.d ./Core/Src/MS5837-30BA.o ./Core/Src/MS5837-30BA.su ./Core/Src/MadgwickAHRS.d ./Core/Src/MadgwickAHRS.o ./Core/Src/MadgwickAHRS.su ./Core/Src/Thruster.d ./Core/Src/Thruster.o ./Core/Src/Thruster.su ./Core/Src/crc.d ./Core/Src/crc.o ./Core/Src/crc.su ./Core/Src/imu9dof.d ./Core/Src/imu9dof.o ./Core/Src/imu9dof.su ./Core/Src/l3gd20.d ./Core/Src/l3gd20.o ./Core/Src/l3gd20.su ./Core/Src/lsm303dlhc.d ./Core/Src/lsm303dlhc.o ./Core/Src/lsm303dlhc.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/max7456.d ./Core/Src/max7456.o ./Core/Src/max7456.su ./Core/Src/osdWidgets.d ./Core/Src/osdWidgets.o ./Core/Src/osdWidgets.su ./Core/Src/stm32f3xx_hal_msp.d ./Core/Src/stm32f3xx_hal_msp.o ./Core/Src/stm32f3xx_hal_msp.su ./Core/Src/stm32f3xx_it.d ./Core/Src/stm32f3xx_it.o ./Core/Src/stm32f3xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f3xx.d ./Core/Src/system_stm32f3xx.o ./Core/Src/system_stm32f3xx.su

.PHONY: clean-Core-2f-Src

