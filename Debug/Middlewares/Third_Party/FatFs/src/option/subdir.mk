################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/FatFs/src/option/syscall.c 

C_DEPS += \
./Middlewares/Third_Party/FatFs/src/option/syscall.d 

OBJS += \
./Middlewares/Third_Party/FatFs/src/option/syscall.o 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/FatFs/src/option/%.o: ../Middlewares/Third_Party/FatFs/src/option/%.c Middlewares/Third_Party/FatFs/src/option/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"C:/Users/Islom/STM32CubeIDE/cam/Cam/Core/Inc" -I"C:/Users/Islom/STM32CubeIDE/cam/Cam/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Islom/STM32CubeIDE/cam/Cam/Drivers/CMSIS/Include" -I"C:/Users/Islom/STM32CubeIDE/cam/Cam/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Islom/STM32CubeIDE/cam/Cam/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Islom/STM32CubeIDE/cam/Cam/FATFS/App" -I"C:/Users/Islom/STM32CubeIDE/cam/Cam/FATFS/Target" -I"C:/Users/Islom/STM32CubeIDE/cam/Cam/Middlewares/Third_Party/FatFs/src/option" -I"C:/Users/Islom/STM32CubeIDE/cam/Cam/Middlewares/Third_Party/FatFs/src" -I../FATFS/App -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../FATFS/Target -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

