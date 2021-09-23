################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/IMA/Git/WAKeMeUP-H755/H755_ETH/Middlewares/ringbuff/src/ringbuff/ringbuff.c 

OBJS += \
./Middlewares/ringbuff/src/ringbuff/ringbuff.o 

C_DEPS += \
./Middlewares/ringbuff/src/ringbuff/ringbuff.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ringbuff/src/ringbuff/ringbuff.o: C:/IMA/Git/WAKeMeUP-H755/H755_ETH/Middlewares/ringbuff/src/ringbuff/ringbuff.c Middlewares/ringbuff/src/ringbuff/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DCORE_CM4 -DDEBUG -DSTM32H755xx -c -I../Core/Inc -I../../Middlewares/ringbuff/src/ringbuff -I../../Middlewares/ringbuff/src/include -I../../Drivers/STM32H7xx_HAL_Driver/Inc -I../../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../../Drivers/CMSIS/Include -I../../Common/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Middlewares/ringbuff/src/ringbuff/ringbuff.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

