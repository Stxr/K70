################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/DynamixelProtocol.c \
../Lib/Mapping.c \
../Lib/Queue.c \
../Lib/UpRobotProtocol.c 

OBJS += \
./Lib/DynamixelProtocol.o \
./Lib/Mapping.o \
./Lib/Queue.o \
./Lib/UpRobotProtocol.o 

C_DEPS += \
./Lib/DynamixelProtocol.d \
./Lib/Mapping.d \
./Lib/Queue.d \
./Lib/UpRobotProtocol.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/%.o: ../Lib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -std=c99 -I"D:\Documents\Me\2016¹úÈü\K70\Includes" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega128 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


