################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Shell/Api.c \
../Shell/ClockUnit.c \
../Shell/CommandParser.c \
../Shell/ServoUnit.c \
../Shell/SignalUnit.c \
../Shell/SystemUnit.c \
../Shell/TtlBusUnit.c 

OBJS += \
./Shell/Api.o \
./Shell/ClockUnit.o \
./Shell/CommandParser.o \
./Shell/ServoUnit.o \
./Shell/SignalUnit.o \
./Shell/SystemUnit.o \
./Shell/TtlBusUnit.o 

C_DEPS += \
./Shell/Api.d \
./Shell/ClockUnit.d \
./Shell/CommandParser.d \
./Shell/ServoUnit.d \
./Shell/SignalUnit.d \
./Shell/SystemUnit.d \
./Shell/TtlBusUnit.d 


# Each subdirectory must supply rules for building sources it contributes
Shell/%.o: ../Shell/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -std=c99 -I"D:\Documents\Me\2016¹úÈü\K70\Includes" -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega128 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


