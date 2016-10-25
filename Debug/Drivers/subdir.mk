################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Adc.c \
../Drivers/ExtInt.c \
../Drivers/Gpio.c \
../Drivers/Isr.c \
../Drivers/Led.c \
../Drivers/Ppm.c \
../Drivers/Sim.c \
../Drivers/Spi.c \
../Drivers/Timer.c \
../Drivers/Uart.c \
../Drivers/VirtualUart.c 

OBJS += \
./Drivers/Adc.o \
./Drivers/ExtInt.o \
./Drivers/Gpio.o \
./Drivers/Isr.o \
./Drivers/Led.o \
./Drivers/Ppm.o \
./Drivers/Sim.o \
./Drivers/Spi.o \
./Drivers/Timer.o \
./Drivers/Uart.o \
./Drivers/VirtualUart.o 

C_DEPS += \
./Drivers/Adc.d \
./Drivers/ExtInt.d \
./Drivers/Gpio.d \
./Drivers/Isr.d \
./Drivers/Led.d \
./Drivers/Ppm.d \
./Drivers/Sim.d \
./Drivers/Spi.d \
./Drivers/Timer.d \
./Drivers/Uart.d \
./Drivers/VirtualUart.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/%.o: ../Drivers/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -std=c99 -I"D:\Documents\Me\2016¹úÈü\K70\Includes" -Wall -g2 -gstabs -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega128 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


