################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Apps/Debug.c \
../Apps/RobotDog.c \
../Apps/SystemInit.c \
../Apps/SystemTask.c 

OBJS += \
./Apps/Debug.o \
./Apps/RobotDog.o \
./Apps/SystemInit.o \
./Apps/SystemTask.o 

C_DEPS += \
./Apps/Debug.d \
./Apps/RobotDog.d \
./Apps/SystemInit.d \
./Apps/SystemTask.d 


# Each subdirectory must supply rules for building sources it contributes
Apps/%.o: ../Apps/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -std=c99 -I"D:\Documents\Me\2016¹úÈü\K70\Includes" -Wall -g2 -gstabs -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega128 -DF_CPU=16000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


