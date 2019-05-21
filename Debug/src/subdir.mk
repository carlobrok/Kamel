################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/2020_Hannover.cpp \
../src/gruen.cpp \
../src/line.cpp 

OBJS += \
./src/2020_Hannover.o \
./src/gruen.o \
./src/line.o 

CPP_DEPS += \
./src/2020_Hannover.d \
./src/gruen.d \
./src/line.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


