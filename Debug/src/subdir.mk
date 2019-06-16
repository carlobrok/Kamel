################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/2020_Hannover.cpp \
../src/CameraCapture.cpp \
../src/KamelI2C.cpp \
../src/VideoServer.cpp \
../src/gruen.cpp \
../src/line.cpp 

OBJS += \
./src/2020_Hannover.o \
./src/CameraCapture.o \
./src/KamelI2C.o \
./src/VideoServer.o \
./src/gruen.o \
./src/line.o 

CPP_DEPS += \
./src/2020_Hannover.d \
./src/CameraCapture.d \
./src/KamelI2C.d \
./src/VideoServer.d \
./src/gruen.d \
./src/line.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++0x -O3 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


