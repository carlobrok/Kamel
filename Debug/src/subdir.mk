################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables
CPP_SRCS += \
../src/2020_Hannover.cpp \
../src/CameraCapture.cpp \
../src/KamelDevices.cpp \
../src/Logger.cpp \
../src/VideoServer.cpp \
../src/gruen.cpp \
../src/line.cpp \
../src/util.cpp \
../src/display.cpp

OBJS += \
./src/2020_Hannover.o \
./src/CameraCapture.o \
./src/KamelDevices.o \
./src/Logger.o \
./src/VideoServer.o \
./src/gruen.o \
./src/line.o \
./src/util.o \
./src/display.o

CPP_DEPS += \
./src/2020_Hannover.d \
./src/CameraCapture.d \
./src/KamelDevices.d \
./src/Logger.d \
./src/VideoServer.d \
./src/gruen.d \
./src/line.d \
./src/util.d \
./src/display.d


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++14 -O3 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
