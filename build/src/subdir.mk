# Add inputs and outputs from these tool invocations to the build variables
CPP_SRCS += \
../src/2020_Hannover.cpp \
../src/CameraCapture.cpp \
../src/KamelDevices.cpp \
../src/Logger.cpp \
../src/VideoServer.cpp \
../src/drive.cpp \
../src/gruen.cpp \
../src/line.cpp \
../src/util.cpp


OBJS += \
./src/2020_Hannover.o \
./src/CameraCapture.o \
./src/KamelDevices.o \
./src/Logger.o \
./src/VideoServer.o \
./src/drive.o \
./src/gruen.o \
./src/line.o \
./src/util.o

CPP_DEPS += \
./src/2020_Hannover.d \
./src/CameraCapture.d \
./src/KamelDevices.d \
./src/Logger.d \
./src/VideoServer.d \
./src/drive.d \
./src/gruen.d \
./src/line.d \
./src/util.d


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++17 -O3 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
