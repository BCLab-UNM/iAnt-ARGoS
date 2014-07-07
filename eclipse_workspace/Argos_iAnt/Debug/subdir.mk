################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../iAnt_controller.cpp \
../iAnt_data_structures.cpp \
../iAnt_genetic_algorithm.cpp \
../iAnt_loop_functions.cpp \
../iAnt_qt_user_functions.cpp 

OBJS += \
./iAnt_controller.o \
./iAnt_data_structures.o \
./iAnt_genetic_algorithm.o \
./iAnt_loop_functions.o \
./iAnt_qt_user_functions.o 

CPP_DEPS += \
./iAnt_controller.d \
./iAnt_data_structures.d \
./iAnt_genetic_algorithm.d \
./iAnt_loop_functions.d \
./iAnt_qt_user_functions.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/lua5.1 -I/usr/include/qt4 -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtOpenGL -O0 -g3 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


