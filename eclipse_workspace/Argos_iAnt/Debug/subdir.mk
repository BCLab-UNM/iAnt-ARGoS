################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../CPFA.cpp \
../Controller.cpp \
../FoodData.cpp \
../LoopFunctions.cpp \
../NavigationData.cpp \
../PheromoneList.cpp \
../PheromoneWaypoint.cpp \
../_iAnt_controller.cpp \
../_iAnt_data_structures.cpp \
../_iAnt_genetic_algorithm.cpp \
../_iAnt_loop_functions.cpp \
../_iAnt_qt_user_functions.cpp 

OBJS += \
./CPFA.o \
./Controller.o \
./FoodData.o \
./LoopFunctions.o \
./NavigationData.o \
./PheromoneList.o \
./PheromoneWaypoint.o \
./_iAnt_controller.o \
./_iAnt_data_structures.o \
./_iAnt_genetic_algorithm.o \
./_iAnt_loop_functions.o \
./_iAnt_qt_user_functions.o 

CPP_DEPS += \
./CPFA.d \
./Controller.d \
./FoodData.d \
./LoopFunctions.d \
./NavigationData.d \
./PheromoneList.d \
./PheromoneWaypoint.d \
./_iAnt_controller.d \
./_iAnt_data_structures.d \
./_iAnt_genetic_algorithm.d \
./_iAnt_loop_functions.d \
./_iAnt_qt_user_functions.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/lua5.1 -I/usr/include/qt4 -I/usr/include/qt4/QtCore -I/usr/include/qt4/QtOpenGL -O0 -g3 -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


