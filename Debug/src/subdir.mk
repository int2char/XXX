################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/BFS.cpp \
../src/Compare.cpp \
../src/CplexM.cpp \
../src/Edge.cpp \
../src/Graph.cpp \
../src/Heap.cpp \
../src/LagSerial.cpp \
../src/LinkQueue.cpp \
../src/PathArrange.cpp \
../src/dijkstra.cpp \
../src/main.cpp \
../src/node.cpp \
../src/service.cpp \
../src/taskPath.cpp \
../src/valuemark.cpp 

CU_SRCS += \
../src/GAparrel.cu \
../src/GraphPath.cu 

CU_DEPS += \
./src/GAparrel.d \
./src/GraphPath.d 

OBJS += \
./src/BFS.o \
./src/Compare.o \
./src/CplexM.o \
./src/Edge.o \
./src/GAparrel.o \
./src/Graph.o \
./src/GraphPath.o \
./src/Heap.o \
./src/LagSerial.o \
./src/LinkQueue.o \
./src/PathArrange.o \
./src/dijkstra.o \
./src/main.o \
./src/node.o \
./src/service.o \
./src/taskPath.o \
./src/valuemark.o 

CPP_DEPS += \
./src/BFS.d \
./src/Compare.d \
./src/CplexM.d \
./src/Edge.d \
./src/Graph.d \
./src/Heap.d \
./src/LagSerial.d \
./src/LinkQueue.d \
./src/PathArrange.d \
./src/dijkstra.d \
./src/main.d \
./src/node.d \
./src/service.d \
./src/taskPath.d \
./src/valuemark.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-8.0/bin/nvcc -I/home/zhang/addlinux/addlinux/include -I/opt/ibm/ILOG/CPLEX_Studio127/cplex/include -I/opt/ibm/ILOG/CPLEX_Studio127/concert/include -G -g -O0 -Xcompiler -DIL_STD -std=c++11 -gencode arch=compute_35,code=sm_35  -odir "src" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-8.0/bin/nvcc -I/home/zhang/addlinux/addlinux/include -I/opt/ibm/ILOG/CPLEX_Studio127/cplex/include -I/opt/ibm/ILOG/CPLEX_Studio127/concert/include -G -g -O0 -Xcompiler -DIL_STD -std=c++11 --compile  -x c++ -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.cu
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-8.0/bin/nvcc -I/home/zhang/addlinux/addlinux/include -I/opt/ibm/ILOG/CPLEX_Studio127/cplex/include -I/opt/ibm/ILOG/CPLEX_Studio127/concert/include -G -g -O0 -Xcompiler -DIL_STD -std=c++11 -gencode arch=compute_35,code=sm_35  -odir "src" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-8.0/bin/nvcc -I/home/zhang/addlinux/addlinux/include -I/opt/ibm/ILOG/CPLEX_Studio127/cplex/include -I/opt/ibm/ILOG/CPLEX_Studio127/concert/include -G -g -O0 -Xcompiler -DIL_STD -std=c++11 --compile --relocatable-device-code=false -gencode arch=compute_35,code=compute_35 -gencode arch=compute_35,code=sm_35  -x cu -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


