################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/button.c \
../code/buzzer.c \
../code/menu.c \
../code/param.c \
../code/servo.c \
../code/speed.c \
../code/uart.c 

OBJS += \
./code/button.o \
./code/buzzer.o \
./code/menu.o \
./code/param.o \
./code/servo.o \
./code/speed.o \
./code/uart.o 

COMPILED_SRCS += \
./code/button.src \
./code/buzzer.src \
./code/menu.src \
./code/param.src \
./code/servo.src \
./code/speed.src \
./code/uart.src 

C_DEPS += \
./code/button.d \
./code/buzzer.d \
./code/menu.d \
./code/param.d \
./code/servo.d \
./code/speed.d \
./code/uart.d 


# Each subdirectory must supply rules for building sources it contributes
code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb "-fD:/ADS/Workspace/lower_com/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file="$(@:.src=.d)" --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo 'Finished building: $<'
	@echo ' '

code/%.o: ./code/%.src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


