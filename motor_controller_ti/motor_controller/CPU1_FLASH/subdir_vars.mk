################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../2837xD_FLASH_CLA_lnk_cpu1.cmd 

LIB_SRCS += \
C:/ti/c2000/C2000Ware_3_01_00_00/libraries/math/IQmath/c28/lib/IQmath_fpu32.lib \
C:/ti/c2000/C2000Ware_3_01_00_00/libraries/dsp/FPU/c28/lib/c28x_fpu_dsp_library.lib \
C:/ti/c2000/C2000Ware_3_01_00_00/driverlib/f2837xd/driverlib/ccs/Debug/driverlib.lib 

C_SRCS += \
../current_control.c \
../interrupt_routines.c \
../main.c \
../peripheral_initializations.c 

C_DEPS += \
./current_control.d \
./interrupt_routines.d \
./main.d \
./peripheral_initializations.d 

OBJS += \
./current_control.obj \
./interrupt_routines.obj \
./main.obj \
./peripheral_initializations.obj 

OBJS__QUOTED += \
"current_control.obj" \
"interrupt_routines.obj" \
"main.obj" \
"peripheral_initializations.obj" 

C_DEPS__QUOTED += \
"current_control.d" \
"interrupt_routines.d" \
"main.d" \
"peripheral_initializations.d" 

C_SRCS__QUOTED += \
"../interrupt_routines.c" \
"../main.c" \
"../peripheral_initializations.c" 


