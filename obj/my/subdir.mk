################################################################################
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../my/adc.c \
../my/base.c \
../my/beep.c \
../my/encoder.c \
../my/findline.c \
../my/imag.c \
../my/include.c \
../my/key.c \
../my/menu.c \
../my/motor.c \
../my/my_imagetransfer.c \
../my/pid.c 

OBJS += \
./my/adc.o \
./my/base.o \
./my/beep.o \
./my/encoder.o \
./my/findline.o \
./my/imag.o \
./my/include.o \
./my/key.o \
./my/menu.o \
./my/motor.o \
./my/my_imagetransfer.o \
./my/pid.o 

C_DEPS += \
./my/adc.d \
./my/base.d \
./my/beep.d \
./my/encoder.d \
./my/findline.d \
./my/imag.d \
./my/include.d \
./my/key.d \
./my/menu.d \
./my/motor.d \
./my/my_imagetransfer.d \
./my/pid.d 


# Each subdirectory must supply rules for building sources it contributes
my/%.o: ../my/%.c
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -O2 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"../Libraries/wch_libraries/Core" -I"../Libraries/wch_libraries/Peripheral" -I"../Libraries/wch_libraries/Startup" -I"../Libraries/seekfree_libraries" -I"../Libraries/seekfree_peripheral" -I"../Libraries/board" -I"../CODE" -I"../USER" -I"E:\Learning\Mecanum_wheel\v222\mailun\my" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

