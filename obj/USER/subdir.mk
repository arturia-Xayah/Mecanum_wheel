################################################################################
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USER/fuzzy.c \
../USER/isr.c \
../USER/main.c \
../USER/run.c 

OBJS += \
./USER/fuzzy.o \
./USER/isr.o \
./USER/main.o \
./USER/run.o 

C_DEPS += \
./USER/fuzzy.d \
./USER/isr.d \
./USER/main.d \
./USER/run.d 


# Each subdirectory must supply rules for building sources it contributes
USER/%.o: ../USER/%.c
	@	@	riscv-none-embed-gcc -march=rv32imac -mabi=ilp32 -msmall-data-limit=8 -mno-save-restore -O2 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"../Libraries/wch_libraries/Core" -I"../Libraries/wch_libraries/Peripheral" -I"../Libraries/wch_libraries/Startup" -I"../Libraries/seekfree_libraries" -I"../Libraries/seekfree_peripheral" -I"../Libraries/board" -I"../CODE" -I"../USER" -I"E:\Learning\Mecanum_wheel\v222\mailun\my" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

