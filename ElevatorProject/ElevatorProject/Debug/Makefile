################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../ElevatorProject.cpp


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
ElevatorProject.o

OBJS_AS_ARGS +=  \
ElevatorProject.o

C_DEPS +=  \
ElevatorProject.d

C_DEPS_AS_ARGS +=  \
ElevatorProject.d

OUTPUT_FILE_PATH +=ElevatorProject.elf

OUTPUT_FILE_PATH_AS_ARGS +=ElevatorProject.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler
./ElevatorProject.o: .././ElevatorProject.cpp
	@echo Building file: $<
	@echo Invoking: AVR8/GNU C Compiler : 5.4.0
	$(QUOTE)D:\AtmelStudio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -funsigned-char -funsigned-bitfields -DDEBUG  -I"D:\AtmelStudio\7.0\Packs\atmel\ATmega_DFP\1.7.374\include"  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -mmcu=atmega32 -B "D:\AtmelStudio\7.0\Packs\atmel\ATmega_DFP\1.7.374\gcc\dev\atmega32" -c -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	




# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR8/GNU Linker : 5.4.0
	$(QUOTE)D:\AtmelStudio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-g++.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="ElevatorProject.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mmcu=atmega32 -B "D:\AtmelStudio\7.0\Packs\atmel\ATmega_DFP\1.7.374\gcc\dev\atmega32"  
	@echo Finished building target: $@
	"D:\AtmelStudio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "ElevatorProject.elf" "ElevatorProject.hex"
	"D:\AtmelStudio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "ElevatorProject.elf" "ElevatorProject.eep" || exit 0
	"D:\AtmelStudio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "ElevatorProject.elf" > "ElevatorProject.lss"
	"D:\AtmelStudio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "ElevatorProject.elf" "ElevatorProject.srec"
	"D:\AtmelStudio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-size.exe" "ElevatorProject.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "ElevatorProject.elf" "ElevatorProject.a" "ElevatorProject.hex" "ElevatorProject.lss" "ElevatorProject.eep" "ElevatorProject.map" "ElevatorProject.srec" "ElevatorProject.usersignatures"
	