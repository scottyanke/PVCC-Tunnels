################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: pvcc-tunnels.ihx

# Tool invocations
pvcc-tunnels.ihx: $(RELS) $(USER_RELS)
	@echo 'Building target: $@'
	@echo 'Invoking: SDCC Linker'
	sdcc -L/home/scott/Samples/stm8-bare-min/stm8 -mstm8 -lstm8 --out-fmt-ihx -o "pvcc-tunnels.ihx" $(RELS) $(USER_RELS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(RELS)$(C_DEPS) pvcc-tunnels.ihx
	-@echo ' '

.PHONY: all clean dependents

-include ../makefile.targets
