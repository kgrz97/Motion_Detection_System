################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
Wykrywacz_ruchu.lss \

FLASH_IMAGE += \
Wykrywacz_ruchu.hex \

EEPROM_IMAGE += \
Wykrywacz_ruchu.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: Wykrywacz_ruchu.elf secondary-outputs

# Tool invocations
Wykrywacz_ruchu.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,Wykrywacz_ruchu.map -mmcu=atmega8 -o "Wykrywacz_ruchu.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Wykrywacz_ruchu.lss: Wykrywacz_ruchu.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S Wykrywacz_ruchu.elf  >"Wykrywacz_ruchu.lss"
	@echo 'Finished building: $@'
	@echo ' '

Wykrywacz_ruchu.hex: Wykrywacz_ruchu.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex Wykrywacz_ruchu.elf  "Wykrywacz_ruchu.hex"
	@echo 'Finished building: $@'
	@echo ' '

Wykrywacz_ruchu.eep: Wykrywacz_ruchu.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex Wykrywacz_ruchu.elf  "Wykrywacz_ruchu.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: Wykrywacz_ruchu.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=atmega8 Wykrywacz_ruchu.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(LSS)$(C_DEPS) Wykrywacz_ruchu.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
