################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
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
EXECUTABLES += \
FreeRTOS_1_TaskYonetimi.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
FreeRTOS_1_TaskYonetimi.list \

OBJCOPY_HEX += \
FreeRTOS_1_TaskYonetimi.hex \

OBJCOPY_BIN += \
FreeRTOS_1_TaskYonetimi.bin \


# All Target
all: FreeRTOS_1_TaskYonetimi.elf secondary-outputs

# Tool invocations
FreeRTOS_1_TaskYonetimi.elf: $(OBJS) $(USER_OBJS) C:\Users\kouhu\STM32CubeIDE\workspace_1.4.2\FreeRTOS_1_TaskYonetimi\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "FreeRTOS_1_TaskYonetimi.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\kouhu\STM32CubeIDE\workspace_1.4.2\FreeRTOS_1_TaskYonetimi\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="FreeRTOS_1_TaskYonetimi.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

FreeRTOS_1_TaskYonetimi.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "FreeRTOS_1_TaskYonetimi.list"
	@echo 'Finished building: $@'
	@echo ' '

FreeRTOS_1_TaskYonetimi.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "FreeRTOS_1_TaskYonetimi.hex"
	@echo 'Finished building: $@'
	@echo ' '

FreeRTOS_1_TaskYonetimi.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "FreeRTOS_1_TaskYonetimi.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
