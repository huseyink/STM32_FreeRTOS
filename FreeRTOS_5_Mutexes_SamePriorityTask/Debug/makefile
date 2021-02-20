################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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
FreeRTOS_5_Mutexes_SamePriorityTask.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
FreeRTOS_5_Mutexes_SamePriorityTask.list \

OBJCOPY_BIN += \
FreeRTOS_5_Mutexes_SamePriorityTask.bin \


# All Target
all: FreeRTOS_5_Mutexes_SamePriorityTask.elf secondary-outputs

# Tool invocations
FreeRTOS_5_Mutexes_SamePriorityTask.elf: $(OBJS) $(USER_OBJS) C:\Users\kouhu\STM32CubeIDE\workspace_1.4.2\FreeRTOS_5_Mutexes_SamePriorityTask\STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "FreeRTOS_5_Mutexes_SamePriorityTask.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\kouhu\STM32CubeIDE\workspace_1.4.2\FreeRTOS_5_Mutexes_SamePriorityTask\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="FreeRTOS_5_Mutexes_SamePriorityTask.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

FreeRTOS_5_Mutexes_SamePriorityTask.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "FreeRTOS_5_Mutexes_SamePriorityTask.list"
	@echo 'Finished building: $@'
	@echo ' '

FreeRTOS_5_Mutexes_SamePriorityTask.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "FreeRTOS_5_Mutexes_SamePriorityTask.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
