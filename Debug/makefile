################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include Core/Inc/minmea-master/subdir.mk
-include B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/cJSON/subdir.mk
-include B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MbedTLS/subdir.mk
-include B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MQTTPacket/subdir.mk
-include B-L475E-IOT01_GenericMQTT/Middlewares/Third_Party/MQTTClient-C/subdir.mk
-include B-L475E-IOT01_GenericMQTT/Drivers/STM32L4xx_HAL_Driver/subdir.mk
-include B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LSM6DSL/subdir.mk
-include B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LPS22HB/subdir.mk
-include B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/LIS3MDL/subdir.mk
-include B-L475E-IOT01_GenericMQTT/Drivers/BSP/Components/HTS221/subdir.mk
-include B-L475E-IOT01_GenericMQTT/Drivers/BSP/B-L475E-IOT01/subdir.mk
-include B-L475E-IOT01_GenericMQTT/Drivers/BSP/subdir.mk
-include B-L475E-IOT01_GenericMQTT/Application/Wifi/subdir.mk
-include B-L475E-IOT01_GenericMQTT/Application/Common/subdir.mk
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
Codigo9\ ADC+UART+CLD_IOT.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Codigo9\ ADC+UART+CLD_IOT.list \

OBJCOPY_BIN += \
Codigo9\ ADC+UART+CLD_IOT.bin \


# All Target
all: Codigo9\ ADC+UART+CLD_IOT.elf secondary-outputs

# Tool invocations
Codigo9\ ADC+UART+CLD_IOT.elf: $(OBJS) $(USER_OBJS) D:\UPM\4.2\ -\ TFG\Cap\ V-\ Implementacion\ y\ Desarrollo\Proyectos\ BLIOTA\ STCUBE\9.-Codigo\ Integracion\ ADC+UART+CLD\Codigo9\ ADC+UART+CLD_IOT\STM32L475VGTX_FLASH.ld
	arm-none-eabi-gcc -o "Codigo9 ADC+UART+CLD_IOT.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\UPM\4.2 - TFG\Cap V- Implementacion y Desarrollo\Proyectos BLIOTA STCUBE\9.-Codigo Integracion ADC+UART+CLD\Codigo9 ADC+UART+CLD_IOT\STM32L475VGTX_FLASH.ld" -Wl,-Map="Codigo9 ADC+UART+CLD_IOT.map" -Wl,--gc-sections -static -L../Middlewares/ST/STM32_MotionFX_Library/Lib --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -u _printf_float -u _scanf_float
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Codigo9\ ADC+UART+CLD_IOT.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Codigo9 ADC+UART+CLD_IOT.list"
	@echo 'Finished building: $@'
	@echo ' '

Codigo9\ ADC+UART+CLD_IOT.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Codigo9 ADC+UART+CLD_IOT.bin"
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
