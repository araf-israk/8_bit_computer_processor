################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/EEPROM_RW/eeprom_rw.c 

OBJS += \
./Drivers/EEPROM_RW/eeprom_rw.o 

C_DEPS += \
./Drivers/EEPROM_RW/eeprom_rw.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/EEPROM_RW/%.o Drivers/EEPROM_RW/%.su Drivers/EEPROM_RW/%.cyclo: ../Drivers/EEPROM_RW/%.c Drivers/EEPROM_RW/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"D:/STM32Projects/stm32_eeprom_burner/Drivers/OLED" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"D:/STM32Projects/stm32_eeprom_burner/Drivers/EEPROM_RW" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-EEPROM_RW

clean-Drivers-2f-EEPROM_RW:
	-$(RM) ./Drivers/EEPROM_RW/eeprom_rw.cyclo ./Drivers/EEPROM_RW/eeprom_rw.d ./Drivers/EEPROM_RW/eeprom_rw.o ./Drivers/EEPROM_RW/eeprom_rw.su

.PHONY: clean-Drivers-2f-EEPROM_RW

