################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ..\makefile.init

RM := del 

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include core\core\libb64\subdir.mk
-include core\core\subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ..\makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
AR += \
C:\Users\kgandhi\Desktop\Lab2_Ultrasonic\Release\arduino.ar \

ELF += \
Lab2_Ultrasonic.elf \

HEX += \
Lab2_Ultrasonic.hex \


# All Target
all: Lab2_Ultrasonic

# Tool invocations
Lab2_Ultrasonic: $(HEX)
	@echo 'Building target: $@'
	@echo 'Printing size:'
	"C:\RBE\sloeber\/arduinoPlugin/packages/arduino/hardware/espressif/esp32/tools/xtensa-esp32-elf/bin/xtensa-esp32-elf-size" -A "C:/Users/kgandhi/Desktop/Lab2_Ultrasonic/Release/Lab2_Ultrasonic.elf"
	@echo 'Finished building target: $@'
	@echo ' '

C:\Users\kgandhi\Desktop\Lab2_Ultrasonic\Release\arduino.ar: $(AR_OBJ)
	@echo 'Starting archiver'
	"C:\RBE\sloeber\/arduinoPlugin/packages/arduino/hardware/espressif/esp32/tools/xtensa-esp32-elf/bin/xtensa-esp32-elf-ar" cru  "C:/Users/kgandhi/Desktop/Lab2_Ultrasonic/Release/arduino.ar"    C:/Users/kgandhi/Desktop/Lab2_Ultrasonic/Release/arduino.ar $(AR_OBJ)
	@echo 'Finished building: $@'
	@echo ' '

Lab2_Ultrasonic.elf: $(LINK_OBJ) $(AR)
	@echo 'Starting combiner'
	"C:\RBE\sloeber\/arduinoPlugin/packages/arduino/hardware/espressif/esp32/tools/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc" -nostdlib "-LC:\RBE\sloeber\/arduinoPlugin/packages/arduino/hardware/espressif/esp32/tools/sdk/lib" "-LC:\RBE\sloeber\/arduinoPlugin/packages/arduino/hardware/espressif/esp32/tools/sdk/ld" -T esp32_out.ld -T esp32.common.ld -T esp32.rom.ld -T esp32.peripherals.ld -T esp32.rom.spiram_incompatible_fns.ld -u ld_include_panic_highint_hdl -u call_user_start_cpu0 -Wl,--gc-sections -Wl,-static -Wl,--undefined=uxTopUsedPriority  -u __cxa_guard_dummy -u __cxx_fatal_exception  -Wl,--start-group   $(LINK_OBJ)  "C:/Users/kgandhi/Desktop/Lab2_Ultrasonic/Release/arduino.ar" -lgcc -lopenssl -lbtdm_app -lfatfs -lwps -lhttp_server -lcoexist -lwear_levelling -lesp_http_client -lhal -lnewlib -ldriver -lbootloader_support -lpp -lmesh -lsmartconfig -ljsmn -lwpa -lethernet -lphy -lapp_trace -lconsole -lulp -lwpa_supplicant -lfreertos -lbt -lmicro-ecc -lcxx -lxtensa-debug-module -ltcp_transport -lmdns -lvfs -lesp_ringbuf -lsoc -lcore -lsdmmc -lcoap -ltcpip_adapter -lc_nano -lesp-tls -lasio -lrtc -lspi_flash -lwpa2 -lesp32 -lapp_update -lnghttp -lspiffs -lespnow -lnvs_flash -lesp_adc_cal -llog -lsmartconfig_ack -lexpat -lm -lmqtt -lc -lheap -lmbedtls -llwip -lnet80211 -lpthread -ljson -lesp_https_ota  -lstdc++ -Wl,--end-group -Wl,-EL -o "C:/Users/kgandhi/Desktop/Lab2_Ultrasonic/Release/Lab2_Ultrasonic.elf" C:/Users/kgandhi/Desktop/Lab2_Ultrasonic/Release/arduino.ar
	@echo 'Finished building: $@'
	@echo ' '

Lab2_Ultrasonic.hex: $(ELF)
	@echo 'Do all objcopy commands'
	${A.RECIPE.OBJCOPY.EEP.PATTERN}
	${A.RECIPE.OBJCOPY.HEX.PATTERN}
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(C_UPPER_DEPS)$(PDE_DEPS)$(C_DEPS)$(AR)$(CC_DEPS)$(AR_OBJ)$(C++_DEPS)$(LINK_OBJ)$(CXX_DEPS)$(ASM_DEPS)$(HEX)$(INO_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(ELF)$(CPP_DEPS)
	-@echo ' '

.PHONY: all clean dependents

-include ..\makefile.targets
