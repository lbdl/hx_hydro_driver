################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include gecko_sdk_4.2.2/util/third_party/freertos/kernel/portable/SiliconLabs/subdir.mk
-include gecko_sdk_4.2.2/util/third_party/freertos/kernel/portable/MemMang/subdir.mk
-include gecko_sdk_4.2.2/util/third_party/freertos/kernel/portable/GCC/ARM_CM33_NTZ/non_secure/subdir.mk
-include gecko_sdk_4.2.2/util/third_party/freertos/kernel/subdir.mk
-include gecko_sdk_4.2.2/util/third_party/freertos/cmsis/Source/subdir.mk
-include gecko_sdk_4.2.2/util/third_party/aws_iot_libs/vendors/siliconlabs/boards/all/ports/common_io/iot_adc/source/subdir.mk
-include gecko_sdk_4.2.2/platform/service/udelay/src/subdir.mk
-include gecko_sdk_4.2.2/platform/service/system/src/subdir.mk
-include gecko_sdk_4.2.2/platform/service/sleeptimer/src/subdir.mk
-include gecko_sdk_4.2.2/platform/service/power_manager/src/subdir.mk
-include gecko_sdk_4.2.2/platform/service/hfxo_manager/src/subdir.mk
-include gecko_sdk_4.2.2/platform/service/device_init/src/subdir.mk
-include gecko_sdk_4.2.2/platform/emlib/src/subdir.mk
-include gecko_sdk_4.2.2/platform/emdrv/gpiointerrupt/src/subdir.mk
-include gecko_sdk_4.2.2/platform/emdrv/dmadrv/src/subdir.mk
-include gecko_sdk_4.2.2/platform/driver/pwm/src/subdir.mk
-include gecko_sdk_4.2.2/platform/driver/leddrv/src/subdir.mk
-include gecko_sdk_4.2.2/platform/driver/button/src/subdir.mk
-include gecko_sdk_4.2.2/platform/common/toolchain/src/subdir.mk
-include gecko_sdk_4.2.2/platform/common/src/subdir.mk
-include gecko_sdk_4.2.2/platform/Device/SiliconLabs/EFR32BG22/Source/subdir.mk
-include gecko_sdk_4.2.2/platform/CMSIS/RTOS2/Source/subdir.mk
-include gecko_sdk_4.2.2/hardware/driver/mx25_flash_shutdown/src/sl_mx25_flash_shutdown_usart/subdir.mk
-include gecko_sdk_4.2.2/hardware/board/src/subdir.mk
-include autogen/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := simple_button_try
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: main-build

# Main-build Target
main-build: simple_button_try.axf

# Tool invocations
simple_button_try.axf: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -T "/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/autogen/linkerfile.ld" -Xlinker --gc-sections -Xlinker -Map="simple_button_try.map" -mfpu=fpv5-sp-d16 -mfloat-abi=hard --specs=nano.specs -o simple_button_try.axf -Wl,--start-group "./app.o" "./main.o" "./simple_button_baremetal.o" "./autogen/iot_adc_desc.o" "./autogen/sl_board_default_init.o" "./autogen/sl_device_init_clocks.o" "./autogen/sl_event_handler.o" "./autogen/sl_pwm_init.o" "./autogen/sl_simple_button_instances.o" "./autogen/sl_simple_led_instances.o" "./gecko_sdk_4.2.2/hardware/board/src/sl_board_control_gpio.o" "./gecko_sdk_4.2.2/hardware/board/src/sl_board_init.o" "./gecko_sdk_4.2.2/hardware/driver/mx25_flash_shutdown/src/sl_mx25_flash_shutdown_usart/sl_mx25_flash_shutdown.o" "./gecko_sdk_4.2.2/platform/CMSIS/RTOS2/Source/os_systick.o" "./gecko_sdk_4.2.2/platform/Device/SiliconLabs/EFR32BG22/Source/startup_efr32bg22.o" "./gecko_sdk_4.2.2/platform/Device/SiliconLabs/EFR32BG22/Source/system_efr32bg22.o" "./gecko_sdk_4.2.2/platform/common/src/sl_assert.o" "./gecko_sdk_4.2.2/platform/common/src/sl_slist.o" "./gecko_sdk_4.2.2/platform/common/toolchain/src/sl_memory.o" "./gecko_sdk_4.2.2/platform/driver/button/src/sl_button.o" "./gecko_sdk_4.2.2/platform/driver/button/src/sl_simple_button.o" "./gecko_sdk_4.2.2/platform/driver/leddrv/src/sl_led.o" "./gecko_sdk_4.2.2/platform/driver/leddrv/src/sl_simple_led.o" "./gecko_sdk_4.2.2/platform/driver/pwm/src/sl_pwm.o" "./gecko_sdk_4.2.2/platform/emdrv/dmadrv/src/dmadrv.o" "./gecko_sdk_4.2.2/platform/emdrv/gpiointerrupt/src/gpiointerrupt.o" "./gecko_sdk_4.2.2/platform/emlib/src/em_burtc.o" "./gecko_sdk_4.2.2/platform/emlib/src/em_cmu.o" "./gecko_sdk_4.2.2/platform/emlib/src/em_core.o" "./gecko_sdk_4.2.2/platform/emlib/src/em_emu.o" "./gecko_sdk_4.2.2/platform/emlib/src/em_gpio.o" "./gecko_sdk_4.2.2/platform/emlib/src/em_iadc.o" "./gecko_sdk_4.2.2/platform/emlib/src/em_ldma.o" "./gecko_sdk_4.2.2/platform/emlib/src/em_msc.o" "./gecko_sdk_4.2.2/platform/emlib/src/em_rtcc.o" "./gecko_sdk_4.2.2/platform/emlib/src/em_system.o" "./gecko_sdk_4.2.2/platform/emlib/src/em_timer.o" "./gecko_sdk_4.2.2/platform/emlib/src/em_usart.o" "./gecko_sdk_4.2.2/platform/service/device_init/src/sl_device_init_dcdc_s2.o" "./gecko_sdk_4.2.2/platform/service/device_init/src/sl_device_init_emu_s2.o" "./gecko_sdk_4.2.2/platform/service/device_init/src/sl_device_init_hfxo_s2.o" "./gecko_sdk_4.2.2/platform/service/device_init/src/sl_device_init_lfxo_s2.o" "./gecko_sdk_4.2.2/platform/service/device_init/src/sl_device_init_nvic.o" "./gecko_sdk_4.2.2/platform/service/hfxo_manager/src/sl_hfxo_manager.o" "./gecko_sdk_4.2.2/platform/service/hfxo_manager/src/sl_hfxo_manager_hal_s2.o" "./gecko_sdk_4.2.2/platform/service/power_manager/src/sl_power_manager.o" "./gecko_sdk_4.2.2/platform/service/power_manager/src/sl_power_manager_debug.o" "./gecko_sdk_4.2.2/platform/service/power_manager/src/sl_power_manager_hal_s2.o" "./gecko_sdk_4.2.2/platform/service/sleeptimer/src/sl_sleeptimer.o" "./gecko_sdk_4.2.2/platform/service/sleeptimer/src/sl_sleeptimer_hal_burtc.o" "./gecko_sdk_4.2.2/platform/service/sleeptimer/src/sl_sleeptimer_hal_prortc.o" "./gecko_sdk_4.2.2/platform/service/sleeptimer/src/sl_sleeptimer_hal_rtcc.o" "./gecko_sdk_4.2.2/platform/service/sleeptimer/src/sl_sleeptimer_hal_timer.o" "./gecko_sdk_4.2.2/platform/service/system/src/sl_system_init.o" "./gecko_sdk_4.2.2/platform/service/system/src/sl_system_kernel.o" "./gecko_sdk_4.2.2/platform/service/udelay/src/sl_udelay.o" "./gecko_sdk_4.2.2/platform/service/udelay/src/sl_udelay_armv6m_gcc.o" "./gecko_sdk_4.2.2/util/third_party/aws_iot_libs/vendors/siliconlabs/boards/all/ports/common_io/iot_adc/source/iot_adc_drv_inc.o" "./gecko_sdk_4.2.2/util/third_party/aws_iot_libs/vendors/siliconlabs/boards/all/ports/common_io/iot_adc/source/iot_adc_drv_sar.o" "./gecko_sdk_4.2.2/util/third_party/aws_iot_libs/vendors/siliconlabs/boards/all/ports/common_io/iot_adc/source/iot_adc_hal.o" "./gecko_sdk_4.2.2/util/third_party/aws_iot_libs/vendors/siliconlabs/boards/all/ports/common_io/iot_adc/source/iot_adc_pinout_drv.o" "./gecko_sdk_4.2.2/util/third_party/freertos/cmsis/Source/cmsis_os2.o" "./gecko_sdk_4.2.2/util/third_party/freertos/kernel/croutine.o" "./gecko_sdk_4.2.2/util/third_party/freertos/kernel/event_groups.o" "./gecko_sdk_4.2.2/util/third_party/freertos/kernel/list.o" "./gecko_sdk_4.2.2/util/third_party/freertos/kernel/queue.o" "./gecko_sdk_4.2.2/util/third_party/freertos/kernel/stream_buffer.o" "./gecko_sdk_4.2.2/util/third_party/freertos/kernel/tasks.o" "./gecko_sdk_4.2.2/util/third_party/freertos/kernel/timers.o" "./gecko_sdk_4.2.2/util/third_party/freertos/kernel/portable/GCC/ARM_CM33_NTZ/non_secure/port.o" "./gecko_sdk_4.2.2/util/third_party/freertos/kernel/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.o" "./gecko_sdk_4.2.2/util/third_party/freertos/kernel/portable/MemMang/heap_4.o" "./gecko_sdk_4.2.2/util/third_party/freertos/kernel/portable/SiliconLabs/tick_power_manager.o" -lgcc -lc -lm -lnosys -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: simple_button_try.hex'
	arm-none-eabi-objcopy -O ihex "simple_button_try.axf" "simple_button_try.hex"
	@echo ' '

	@echo 'Building bin file: simple_button_try.bin'
	arm-none-eabi-objcopy -O binary "simple_button_try.axf" "simple_button_try.bin"
	@echo ' '

	@echo 'Building s37 file: simple_button_try.s37'
	arm-none-eabi-objcopy -O srec "simple_button_try.axf" "simple_button_try.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "simple_button_try.axf" -A
	@echo ' '

# Other Targets
clean:
	-@$(foreach FILE, $(EXECUTABLES), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(OBJS), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(C_DEPS), $(shell $(RM) $(FILE)) )
	-$(RM) simple_button_try.axf
	-@echo ' '

.PHONY: all clean dependents main-build

-include ../makefile.targets
