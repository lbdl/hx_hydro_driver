################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../gecko_sdk_4.2.2/platform/emdrv/dmadrv/src/dmadrv.c 

OBJS += \
./gecko_sdk_4.2.2/platform/emdrv/dmadrv/src/dmadrv.o 

C_DEPS += \
./gecko_sdk_4.2.2/platform/emdrv/dmadrv/src/dmadrv.d 


# Each subdirectory must supply rules for building sources it contributes
gecko_sdk_4.2.2/platform/emdrv/dmadrv/src/dmadrv.o: ../gecko_sdk_4.2.2/platform/emdrv/dmadrv/src/dmadrv.c gecko_sdk_4.2.2/platform/emdrv/dmadrv/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32BG22C224F512IM40=1' '-DSL_BOARD_NAME="BRD4184A"' '-DSL_BOARD_REV="A02"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/config" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/autogen" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/Device/SiliconLabs/EFR32BG22/Include" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/common/inc" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/hardware/board/inc" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/driver/button/inc" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/CMSIS/Core/Include" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/CMSIS/RTOS2/Include" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/service/device_init/inc" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/emdrv/dmadrv/inc" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/emdrv/common/inc" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/emlib/inc" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/util/third_party/freertos/cmsis/Include" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/util/third_party/freertos/kernel/include" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/util/third_party/freertos/kernel/portable/GCC/ARM_CM33_NTZ/non_secure" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/emdrv/gpiointerrupt/inc" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/service/hfxo_manager/inc" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/util/third_party/aws_iot_libs/libraries/abstractions/common_io/include" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/util/third_party/aws_iot_libs/vendors/siliconlabs/boards/all/ports/common_io/iot_adc/include" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/driver/leddrv/inc" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/service/power_manager/inc" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/driver/pwm/inc" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/common/toolchain/inc" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/service/system/inc" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/service/sleeptimer/inc" -I"/Users/tims/SimplicityStudio/v5_workspace/simple_button_try/gecko_sdk_4.2.2/platform/service/udelay/inc" -Os -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse --specs=nano.specs -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.2.2/platform/emdrv/dmadrv/src/dmadrv.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


