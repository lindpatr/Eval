################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Dev/SiLabs/SDKs/gecko_sdk/app/common/util/app_log/app_log.c 

OBJS += \
./gecko_sdk_4.1.1/app/common/util/app_log/app_log.o 

C_DEPS += \
./gecko_sdk_4.1.1/app/common/util/app_log/app_log.d 


# Each subdirectory must supply rules for building sources it contributes
gecko_sdk_4.1.1/app/common/util/app_log/app_log.o: C:/Dev/SiLabs/SDKs/gecko_sdk/app/common/util/app_log/app_log.c gecko_sdk_4.1.1/app/common/util/app_log/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DEFR32MG22C224F512IM40=1' '-DSL_BOARD_NAME="BRD4182A"' '-DSL_BOARD_REV="B06"' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' -I"C:\Dev\SiLabs\Eval\UniDir\config" -I"C:\Dev\SiLabs\Eval\UniDir\autogen" -I"C:\Dev\SiLabs\Eval\UniDir" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32MG22/Include" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//app/common/util/app_assert" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//app/common/util/app_log" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/common/inc" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/driver/button/inc" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/service/cli/inc" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/service/cli/src" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/CMSIS/Core/Include" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//hardware/driver/configuration_over_swo/inc" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/driver/debug/inc" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/emdrv/gpiointerrupt/inc" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/service/iostream/inc" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/driver/leddrv/inc" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/service/mpu/inc" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//util/third_party/printf" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//util/third_party/printf/inc" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/radio/rail_lib/common" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/ble" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/ieee802154" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/radio/rail_lib/protocol/zwave" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/radio/rail_lib/chip/efr32/efr32xg2x" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/rail_util_callbacks" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/pa-conversions" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/pa-conversions/efr32xg22" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/rail_util_protocol" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/rail_util_pti" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/radio/rail_lib/plugin/rail_util_rssi" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//app/flex/component/rail/sl_flex_rail_packet_asm" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Dev/SiLabs/SDKs/gecko_sdk//platform/service/udelay/inc" -Og -Wall -Wextra -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse -fno-builtin-printf -fno-builtin-sprintf -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.1.1/app/common/util/app_log/app_log.d" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


