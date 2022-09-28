#include "sl_event_handler.h"

#include "em_chip.h"
#include "sl_device_init_nvic.h"
#include "sl_board_init.h"
#include "sl_device_init_dcdc.h"
#include "sl_device_init_hfxo.h"
#include "sl_device_init_lfxo.h"
#include "sl_device_init_clocks.h"
#include "sl_device_init_emu.h"
#include "sl_rail_util_dma.h"
#include "pa_conversions_efr32.h"
#include "sl_rail_util_pti.h"
#include "sl_rail_util_rssi.h"
#include "sl_rail_util_init.h"
#include "sl_board_control.h"
#include "app_log.h"
#include "sl_debug_swo.h"
#include "gpiointerrupt.h"
#include "sl_iostream_init_eusart_instances.h"
#include "sl_iostream_stdlib_config.h"
#include "sl_mpu.h"
#include "nvm3_default.h"
#include "sl_pwm_instances.h"
#include "sl_simple_button_instances.h"
#include "sl_cli_instances.h"
#include "sl_iostream_init_instances.h"
#include "sl_cos.h"

void sl_platform_init(void)
{
  CHIP_Init();
  sl_device_init_nvic();
  sl_board_preinit();
  sl_device_init_dcdc();
  sl_device_init_hfxo();
  sl_device_init_lfxo();
  sl_device_init_clocks();
  sl_device_init_emu();
  sl_board_init();
  nvm3_initDefault();
}

void sl_driver_init(void)
{
  sl_debug_swo_init();
  GPIOINT_Init();
  sl_pwm_init_instances();
  sl_simple_button_init_instances();
  sl_cos_send_config();
}

void sl_service_init(void)
{
  sl_board_configure_vcom();
  sl_iostream_stdlib_disable_buffering();
  sl_mpu_disable_execute_from_ram();
  sl_iostream_init_instances();
  sl_cli_instances_init();
}

void sl_stack_init(void)
{
  sl_rail_util_dma_init();
  sl_rail_util_pa_init();
  sl_rail_util_pti_init();
  sl_rail_util_rssi_init();
  sl_rail_util_init();
}

void sl_internal_app_init(void)
{
  app_log_init();
}

void sl_platform_process_action(void)
{
}

void sl_service_process_action(void)
{
  sl_cli_instances_tick();
}

void sl_stack_process_action(void)
{
}

void sl_internal_app_process_action(void)
{
}

void sl_iostream_init_instances(void)
{
  sl_iostream_eusart_init_instances();
}
