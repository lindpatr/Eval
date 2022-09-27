#include "sl_emlib_gpio_simple_init.h"
#include "sl_emlib_gpio_init_alarm_temp_config.h"
#include "em_gpio.h"
#include "em_cmu.h"

void sl_emlib_gpio_simple_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_ALARM_TEMP_PORT,
                  SL_EMLIB_GPIO_INIT_ALARM_TEMP_PIN,
                  SL_EMLIB_GPIO_INIT_ALARM_TEMP_MODE,
                  SL_EMLIB_GPIO_INIT_ALARM_TEMP_DOUT);
}
