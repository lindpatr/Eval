/*
 * common_custom_sl_pwm.c
 *
 *  Created on: 17 oct. 2022
 *      Author: BEL-LinPat
 */

#include "common_custom_sl_pwm.h"

void sl_pwm_set_duty_cycle_percent(sl_pwm_instance_t *pwm, float percent)
{
  uint32_t top = TIMER_TopGet(pwm->timer);

  // Set compare value
  TIMER_CompareBufSet(pwm->timer, pwm->channel, (uint32_t)(top * percent));
}

float sl_pwm_get_duty_cycle_percent(sl_pwm_instance_t *pwm)
{
  uint32_t top = TIMER_TopGet(pwm->timer);
  uint32_t compare = TIMER_CaptureGet(pwm->timer, pwm->channel);

  float percent = (float)(compare / top);

  return percent;
}

void sl_pwm_set_duty_cycle_step(sl_pwm_instance_t *pwm, uint32_t step)
{
  uint32_t top = TIMER_TopGet(pwm->timer);

  if (step > top)
  {
      step = top;
  }

  // Set compare value
  TIMER_CompareBufSet(pwm->timer, pwm->channel, step);
}

uint32_t sl_pwm_get_duty_cycle_step(sl_pwm_instance_t *pwm)
{
    return TIMER_CaptureGet(pwm->timer, pwm->channel);
}

uint32_t sl_pwm_get_max_duty_cycle_step(sl_pwm_instance_t *pwm)
{
    return TIMER_TopGet(pwm->timer);
}
