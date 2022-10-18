/*
 * common_custom_sl_pwm.c
 *
 *  Created on: 17 oct. 2022
 *      Author: BEL-LinPat
 */

#include "common_custom_sl_pwm.h"

/**************************************************************************//**
 * @brief
 *   Set duty cycle for PWM waveform.
 *
 * @param[in] pwm
 *   PWM driver instance
 *
 * @param[in] percent
 *   Percent (between 0.0 and 1.0) of the PWM period waveform is in the state defined
 *   as the active polarity in the driver configuration
 *****************************************************************************/
void sl_pwm_set_duty_cycle_percent(sl_pwm_instance_t *pwm, float percent)
{
  uint32_t top = TIMER_TopGet(pwm->timer);

  // Set compare value
  TIMER_CompareBufSet(pwm->timer, pwm->channel, (uint32_t)(top * percent));
}

/**************************************************************************//**
 * @brief
 *  Get PWM duty cycle.
 *
 * @param[in] pwm
 *   PWM driver instance
 *
 * @return
 *   Percent Percent (between 0.0 and 1.0) of the PWM period waveform is in the state defined
 *   as the active polarity in the driver configuration
 *****************************************************************************/
float sl_pwm_get_duty_cycle_percent(sl_pwm_instance_t *pwm)
{
  uint32_t top = TIMER_TopGet(pwm->timer);
  uint32_t compare = TIMER_CaptureGet(pwm->timer, pwm->channel);

  float percent = (float)(compare / top);

  return percent;
}

/**************************************************************************//**
 * @brief
 *   Set duty cycle for PWM waveform.
 *
 * @param[in] pwm
 *   PWM driver instance
 *
 * @param[in] percent
 *   Step (between 0 and sl_pwm_get_max_duty_cycle_step)
 *   of the PWM period waveform is in the state defined
 *   as the active polarity in the driver configuration
 *****************************************************************************/
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

/**************************************************************************//**
 * @brief
 *   Get duty cycle for PWM waveform.
 *
 * @param[in] pwm
 *   PWM driver instance
 *
 * @return
 *   Return steps (between 0 and sl_pwm_get_max_duty_cycle_step)
 *   of the PWM period waveform is in the state defined
 *   as the active polarity in the driver configuration
 *****************************************************************************/
uint32_t sl_pwm_get_duty_cycle_step(sl_pwm_instance_t *pwm)
{
    return TIMER_CaptureGet(pwm->timer, pwm->channel);
}

/**************************************************************************//**
 * @brief
 *   Get max steps for PWM waveform.
 *
 * @param[in] pwm
 *   PWM driver instance
 *
 * @return
 *   Return max steps of the pwm driver
 *****************************************************************************/
uint32_t sl_pwm_get_max_duty_cycle_step(sl_pwm_instance_t *pwm)
{
    return TIMER_TopGet(pwm->timer);
}
