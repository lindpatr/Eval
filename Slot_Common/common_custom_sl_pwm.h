/*
 * common_custom_sl_pwm.h
 *
 *  Created on: 17 oct. 2022
 *      Author: BEL-LinPat
 */

#ifndef COMMON_CUSTOM_SL_PWM_H_
#define COMMON_CUSTOM_SL_PWM_H_

#include <stdint.h>
#include "sl_pwm.h"

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
void sl_pwm_set_duty_cycle_percent(sl_pwm_instance_t *pwm, float percent);

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
float sl_pwm_get_duty_cycle_percent(sl_pwm_instance_t *pwm);

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
void sl_pwm_set_duty_cycle_step(sl_pwm_instance_t *pwm, uint32_t step);

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
uint32_t sl_pwm_get_duty_cycle_step(sl_pwm_instance_t *pwm);

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
uint32_t sl_pwm_get_max_duty_cycle_step(sl_pwm_instance_t *pwm);

#endif /* COMMON_CUSTOM_SL_PWM_H_ */
