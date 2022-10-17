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

void sl_pwm_set_duty_cycle_percent(sl_pwm_instance_t *pwm, float percent);

float sl_pwm_get_duty_cycle_percent(sl_pwm_instance_t *pwm);

void sl_pwm_set_duty_cycle_step(sl_pwm_instance_t *pwm, uint32_t step);

uint32_t sl_pwm_get_duty_cycle_step(sl_pwm_instance_t *pwm);

uint32_t sl_pwm_get_max_duty_cycle_step(sl_pwm_instance_t *pwm);

#endif /* COMMON_CUSTOM_SL_PWM_H_ */
