// custom_pwm.h

#ifndef CUSTOM_PWM_H
#define CUSTOM_PWM_H

#include "hardware/pwm.h"
#include "pico/stdlib.h"


// Function declaration
uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, float d);

#endif // CUSTOM_PWM_H