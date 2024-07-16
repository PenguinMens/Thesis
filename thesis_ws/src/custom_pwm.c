#include "custom_pwm.h"

// Function to set PWM frequency and duty cycle
// Parameters:
//   - slice_num: PWM slice number
//   - chan: PWM channel
//   - f: Desired frequency in Hz
//   - d: Desired duty cycle percentage (0-100)
// Returns:
//   - The calculated wrap value
uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, float d) {
    // Clock frequency (assuming 125 MHz)
    uint32_t clock = 125000000;
    // Calculate divider to achieve desired frequency
    uint32_t divider16 = clock / f / 4096 + (clock % (f * 4096) != 0);
    // Minimum divider value is 16
    if (divider16 / 16 == 0)
        divider16 = 16;
    // Calculate wrap value based on frequency and divider
    uint32_t wrap = clock * 16 / divider16 / f - 1;
    // Set clock divider for integer and fractional parts
    pwm_set_clkdiv_int_frac(slice_num, divider16 / 16, divider16 & 0xF);
    // Set wrap value for PWM slice
    pwm_set_wrap(slice_num, wrap);
    // Set channel level based on duty cycle    
    int duty =(int)((wrap * d )/100);
    pwm_set_chan_level(slice_num, chan,duty);
    // Return the calculated wrap value
    return wrap;
}
