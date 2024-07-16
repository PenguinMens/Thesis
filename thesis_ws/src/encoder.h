// encoder_init.h

#ifndef ENCODER_INIT_H
#define ENCODER_INIT_H

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
// Function declaration


int init_PIO_encoder(uint PIN_AB, uint PIN_CD, uint encode1, uint encode2);
int32_t get_encoder_count_A();
int32_t get_encoder_count_B();
int32_t encoder_read_and_reset(uint encoder); // not working
int32_t encoder_read_and_reset_A();
int32_t encoder_read_and_reset_B();
#endif // ENCODER_INIT_H