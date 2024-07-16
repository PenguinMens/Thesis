// utils.h

#ifndef UTILS_H
#define UTILS_H
#include "pico/stdlib.h"
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
float fmap(float val, float in_min, float in_max, float out_min, float out_max);

#endif // UTILS_H