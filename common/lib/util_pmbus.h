#ifndef UTIL_PMBUS_H
#define UTIL_PMBUS_H

float slinear11_to_float(uint16_t);
bool get_exponent_from_vout_mode(uint8_t, float *);

#endif
