#ifndef PCH_H
#define PCH_H

#include <stdbool.h>
#include <stdint.h>

bool pch_read(uint8_t sensor_num, float *reading);

#endif
