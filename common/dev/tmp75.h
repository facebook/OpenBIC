#ifndef TMP75_H
#define TMP75_H

#include <stdbool.h>
#include <stdint.h>

bool tmp75_read(uint8_t sensor_num, float *reading);

#endif
