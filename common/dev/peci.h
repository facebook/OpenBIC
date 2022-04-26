#ifndef PECI_H
#define PECI_H

#include <stdbool.h>
#include <stdint.h>

bool peci_sensor_read(uint8_t sensor_num, float *reading);

#endif
