#ifndef ADM1278_H
#define ADM1278_H

#include <stdbool.h>
#include <stdint.h>

bool adm1278_init();
bool adm1278_read(uint8_t sensor_num, float *reading);

#endif
