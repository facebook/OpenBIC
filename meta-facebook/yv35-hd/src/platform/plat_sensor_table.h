#ifndef PLAT_SENSOR_TABLE_H
#define PLAT_SENSOR_TABLE_H

#include <stdint.h>

#define SENSOR_NUM_PWR_HSCIN 0x2C

uint8_t plat_get_config_size();
void load_sensor_config(void);

#endif
