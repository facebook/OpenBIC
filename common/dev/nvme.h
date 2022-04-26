#ifndef NVME_H
#define NVME_H

#include <stdbool.h>
#include <stdint.h>

bool nvme_read(uint8_t sensor_num, float *reading);

#endif
