#ifndef ISL69260_H
#define ISL69260_H

#include <stdbool.h>
#include <stdint.h>

struct isl69260_page_data {
	uint8_t data0;
	uint8_t data1;
};

extern struct isl69260_page_data isl69260_page_data_args[];

int isl69260_switch_page(uint8_t sensor_num, void *args);
bool isl69260_read(uint8_t sensor_num, float *reading);

#endif
