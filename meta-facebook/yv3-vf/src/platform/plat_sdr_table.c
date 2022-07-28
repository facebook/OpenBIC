#include "plat_sdr_table.h"

#include <stdio.h>
#include <string.h>

#include "sdr.h"
#include "plat_ipmb.h"
#include "plat_sensor_table.h"

#define HSC_SENSOR 0

SDR_Full_sensor plat_sdr_table[] = {};

uint8_t plat_get_sdr_size()
{
	return ARRAY_SIZE(plat_sdr_table);
}

void load_sdr_table(void)
{
	memcpy(full_sdr_table, plat_sdr_table, sizeof(plat_sdr_table));
	sdr_count = ARRAY_SIZE(plat_sdr_table);
};
