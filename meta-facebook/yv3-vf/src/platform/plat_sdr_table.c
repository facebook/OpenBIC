#include "plat_sdr_table.h"

#include <stdio.h>
#include <string.h>

#include "sdr.h"
#include "plat_ipmb.h"
#include "plat_sensor_table.h"

#define HSC_SENSOR 0

SDR_Full_sensor plat_sdr_table[] = {};

const int SDR_TABLE_SIZE = ARRAY_SIZE(plat_sdr_table);
