#include "plat_sensor_table.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "sensor.h"
#include "ast_adc.h"
#include "intel_peci.h"
#include "hal_gpio.h"
#include "plat_class.h"
#include "plat_gpio.h"
#include "plat_hook.h"
#include "plat_i2c.h"
#include "power_status.h"
#include "pmbus.h"
#include "tmp431.h"
#include "libutil.h"

sensor_cfg plat_sensor_config[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, cache, cache_status, mux_ADDRess, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

};

uint8_t load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, sizeof(plat_sensor_config));
	return ARRAY_SIZE(plat_sensor_config);
}
