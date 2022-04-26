#include "plat_sensor_table.h"

#include <stdio.h>
#include <string.h>

#include "sensor.h"
#include "adc.h"
#include "plat_i2c.h"
#include "plat_def.h"

#define CONFIG_ISL69260 false
bool stby_access(uint8_t snr_num);

snr_cfg plat_sensor_config[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, cache, cache_status, mux_address, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

	// temperature
	{ SENSOR_NUM_TEMP_TMP75_IN, TYPE_TMP75, I2C_BUS1, TMP75_IN_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS },
	{ SENSOR_NUM_TEMP_TMP75_OUT, TYPE_TMP75, I2C_BUS1, TMP75_OUT_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS },

	// adc
	{ SENSOR_NUM_VOL_P5V_STBY, TYPE_ADC, ADC_PORT1, NULL, NULL, stby_access, 736, 200, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_VOL_P12V_STBY, TYPE_ADC, ADC_PORT0, NULL, NULL, stby_access, 178, 20, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_VOL_P3V3_STBY, TYPE_ADC, ADC_PORT2, NULL, NULL, stby_access, 487, 200, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_VOL_P5V_USB, TYPE_ADC, ADC_PORT7, NULL, NULL, stby_access, 736, 200, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_VOL_P1V2_BIC_STBY, TYPE_ADC, ADC_PORT5, NULL, NULL, stby_access, 1, 1, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_VOL_P1V0_STBY, TYPE_ADC, ADC_PORT4, NULL, NULL, stby_access, 1, 1, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_CUR_P12V_FAN, TYPE_ADC, ADC_PORT6, NULL, NULL, stby_access, 1, 1, 0,
	  SENSOR_INIT_STATUS },

	// medusa board
	{ SENSOR_NUM_VOL_MEDUSA_12V_IN, TYPE_MEDUSA, I2C_BUS2, MEDUSA_ADDR, MEDUSA_VOL_OUT_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS },
	{ SENSOR_NUM_VOL_MEDUSA_12V_OUT, TYPE_MEDUSA, I2C_BUS2, MEDUSA_ADDR, MEDUSA_VOL_IN_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS },
	{ SENSOR_NUM_CUR_MEDUSA_IOUT, TYPE_MEDUSA, I2C_BUS2, MEDUSA_ADDR, MEDUSA_CUR_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS },
	{ SENSOR_NUM_PWR_MEDUSA_12V, TYPE_MEDUSA, I2C_BUS2, MEDUSA_ADDR, MEDUSA_PWR_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS },

	// HSC
	{ SENSOR_NUM_TEMP_HSC, TYPE_HSC, I2C_BUS2, HSC_ADDR, HSC_TEMP_OFFSET, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_VOL_HSCIN, TYPE_HSC, I2C_BUS2, HSC_ADDR, HSC_VOL_OFFSET, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_CUR_HSCOUT, TYPE_HSC, I2C_BUS2, HSC_ADDR, HSC_CUR_OFFSET, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_PWR_HSCIN, TYPE_HSC, I2C_BUS2, HSC_ADDR, HSC_PWR_OFFSET, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS },

	// Fan
	{ SENSOR_NUM_DUAL_FAN_BMC_TACH_0, TYPE_FAN, I2C_BUS2, FAN_ADDR, NULL, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_DUAL_FAN_BMC_TACH_1, TYPE_FAN, I2C_BUS2, FAN_ADDR, NULL, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_DUAL_FAN_BMC_TACH_2, TYPE_FAN, I2C_BUS2, FAN_ADDR, NULL, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_DUAL_FAN_BMC_TACH_3, TYPE_FAN, I2C_BUS2, FAN_ADDR, NULL, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_DUAL_FAN_BMC_TACH_4, TYPE_FAN, I2C_BUS2, FAN_ADDR, NULL, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_DUAL_FAN_BMC_TACH_5, TYPE_FAN, I2C_BUS2, FAN_ADDR, NULL, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_DUAL_FAN_BMC_TACH_6, TYPE_FAN, I2C_BUS2, FAN_ADDR, NULL, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_DUAL_FAN_BMC_TACH_7, TYPE_FAN, I2C_BUS2, FAN_ADDR, NULL, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS },
	{ SENSOR_NUM_SINGLE_FAN_BMC_TACH_0, TYPE_FAN, I2C_BUS2, FAN_ADDR, NULL, stby_access, 0, 0,
	  0, SENSOR_INIT_STATUS },
	{ SENSOR_NUM_SINGLE_FAN_BMC_TACH_1, TYPE_FAN, I2C_BUS2, FAN_ADDR, NULL, stby_access, 0, 0,
	  0, SENSOR_INIT_STATUS },
	{ SENSOR_NUM_SINGLE_FAN_BMC_TACH_2, TYPE_FAN, I2C_BUS2, FAN_ADDR, NULL, stby_access, 0, 0,
	  0, SENSOR_INIT_STATUS },
	{ SENSOR_NUM_SINGLE_FAN_BMC_TACH_3, TYPE_FAN, I2C_BUS2, FAN_ADDR, NULL, stby_access, 0, 0,
	  0, SENSOR_INIT_STATUS },
};

uint8_t load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, sizeof(plat_sensor_config));
	return sizeof(plat_sensor_config) / sizeof(snr_cfg);
}
