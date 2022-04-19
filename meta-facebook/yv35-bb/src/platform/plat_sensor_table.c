#include "plat_sensor_table.h"

#include <stdio.h>
#include <string.h>

#include "ast_adc.h"
#include "sensor.h"
#include "plat_hook.h"
#include "plat_i2c.h"
#include "plat_def.h"

#define CONFIG_ISL69260 false
bool stby_access(uint8_t sensor_number);

sensor_cfg plat_sensor_config[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, cache, cache_status, mux_address, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

	// temperature
	{ SENSOR_NUM_TEMP_TMP75_IN, sensor_dev_tmp75, I2C_BUS1, TMP75_IN_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_TMP75_OUT, sensor_dev_tmp75, I2C_BUS1, TMP75_OUT_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	// adc
	{ SENSOR_NUM_VOL_P5V_STBY, sensor_dev_ast_adc, ADC_PORT1, NONE, NONE, stby_access, 736, 200,
	  0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P12V_STBY, sensor_dev_ast_adc, ADC_PORT0, NONE, NONE, stby_access, 178, 20,
	  0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P3V3_STBY, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE, stby_access, 487,
	  200, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P5V_USB, sensor_dev_ast_adc, ADC_PORT7, NONE, NONE, stby_access, 736, 200,
	  0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P1V2_BIC_STBY, sensor_dev_ast_adc, ADC_PORT5, NONE, NONE, stby_access, 1,
	  1, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P1V0_STBY, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE, stby_access, 1, 1, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_CUR_P12V_FAN, sensor_dev_ast_adc, ADC_PORT6, NONE, NONE, stby_access, 1, 1, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	// medusa board
	{ SENSOR_NUM_VOL_MEDUSA_12V_IN, sensor_dev_ltc4282, I2C_BUS2, MEDUSA_ADDR,
	  MEDUSA_VOL_OUT_OFFSET, stby_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_ltc4282_read,
	  &ltc4282_pre_read_args[0], NULL, NULL, &ltc4282_init_args[0] },
	{ SENSOR_NUM_VOL_MEDUSA_12V_OUT, sensor_dev_ltc4282, I2C_BUS2, MEDUSA_ADDR,
	  MEDUSA_VOL_IN_OFFSET, stby_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_ltc4282_read,
	  &ltc4282_pre_read_args[1], NULL, NULL, &ltc4282_init_args[0] },
	{ SENSOR_NUM_CUR_MEDUSA_IOUT, sensor_dev_ltc4282, I2C_BUS2, MEDUSA_ADDR, MEDUSA_CUR_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4282_init_args[0] },
	{ SENSOR_NUM_PWR_MEDUSA_12V, sensor_dev_ltc4282, I2C_BUS2, MEDUSA_ADDR, MEDUSA_PWR_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4282_init_args[0] },

	// HSC
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_adm1278, I2C_BUS2, HSC_ADDR, HSC_TEMP_OFFSET, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_adm1278, I2C_BUS2, HSC_ADDR, HSC_VOL_OFFSET, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_adm1278, I2C_BUS2, HSC_ADDR, HSC_CUR_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_adm1278, I2C_BUS2, HSC_ADDR, HSC_PWR_OFFSET, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },

	// Fan
	{ SENSOR_NUM_DUAL_FAN_BMC_TACH_0, sensor_dev_ast_fan, I2C_BUS2, FAN_ADDR, NONE, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_DUAL_FAN_BMC_TACH_1, sensor_dev_ast_fan, I2C_BUS2, FAN_ADDR, NONE, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_DUAL_FAN_BMC_TACH_2, sensor_dev_ast_fan, I2C_BUS2, FAN_ADDR, NONE, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_DUAL_FAN_BMC_TACH_3, sensor_dev_ast_fan, I2C_BUS2, FAN_ADDR, NONE, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_DUAL_FAN_BMC_TACH_4, sensor_dev_ast_fan, I2C_BUS2, FAN_ADDR, NONE, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_DUAL_FAN_BMC_TACH_5, sensor_dev_ast_fan, I2C_BUS2, FAN_ADDR, NONE, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_DUAL_FAN_BMC_TACH_6, sensor_dev_ast_fan, I2C_BUS2, FAN_ADDR, NONE, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_DUAL_FAN_BMC_TACH_7, sensor_dev_ast_fan, I2C_BUS2, FAN_ADDR, NONE, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_SINGLE_FAN_BMC_TACH_0, sensor_dev_ast_fan, I2C_BUS2, FAN_ADDR, NONE,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_SINGLE_FAN_BMC_TACH_1, sensor_dev_ast_fan, I2C_BUS2, FAN_ADDR, NONE,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_SINGLE_FAN_BMC_TACH_2, sensor_dev_ast_fan, I2C_BUS2, FAN_ADDR, NONE,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_SINGLE_FAN_BMC_TACH_3, sensor_dev_ast_fan, I2C_BUS2, FAN_ADDR, NONE,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
};

uint8_t load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, sizeof(plat_sensor_config));
	return ARRAY_SIZE(plat_sensor_config);
}
