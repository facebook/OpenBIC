#include <stdio.h>
#include <string.h>
#include "sdr.h"
#include "sensor.h"
#include "sensor_def.h"
#include "hal_i2c.h"
#include "plat_i2c.h"
#include "plat_func.h"
#include "plat_hook.h"
#include "pal.h"

#define NONE 0

sensor_cfg plat_sensor_config[] = {
	/* number,                           type,            port,           address,                  offset,                     access check       arg0,   arg1,   cache,   cache_status */

	// temperature
	{ SENSOR_NUM_TEMP_TMP75_IN, sensor_dev_tmp75, i2c_bus1, tmp75_in_addr, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_TMP75_OUT, sensor_dev_tmp75, i2c_bus1, tmp75_out_addr, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	// adc
	{ SENSOR_NUM_VOL_P5V_STBY, sensor_dev_ast_adc, adc_port1, NONE, NONE, stby_access, 736, 200,
	  0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P12V_STBY, sensor_dev_ast_adc, adc_port0, NONE, NONE, stby_access, 178, 20,
	  0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P3V3_STBY, sensor_dev_ast_adc, adc_port2, NONE, NONE, stby_access, 487,
	  200, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P5V_USB, sensor_dev_ast_adc, adc_port7, NONE, NONE, stby_access, 736, 200,
	  0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P1V2_BIC_STBY, sensor_dev_ast_adc, adc_port5, NONE, NONE, stby_access, 1,
	  1, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_P1V0_STBY, sensor_dev_ast_adc, adc_port4, NONE, NONE, stby_access, 1, 1, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_CUR_P12V_FAN, sensor_dev_ast_adc, adc_port6, NONE, NONE, stby_access, 1, 1, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	// medusa board
	{ SENSOR_NUM_VOL_MEDUSA_12V_IN, sensor_dev_ltc4282, i2c_bus2, medusa_addr,
	  MEDUSA_VOL_OUT_OFFSET, stby_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_ltc4282_read,
	  &ltc4282_pre_read_args[0], NULL, NULL, &ltc4282_init_args[0] },
	{ SENSOR_NUM_VOL_MEDUSA_12V_OUT, sensor_dev_ltc4282, i2c_bus2, medusa_addr,
	  MEDUSA_VOL_IN_OFFSET, stby_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_ltc4282_read,
	  &ltc4282_pre_read_args[1], NULL, NULL, &ltc4282_init_args[0] },
	{ SENSOR_NUM_CUR_MEDUSA_IOUT, sensor_dev_ltc4282, i2c_bus2, medusa_addr, MEDUSA_CUR_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4282_init_args[0] },
	{ SENSOR_NUM_PWR_MEDUSA_12V, sensor_dev_ltc4282, i2c_bus2, medusa_addr, MEDUSA_PWR_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4282_init_args[0] },

	// HSC
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_adm1278, i2c_bus2, HSC_addr, HSC_TEMP_OFFSET, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_adm1278, i2c_bus2, HSC_addr, HSC_VOL_OFFSET, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_adm1278, i2c_bus2, HSC_addr, HSC_CUR_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_adm1278, i2c_bus2, HSC_addr, HSC_PWR_OFFSET, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_HSC_EIN, sensor_dev_adm1278, i2c_bus2, HSC_addr, HSC_EIN_EXT_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_HSC_PEAK_IOUT, sensor_dev_adm1278, i2c_bus2, HSC_addr, HSC_PEAK_IOUT_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_HSC_PEAK_PIN, sensor_dev_adm1278, i2c_bus2, HSC_addr, HSC_PEAK_PIN_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },

	// Fan
	{ SENSOR_NUM_FAN_BMC_TACH_0, type_fan, i2c_bus2, fan_addr, NONE, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_FAN_BMC_TACH_1, type_fan, i2c_bus2, fan_addr, NONE, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_FAN_BMC_TACH_2, type_fan, i2c_bus2, fan_addr, NONE, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_FAN_BMC_TACH_3, type_fan, i2c_bus2, fan_addr, NONE, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_FAN_BMC_TACH_4, type_fan, i2c_bus2, fan_addr, NONE, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_FAN_BMC_TACH_5, type_fan, i2c_bus2, fan_addr, NONE, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_FAN_BMC_TACH_6, type_fan, i2c_bus2, fan_addr, NONE, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_FAN_BMC_TACH_7, type_fan, i2c_bus2, fan_addr, NONE, stby_access, 0, 0, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

};

bool pal_load_sensor_config(void)
{
	memcpy(&sensor_config[0], &plat_sensor_config[0], sizeof(plat_sensor_config));
	return 1;
};
