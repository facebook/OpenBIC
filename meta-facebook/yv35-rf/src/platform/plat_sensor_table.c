#include <stdio.h>
#include <string.h>
#include "ast_adc.h"
#include "sensor.h"
#include "plat_hook.h"
#include "plat_i2c.h"
#include "plat_sensor_table.h"

sensor_cfg plat_sensor_config[] = {
	/* number,                  type,       port,      address,      offset,
     access check arg0, arg1, cache, cache_status, mux_ADDRess, mux_offset,
     pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

	// temperature
	{ SENSOR_NUM_TEMP_TMP75, sensor_dev_tmp75, I2C_BUS3, TMP75_MB_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_CXL_CNTR, sensor_dev_tmp75, I2C_BUS2, TMP75_ASIC_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  NULL },

	// ADC
	{ SENSOR_NUM_VOL_STBY5V, sensor_dev_ast_adc, ADC_PORT9, NONE, NONE, stby_access, 711, 200,
	  SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V2, sensor_dev_ast_adc, ADC_PORT6, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_ASIC_1V8, sensor_dev_ast_adc, ADC_PORT7, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_PVPP_AB, sensor_dev_ast_adc, ADC_PORT13, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_PVPP_CD, sensor_dev_ast_adc, ADC_PORT12, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_PVTT_AB, sensor_dev_ast_adc, ADC_PORT11, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_PVTT_CD, sensor_dev_ast_adc, ADC_PORT10, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[0] },

	// INA233
	{ SENSOR_NUM_VOL_STBY12V, sensor_dev_ina233, I2C_BUS1, INA233_12V_ADDR, SMBUS_VOL_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, pre_ina233_read, NULL,
	  NULL, NULL, &ina233_init_args[0] },
	{ SENSOR_NUM_VOL_STBY3V3, sensor_dev_ina233, I2C_BUS1, INA233_3V3_ADDR, SMBUS_VOL_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, pre_ina233_read, NULL,
	  NULL, NULL, &ina233_init_args[1] },
	{ SENSOR_NUM_CUR_STBY12V, sensor_dev_ina233, I2C_BUS1, INA233_12V_ADDR, SMBUS_CUR_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, pre_ina233_read, NULL,
	  NULL, NULL, &ina233_init_args[0] },
	{ SENSOR_NUM_CUR_STBY3V3, sensor_dev_ina233, I2C_BUS1, INA233_3V3_ADDR, SMBUS_CUR_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, pre_ina233_read, NULL,
	  NULL, NULL, &ina233_init_args[1] },
	{ SENSOR_NUM_PWR_STBY12V, sensor_dev_ina233, I2C_BUS1, INA233_12V_ADDR, SMBUS_PWR_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, pre_ina233_read, NULL,
	  NULL, NULL, &ina233_init_args[0] },
	{ SENSOR_NUM_PWR_STBY3V3, sensor_dev_ina233, I2C_BUS1, INA233_3V3_ADDR, SMBUS_PWR_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, pre_ina233_read, NULL,
	  NULL, NULL, &ina233_init_args[1] },

	// VR temperature
	{ SENSOR_NUM_TEMP_VR0V9A, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_A0V9_ADDR,
	  SMBUS_TEMP_CMD, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_VR0V8A, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_A0V8_ADDR,
	  SMBUS_TEMP_CMD, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_VR0V8D, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_D0V8_ADDR,
	  SMBUS_TEMP_CMD, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_VRVDDQAB, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_VDDQAB_ADDR,
	  SMBUS_TEMP_CMD, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_VRVDDQCD, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_VDDQCD_ADDR,
	  SMBUS_TEMP_CMD, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },

	// VR Voltage
	{ SENSOR_NUM_VOL_VR0V9A, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_A0V9_ADDR, SMBUS_VOL_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_VR0V8A, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_A0V8_ADDR, SMBUS_VOL_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_VR0V8D, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_D0V8_ADDR, SMBUS_VOL_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_VRVDDQAB, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_VDDQAB_ADDR,
	  SMBUS_VOL_CMD, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_VRVDDQCD, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_VDDQCD_ADDR,
	  SMBUS_VOL_CMD, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },

	// VR Current
	{ SENSOR_NUM_CUR_VR0V9A, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_A0V9_ADDR, SMBUS_CUR_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_VR0V8A, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_A0V8_ADDR, SMBUS_CUR_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_VR0V8D, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_D0V8_ADDR, SMBUS_CUR_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_VRVDDQAB, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_VDDQAB_ADDR,
	  SMBUS_CUR_CMD, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_VRVDDQCD, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_VDDQCD_ADDR,
	  SMBUS_CUR_CMD, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },

	// VR Power
	{ SENSOR_NUM_PWR_VR0V9A, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_A0V9_ADDR, SMBUS_PWR_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_VR0V8A, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_A0V8_ADDR, SMBUS_PWR_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_VR0V8D, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_D0V8_ADDR, SMBUS_PWR_CMD,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS, pre_isl69254iraz_t_read,
	  &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_VRVDDQAB, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_VDDQAB_ADDR,
	  SMBUS_PWR_CMD, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_VRVDDQCD, sensor_dev_isl69254iraz_t, I2C_BUS10, VR_VDDQAB_ADDR,
	  SMBUS_PWR_CMD, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69254iraz_t_read, &isl69254iraz_t_pre_read_args[0], NULL, NULL, NULL },
};

uint8_t load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, sizeof(plat_sensor_config));
	return ARRAY_SIZE(plat_sensor_config);
}
