#include "plat_sensor_table.h"

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
#include "pmbus.h"

SET_GPIO_VALUE_CFG pre_bat_3v = { A_P3V_BAT_SCALED_EN_R, GPIO_HIGH };
SET_GPIO_VALUE_CFG post_bat_3v = { A_P3V_BAT_SCALED_EN_R, GPIO_LOW };

sensor_cfg plat_sensor_config[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, cache, cache_status, mux_ADDRess, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

	// temperature
	{ SENSOR_NUM_TEMP_TMP75_IN, sensor_dev_tmp75, I2C_BUS2, TMP75_IN_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_TMP75_OUT, sensor_dev_tmp75, I2C_BUS2, TMP75_OUT_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_TMP75_FIO, sensor_dev_tmp75, I2C_BUS2, TMP75_FIO_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	// NVME
	{ SENSOR_NUM_TEMP_SSD0, sensor_dev_nvme, I2C_BUS2, SSD0_ADDR, SSD0_OFFSET, post_access, 0,
	  0, 0, SENSOR_INIT_STATUS, pre_nvme_read, &mux_conf_addr_0xe2[1], NULL, NULL, NULL },

	// PECI
	{ SENSOR_NUM_TEMP_CPU, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR, PECI_TEMP_CPU,
	  post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_CPU_MARGIN, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CPU_MARGIN, post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL,
	  post_cpu_margin_read, NULL, NULL },
	{ SENSOR_NUM_TEMP_CPU_TJMAX, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CPU_TJMAX, post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_DIMM_A, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL0_DIMM0, post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_C, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL2_DIMM0, post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_D, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL3_DIMM0, post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_E, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL4_DIMM0, post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_G, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL6_DIMM0, post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_H, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL7_DIMM0, post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL,
	  NULL, NULL },
	{ SENSOR_NUM_PWR_CPU, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR, PECI_PWR_CPU, post_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	// adc voltage
	{ SENSOR_NUM_VOL_STBY12V, sensor_dev_ast_adc, ADC_PORT0, NONE, NONE, stby_access, 667, 100,
	  0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY3V, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE, stby_access, 2, 1, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V05, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE, stby_access, 1, 1, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_BAT3V, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE, stby_access, 3, 1, 0,
	  SENSOR_INIT_STATUS, pre_vol_bat3v_read, NULL, post_vol_bat3v_read, NULL,
	  &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY5V, sensor_dev_ast_adc, ADC_PORT9, NONE, NONE, stby_access, 711, 200,
	  0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_DIMM12V, sensor_dev_ast_adc, ADC_PORT11, NONE, NONE, dc_access, 667, 100,
	  0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V2, sensor_dev_ast_adc, ADC_PORT13, NONE, NONE, stby_access, 1, 1, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_M2_3V3, sensor_dev_ast_adc, ADC_PORT14, NONE, NONE, dc_access, 2, 1, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V8, sensor_dev_ast_adc, ADC_PORT15, NONE, NONE, stby_access, 1, 1, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	// VR voltage
	{ SENSOR_NUM_VOL_PVCCD_HV, sensor_dev_isl69259, I2C_BUS5, PVCCD_HV_ADDR, VR_VOL_CMD,
	  dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCINFAON, sensor_dev_isl69259, I2C_BUS5, PVCCINFAON_ADDR, VR_VOL_CMD,
	  dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCFA_EHV, sensor_dev_isl69259, I2C_BUS5, PVCCFA_EHV_ADDR, VR_VOL_CMD,
	  dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[1],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCIN, sensor_dev_isl69259, I2C_BUS5, PVCCIN_ADDR, VR_VOL_CMD, dc_access,
	  0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0], NULL, NULL,
	  NULL },
	{ SENSOR_NUM_VOL_PVCCFA_EHV_FIVRA, sensor_dev_isl69259, I2C_BUS5, PVCCFA_EHV_FIVRA_ADDR,
	  VR_VOL_CMD, dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read,
	  &isl69259_pre_read_args[1], NULL, NULL, NULL },

	// VR current
	{ SENSOR_NUM_CUR_PVCCD_HV, sensor_dev_isl69259, I2C_BUS5, PVCCD_HV_ADDR, VR_CUR_CMD,
	  dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCINFAON, sensor_dev_isl69259, I2C_BUS5, PVCCINFAON_ADDR, VR_CUR_CMD,
	  dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCFA_EHV, sensor_dev_isl69259, I2C_BUS5, PVCCFA_EHV_ADDR, VR_CUR_CMD,
	  dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[1],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCIN, sensor_dev_isl69259, I2C_BUS5, PVCCIN_ADDR, VR_CUR_CMD, dc_access,
	  0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0], NULL, NULL,
	  NULL },
	{ SENSOR_NUM_CUR_PVCCFA_EHV_FIVRA, sensor_dev_isl69259, I2C_BUS5, PVCCFA_EHV_FIVRA_ADDR,
	  VR_CUR_CMD, dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read,
	  &isl69259_pre_read_args[1], NULL, NULL, NULL },

	// VR temperature
	{ SENSOR_NUM_TEMP_PVCCD_HV, sensor_dev_isl69259, I2C_BUS5, PVCCD_HV_ADDR, VR_TEMP_CMD,
	  dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVCCINFAON, sensor_dev_isl69259, I2C_BUS5, PVCCINFAON_ADDR, VR_TEMP_CMD,
	  dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVCCFA_EHV, sensor_dev_isl69259, I2C_BUS5, PVCCFA_EHV_ADDR, VR_TEMP_CMD,
	  dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[1],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVCCIN, sensor_dev_isl69259, I2C_BUS5, PVCCIN_ADDR, VR_TEMP_CMD,
	  dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVCCFA_EHV_FIVRA, sensor_dev_isl69259, I2C_BUS5, PVCCFA_EHV_FIVRA_ADDR,
	  VR_TEMP_CMD, dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read,
	  &isl69259_pre_read_args[1], NULL, NULL, NULL },

	// VR power
	{ SENSOR_NUM_PWR_PVCCD_HV, sensor_dev_isl69259, I2C_BUS5, PVCCD_HV_ADDR, VR_PWR_CMD,
	  dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCINFAON, sensor_dev_isl69259, I2C_BUS5, PVCCINFAON_ADDR, VR_PWR_CMD,
	  dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCFA_EHV, sensor_dev_isl69259, I2C_BUS5, PVCCFA_EHV_ADDR, VR_PWR_CMD,
	  dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[1],
	  NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCIN, sensor_dev_isl69259, I2C_BUS5, PVCCIN_ADDR, VR_PWR_CMD, dc_access,
	  0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[0], NULL, NULL,
	  NULL },
	{ SENSOR_NUM_PWR_PVCCFA_EHV_FIVRA, sensor_dev_isl69259, I2C_BUS5, PVCCFA_EHV_FIVRA_ADDR,
	  VR_PWR_CMD, dc_access, 0, 0, 0, SENSOR_INIT_STATUS, pre_isl69259_read,
	  &isl69259_pre_read_args[1], NULL, NULL, NULL },

	// ME
	{ SENSOR_NUM_TEMP_PCH, sensor_dev_pch, I2C_BUS3, PCH_ADDR, ME_SENSOR_NUM_TEMP_PCH,
	  post_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	// HSC
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_adm1278, I2C_BUS2, HSC_ADDR, PMBUS_READ_TEMPERATURE_1,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_adm1278, I2C_BUS2, HSC_ADDR, PMBUS_READ_VIN, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_adm1278, I2C_BUS2, HSC_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_adm1278, I2C_BUS2, HSC_ADDR, PMBUS_READ_PIN, stby_access,
	  0, 0, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
};

uint8_t load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, sizeof(plat_sensor_config));
	return sizeof(plat_sensor_config) / sizeof(sensor_cfg);
}

void pal_fix_sensor_config()
{
	if (sensor_config_num != SDR_NUM) {
		printf("fix sensor SDR and config table not match\n");
	}
}
