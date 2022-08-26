#include "plat_sensor_table.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <logging/log.h>

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
#include "libutil.h"

LOG_MODULE_REGISTER(plat_sensor_table);

sensor_poll_time_cfg diff_poll_time_sensor_table[] = {
	// sensor_number, last_access_time
	{ SENSOR_NUM_VOL_BAT3V, 0 },
};

dimm_pmic_mapping_cfg dimm_pmic_map_table[] = {
	// dimm_sensor_num, mapping_pmic_sensor_num
	{ SENSOR_NUM_TEMP_DIMM_A, SENSOR_NUM_PWR_DIMMA_PMIC },
	{ SENSOR_NUM_TEMP_DIMM_C, SENSOR_NUM_PWR_DIMMC_PMIC },
	{ SENSOR_NUM_TEMP_DIMM_E, SENSOR_NUM_PWR_DIMME_PMIC },
	{ SENSOR_NUM_TEMP_DIMM_G, SENSOR_NUM_PWR_DIMMG_PMIC },
};

sensor_cfg plat_sensor_config[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, sample_count, cache, cache_status, mux_ADDRess, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

	// TMP
	{ SENSOR_NUM_TEMP_TMP75_IN, sensor_dev_tmp75, I2C_BUS2, TMP75_IN_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_TMP75_OUT, sensor_dev_tmp75, I2C_BUS2, TMP75_OUT_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_TMP75_IOM, sensor_dev_tmp75, I2C_BUS8, TMP75_IOM_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	// NVME, slave address need to be changed
	{ SENSOR_NUM_TEMP_SSD0, sensor_dev_nvme, I2C_BUS2, SSD0_ADDR, SSD0_OFFSET, post_access, 0,
	  0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },

	// PECI, slave address need to be changed
	{ SENSOR_NUM_TEMP_CPU, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR, PECI_TEMP_CPU,
	  post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_CPU_MARGIN, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CPU_MARGIN, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_CPU_TJMAX, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CPU_TJMAX, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_A, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL0_DIMM0, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_peci_dimm_read,
	  &dimm_pre_proc_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_C, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL2_DIMM0, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_peci_dimm_read,
	  &dimm_pre_proc_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_E, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL4_DIMM0, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_peci_dimm_read,
	  &dimm_pre_proc_args[2], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_G, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL6_DIMM0, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_peci_dimm_read,
	  &dimm_pre_proc_args[3], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_CPU, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR, PECI_PWR_CPU, post_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	// adc voltage (Sort in the ADC port number order)
	{ SENSOR_NUM_VOL_STBY12V, sensor_dev_ast_adc, ADC_PORT0, NONE, NONE, stby_access, 66, 10,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_DIMM12V, sensor_dev_ast_adc, ADC_PORT1, NONE, NONE, dc_access, 66, 10,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY3V, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE, stby_access, 2, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V05, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_BAT3V, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE, stby_access, 3, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_BAT3V, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_vol_bat3v_read, NULL, post_vol_bat3v_read, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_M2_3V3, sensor_dev_ast_adc, ADC_PORT5, NONE, NONE, dc_access, 2, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY5V, sensor_dev_ast_adc, ADC_PORT6, NONE, NONE, stby_access, 711, 200,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V8, sensor_dev_ast_adc, ADC_PORT7, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V2, sensor_dev_ast_adc, ADC_PORT8, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	// VR voltage
	{ SENSOR_NUM_VOL_PVCCD_HV, sensor_dev_isl69259, I2C_BUS10, PVCCD_HV_ADDR, VR_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCINFAON, sensor_dev_isl69259, I2C_BUS10, PVCCINFAON_ADDR, VR_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCFA_EHV, sensor_dev_isl69259, I2C_BUS10, PVCCFA_EHV_ADDR, VR_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCIN, sensor_dev_isl69259, I2C_BUS10, PVCCIN_ADDR, VR_VOL_CMD, vr_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCFA_EHV_FIVRA, sensor_dev_isl69259, I2C_BUS10, PVCCFA_EHV_FIVRA_ADDR,
	  VR_VOL_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },

	// VR current
	{ SENSOR_NUM_CUR_PVCCD_HV, sensor_dev_isl69259, I2C_BUS10, PVCCD_HV_ADDR, VR_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_IN_PVCCD_HV, sensor_dev_isl69259, I2C_BUS10, PVCCD_HV_ADDR, VR_CUR_IN_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCINFAON, sensor_dev_isl69259, I2C_BUS10, PVCCINFAON_ADDR, VR_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCFA_EHV, sensor_dev_isl69259, I2C_BUS10, PVCCFA_EHV_ADDR, VR_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCIN, sensor_dev_isl69259, I2C_BUS10, PVCCIN_ADDR, VR_CUR_CMD, vr_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCFA_EHV_FIVRA, sensor_dev_isl69259, I2C_BUS10, PVCCFA_EHV_FIVRA_ADDR,
	  VR_CUR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },

	// VR temperature
	{ SENSOR_NUM_TEMP_PVCCD_HV, sensor_dev_isl69259, I2C_BUS10, PVCCD_HV_ADDR, VR_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVCCINFAON, sensor_dev_isl69259, I2C_BUS10, PVCCINFAON_ADDR, VR_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVCCFA_EHV, sensor_dev_isl69259, I2C_BUS10, PVCCFA_EHV_ADDR, VR_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVCCIN, sensor_dev_isl69259, I2C_BUS10, PVCCIN_ADDR, VR_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVCCFA_EHV_FIVRA, sensor_dev_isl69259, I2C_BUS10, PVCCFA_EHV_FIVRA_ADDR,
	  VR_TEMP_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },

	// VR power
	{ SENSOR_NUM_PWR_PVCCD_HV, sensor_dev_isl69259, I2C_BUS10, PVCCD_HV_ADDR, VR_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCINFAON, sensor_dev_isl69259, I2C_BUS10, PVCCINFAON_ADDR, VR_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCFA_EHV, sensor_dev_isl69259, I2C_BUS10, PVCCFA_EHV_ADDR, VR_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCIN, sensor_dev_isl69259, I2C_BUS10, PVCCIN_ADDR, VR_PWR_CMD, vr_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCFA_EHV_FIVRA, sensor_dev_isl69259, I2C_BUS10, PVCCFA_EHV_FIVRA_ADDR,
	  VR_PWR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },

	// ME, slave address need to be changed
	{ SENSOR_NUM_TEMP_PCH, sensor_dev_pch, I2C_BUS3, PCH_ADDR, ME_SENSOR_NUM_TEMP_PCH,
	  me_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_DIMMA_PMIC, sensor_dev_pmic, I2C_BUS3, PCH_ADDR, NONE, me_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_pmic_read, &pmic_pre_read_args[0], NULL, NULL, &pmic_init_args[0] },
	{ SENSOR_NUM_PWR_DIMMC_PMIC, sensor_dev_pmic, I2C_BUS3, PCH_ADDR, NONE, me_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_pmic_read, &pmic_pre_read_args[1], NULL, NULL, &pmic_init_args[4] },
	{ SENSOR_NUM_PWR_DIMME_PMIC, sensor_dev_pmic, I2C_BUS3, PCH_ADDR, NONE, me_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_pmic_read, &pmic_pre_read_args[2], NULL, NULL, &pmic_init_args[8] },
	{ SENSOR_NUM_PWR_DIMMG_PMIC, sensor_dev_pmic, I2C_BUS3, PCH_ADDR, NONE, me_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_pmic_read, &pmic_pre_read_args[3], NULL, NULL, &pmic_init_args[12] },

	// INA230
	{ SENSOR_NUM_PWR_IOM_INA, sensor_dev_ina230, I2C_BUS8, INA230_ADDR, INA230_PWR_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina230_init_args[0] },
	{ SENSOR_NUM_CUR_IOM_INA, sensor_dev_ina230, I2C_BUS8, INA230_ADDR, INA230_CUR_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina230_init_args[0] },
	{ SENSOR_NUM_VOL_IOM_INA, sensor_dev_ina230, I2C_BUS8, INA230_ADDR, INA230_BUS_VOL_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ina230_init_args[0] },

	// HSC
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_mp5990, I2C_BUS5, MPS_MP5990_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &mp5990_init_args[0] },
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_mp5990, I2C_BUS5, MPS_MP5990_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_mp5990, I2C_BUS5, MPS_MP5990_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_mp5990, I2C_BUS5, MPS_MP5990_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
};

const int SENSOR_CONFIG_SIZE = ARRAY_SIZE(plat_sensor_config);

bool pal_is_time_to_poll(uint8_t sensor_num, int poll_time)
{
	int i = 0;
	int table_size = sizeof(diff_poll_time_sensor_table) / sizeof(sensor_poll_time_cfg);

	for (i = 0; i < table_size; i++) {
		if (sensor_num == diff_poll_time_sensor_table[i].sensor_num) {
			int64_t current_access_time = k_uptime_get();
			int64_t last_access_time = diff_poll_time_sensor_table[i].last_access_time;
			int64_t diff_time = (current_access_time - last_access_time) / 1000;
			if ((last_access_time != 0) && (diff_time < poll_time)) {
				return false;
			} else {
				diff_poll_time_sensor_table[i].last_access_time =
					current_access_time;
				return true;
			}
		}
	}

	LOG_WRN("[%s] can't find sensor 0x%x last access time", __func__, sensor_num);
	return true;
}

uint8_t get_hsc_pwr_reading(int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR)
	return get_sensor_reading(SENSOR_NUM_PWR_HSCIN, reading, GET_FROM_CACHE);
}

bool disable_dimm_pmic_sensor(uint8_t sensor_num)
{
	uint8_t table_size = ARRAY_SIZE(dimm_pmic_map_table);

	for (uint8_t index = 0; index < table_size; ++index) {
		if (sensor_num == dimm_pmic_map_table[index].dimm_sensor_num) {
			control_sensor_polling(dimm_pmic_map_table[index].dimm_sensor_num,
					       DISABLE_SENSOR_POLLING, SENSOR_NOT_PRESENT);
			control_sensor_polling(dimm_pmic_map_table[index].mapping_pmic_sensor_num,
					       DISABLE_SENSOR_POLLING, SENSOR_NOT_PRESENT);
			return true;
		}
	}

	LOG_WRN("[%s] input sensor 0x%x can't find in dimm pmic mapping table", __func__,
	       sensor_num);
	return false;
}
