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

SET_GPIO_VALUE_CFG pre_bat_3v = { A_P3V_BAT_SCALED_EN_R, GPIO_HIGH };
SET_GPIO_VALUE_CFG post_bat_3v = { A_P3V_BAT_SCALED_EN_R, GPIO_LOW };

sensor_poll_time_cfg diff_poll_time_sensor_table[] = {
	// sensor_number, last_access_time
	{ SENSOR_NUM_VOL_BAT3V, 0 },
};

sensor_cfg plat_sensor_config[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, sample_count, cache, cache_status, mux_ADDRess, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

	// temperature
	{ SENSOR_NUM_TEMP_TMP75_IN, sensor_dev_tmp75, I2C_BUS2, TMP75_IN_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL,
	  NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_TMP75_OUT, sensor_dev_tmp75, I2C_BUS2, TMP75_OUT_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL,
	  NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_TMP75_FIO, sensor_dev_tmp75, I2C_BUS2, TMP75_FIO_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL,
	  NULL, NULL, NULL, NULL },

	// NVME
	{ SENSOR_NUM_TEMP_SSD0, sensor_dev_nvme, I2C_BUS2, SSD0_ADDR, SSD0_OFFSET, post_access, 0,
	  0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, pre_nvme_read,
	  &mux_conf_addr_0xe2[1], NULL, NULL, NULL },

	// PECI
	{ SENSOR_NUM_TEMP_CPU, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR, PECI_TEMP_CPU,
	  post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL,
	  NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_CPU_MARGIN, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CPU_MARGIN, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_cpu_margin_read, NULL, NULL },
	{ SENSOR_NUM_TEMP_CPU_TJMAX, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CPU_TJMAX, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_A, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL0_DIMM0, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_C, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL2_DIMM0, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_D, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL3_DIMM0, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_E, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL4_DIMM0, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_G, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL6_DIMM0, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_H, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL7_DIMM0, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_CPU, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR, PECI_PWR_CPU, post_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL,
	  NULL, NULL },

	// adc voltage
	{ SENSOR_NUM_VOL_STBY12V, sensor_dev_ast_adc, ADC_PORT0, NONE, NONE, stby_access, 667, 100,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY3V, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE, stby_access, 2, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V05, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_BAT3V, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE, stby_access, 3, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_BAT3V, 0, SENSOR_INIT_STATUS, pre_vol_bat3v_read, NULL,
	  post_vol_bat3v_read, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY5V, sensor_dev_ast_adc, ADC_PORT9, NONE, NONE, stby_access, 711, 200,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_DIMM12V, sensor_dev_ast_adc, ADC_PORT11, NONE, NONE, dc_access, 667, 100,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V2, sensor_dev_ast_adc, ADC_PORT13, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_M2_3V3, sensor_dev_ast_adc, ADC_PORT14, NONE, NONE, dc_access, 2, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[0] },
	{ SENSOR_NUM_VOL_STBY1V8, sensor_dev_ast_adc, ADC_PORT15, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adc_asd_init_args[0] },

	// VR voltage
	{ SENSOR_NUM_VOL_PVCCD_HV, sensor_dev_isl69259, I2C_BUS5, PVCCD_HV_ADDR, VR_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69259_read, &isl69259_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCINFAON, sensor_dev_isl69259, I2C_BUS5, PVCCINFAON_ADDR, VR_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69259_read, &isl69259_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCFA_EHV, sensor_dev_isl69259, I2C_BUS5, PVCCFA_EHV_ADDR, VR_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69259_read, &isl69259_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCIN, sensor_dev_isl69259, I2C_BUS5, PVCCIN_ADDR, VR_VOL_CMD, vr_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, pre_isl69259_read,
	  &isl69259_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCFA_EHV_FIVRA, sensor_dev_isl69259, I2C_BUS5, PVCCFA_EHV_FIVRA_ADDR,
	  VR_VOL_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0,
	  SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[1], NULL, NULL, NULL },

	// VR current
	{ SENSOR_NUM_CUR_PVCCD_HV, sensor_dev_isl69259, I2C_BUS5, PVCCD_HV_ADDR, VR_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69259_read, &isl69259_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCINFAON, sensor_dev_isl69259, I2C_BUS5, PVCCINFAON_ADDR, VR_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69259_read, &isl69259_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCFA_EHV, sensor_dev_isl69259, I2C_BUS5, PVCCFA_EHV_ADDR, VR_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69259_read, &isl69259_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCIN, sensor_dev_isl69259, I2C_BUS5, PVCCIN_ADDR, VR_CUR_CMD, vr_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, pre_isl69259_read,
	  &isl69259_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCFA_EHV_FIVRA, sensor_dev_isl69259, I2C_BUS5, PVCCFA_EHV_FIVRA_ADDR,
	  VR_CUR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0,
	  SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[1], NULL, NULL, NULL },

	// VR temperature
	{ SENSOR_NUM_TEMP_PVCCD_HV, sensor_dev_isl69259, I2C_BUS5, PVCCD_HV_ADDR, VR_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69259_read, &isl69259_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVCCINFAON, sensor_dev_isl69259, I2C_BUS5, PVCCINFAON_ADDR, VR_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69259_read, &isl69259_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVCCFA_EHV, sensor_dev_isl69259, I2C_BUS5, PVCCFA_EHV_ADDR, VR_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69259_read, &isl69259_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVCCIN, sensor_dev_isl69259, I2C_BUS5, PVCCIN_ADDR, VR_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69259_read, &isl69259_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVCCFA_EHV_FIVRA, sensor_dev_isl69259, I2C_BUS5, PVCCFA_EHV_FIVRA_ADDR,
	  VR_TEMP_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0,
	  SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[1], NULL, NULL, NULL },

	// VR power
	{ SENSOR_NUM_PWR_PVCCD_HV, sensor_dev_isl69259, I2C_BUS5, PVCCD_HV_ADDR, VR_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69259_read, &isl69259_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCINFAON, sensor_dev_isl69259, I2C_BUS5, PVCCINFAON_ADDR, VR_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69259_read, &isl69259_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCFA_EHV, sensor_dev_isl69259, I2C_BUS5, PVCCFA_EHV_ADDR, VR_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS,
	  pre_isl69259_read, &isl69259_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCIN, sensor_dev_isl69259, I2C_BUS5, PVCCIN_ADDR, VR_PWR_CMD, vr_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, pre_isl69259_read,
	  &isl69259_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCFA_EHV_FIVRA, sensor_dev_isl69259, I2C_BUS5, PVCCFA_EHV_FIVRA_ADDR,
	  VR_PWR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0,
	  SENSOR_INIT_STATUS, pre_isl69259_read, &isl69259_pre_read_args[1], NULL, NULL, NULL },

	// ME
	{ SENSOR_NUM_TEMP_PCH, sensor_dev_pch, I2C_BUS3, PCH_ADDR, ME_SENSOR_NUM_TEMP_PCH,
	  me_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL,
	  NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_DIMMA_PMIC, sensor_dev_pmic, I2C_BUS3, PCH_ADDR, NONE, me_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, pre_pmic_read,
	  &pmic_pre_read_args[0], NULL, NULL, &pmic_init_args[0] },
	{ SENSOR_NUM_PWR_DIMMC_PMIC, sensor_dev_pmic, I2C_BUS3, PCH_ADDR, NONE, me_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, pre_pmic_read,
	  &pmic_pre_read_args[1], NULL, NULL, &pmic_init_args[1] },
	{ SENSOR_NUM_PWR_DIMMD_PMIC, sensor_dev_pmic, I2C_BUS3, PCH_ADDR, NONE, me_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, pre_pmic_read,
	  &pmic_pre_read_args[2], NULL, NULL, &pmic_init_args[2] },
	{ SENSOR_NUM_PWR_DIMME_PMIC, sensor_dev_pmic, I2C_BUS3, PCH_ADDR, NONE, me_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, pre_pmic_read,
	  &pmic_pre_read_args[3], NULL, NULL, &pmic_init_args[3] },
	{ SENSOR_NUM_PWR_DIMMG_PMIC, sensor_dev_pmic, I2C_BUS3, PCH_ADDR, NONE, me_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, pre_pmic_read,
	  &pmic_pre_read_args[4], NULL, NULL, &pmic_init_args[4] },
	{ SENSOR_NUM_PWR_DIMMH_PMIC, sensor_dev_pmic, I2C_BUS3, PCH_ADDR, NONE, me_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, pre_pmic_read,
	  &pmic_pre_read_args[5], NULL, NULL, &pmic_init_args[5] },
};

sensor_cfg mp5990_sensor_config_table[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, sample_count, cache, cache_status, mux_address, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_mp5990, I2C_BUS2, MPS_MP5990_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_mp5990, I2C_BUS2, MPS_MP5990_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL,
	  NULL, NULL, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_mp5990, I2C_BUS2, MPS_MP5990_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL,
	  NULL, NULL, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_mp5990, I2C_BUS2, MPS_MP5990_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL,
	  NULL, NULL, NULL, &mp5990_init_args[0] },
};

sensor_cfg adm1278_sensor_config_table[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, sample_count, cache, cache_status, mux_address, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_adm1278, I2C_BUS2, ADI_ADM1278_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_adm1278, I2C_BUS2, ADI_ADM1278_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL,
	  NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_adm1278, I2C_BUS2, ADI_ADM1278_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL,
	  NULL, post_adm1278_current_read, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_adm1278, I2C_BUS2, ADI_ADM1278_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL,
	  NULL, post_adm1278_power_read, NULL, &adm1278_init_args[0] },
};

sensor_cfg evt3_class1_adi_temperature_sensor_table[] = {
	{ SENSOR_NUM_TEMP_TMP75_OUT, sensor_dev_tmp431, I2C_BUS2, TMP431_ADDR,
	  TMP431_LOCAL_TEMPERATRUE, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_tmp431, I2C_BUS2, TMP431_ADDR, TMP431_REMOTE_TEMPERATRUE,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL,
	  NULL, NULL, NULL, NULL },
};

sensor_cfg DPV2_sensor_config_table[] = {
	{ SENSOR_NUM_VOL_DPV2_12VIN, sensor_dev_max16550a, I2C_BUS9, DPV2_16_ADDR, PMBUS_READ_VIN,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL,
	  NULL, NULL, NULL, &max16550a_init_args[0] },
	{ SENSOR_NUM_VOL_DPV2_12VOUT, sensor_dev_max16550a, I2C_BUS9, DPV2_16_ADDR, PMBUS_READ_VOUT,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL,
	  NULL, NULL, NULL, &max16550a_init_args[0] },
	{ SENSOR_NUM_CUR_DPV2OUT, sensor_dev_max16550a, I2C_BUS9, DPV2_16_ADDR, PMBUS_READ_IOUT,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL,
	  NULL, NULL, NULL, &max16550a_init_args[0] },
	{ SENSOR_NUM_TEMP_DPV2_EFUSE, sensor_dev_max16550a, I2C_BUS9, DPV2_16_ADDR,
	  PMBUS_READ_TEMPERATURE_1, dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &max16550a_init_args[0] },
	{ SENSOR_NUM_PWR_DPV2, sensor_dev_max16550a, I2C_BUS9, DPV2_16_ADDR, PMBUS_READ_PIN,
	  dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, 0, SENSOR_INIT_STATUS, NULL,
	  NULL, NULL, NULL, &max16550a_init_args[0] },
};

uint8_t plat_get_config_size()
{
	return ARRAY_SIZE(plat_sensor_config);
}

void load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, sizeof(plat_sensor_config));
	sensor_config_count = ARRAY_SIZE(plat_sensor_config);

	// Fix config table in different system/config
	pal_extend_sensor_config();
}

uint8_t pal_get_extend_sensor_config()
{
	uint8_t extend_sensor_config_size = 0;
	uint8_t hsc_module = get_hsc_module();
	switch (hsc_module) {
	case HSC_MODULE_ADM1278:
		extend_sensor_config_size += ARRAY_SIZE(adm1278_sensor_config_table);
		break;
	case HSC_MODULE_MP5990:
		extend_sensor_config_size += ARRAY_SIZE(mp5990_sensor_config_table);
		break;
	case HSC_MODULE_LTC4282:
	case HSC_MODULE_LTC4286:
	default:
		printf("[%s] not support on this hsc module, hsc module: 0x%x\n", __func__,
		       hsc_module);
		break;
	}

	// Fix sensor config table if 2ou card is present
	CARD_STATUS _2ou_status = get_2ou_status();
	if (_2ou_status.present) {
		// Add DPV2 config if DPV2_16 is present
		if ((_2ou_status.card_type & TYPE_2OU_DPV2_16) == TYPE_2OU_DPV2_16) {
			extend_sensor_config_size += ARRAY_SIZE(DPV2_sensor_config_table);
		}
	}
	return extend_sensor_config_size;
}

void check_vr_type(uint8_t index)
{
	uint8_t retry = 5;
	I2C_MSG msg;

	isl69259_pre_proc_arg *args = sensor_config[index].pre_sensor_read_args;
	memset(&msg, 0, sizeof(msg));
	msg.bus = sensor_config[index].port;
	msg.target_addr = sensor_config[index].target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = args->vr_page;
	if (i2c_master_write(&msg, retry)) {
		printf("Failed to switch to VR page %d\n", args->vr_page);
		return;
	}

	/* Get IC Device ID from VR chip
	 * - Command code: 0xAD
	 * - The response data 
	 *   byte-1: Block read count
	 *   byte-2: Device ID
	 * For the ISL69259 chip,
	 * the byte-1 of response data is 4 and the byte-2 to 5 is 49D28100h.
	 * For the TPS53689 chip,
	 * the byte-1 of response data is 6 and the byte-2 to 7 is 544953689000h.
	 * For the XDPE15284 chip,
	 * the byte-1 is returned as 2 and the byte-2 is 8Ah(XDPE15284).
	 */
	memset(&msg, 0, sizeof(msg));
	msg.bus = sensor_config[index].port;
	msg.target_addr = sensor_config[index].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 7;
	msg.data[0] = PMBUS_IC_DEVICE_ID;

	if (i2c_master_read(&msg, retry)) {
		printf("Failed to read VR IC_DEVICE_ID: register(0x%x)\n", PMBUS_IC_DEVICE_ID);
		return;
	}

	if ((msg.data[0] == 0x06) && (msg.data[1] == 0x54) && (msg.data[2] == 0x49) &&
	    (msg.data[3] == 0x53) && (msg.data[4] == 0x68) && (msg.data[5] == 0x90) &&
	    (msg.data[6] == 0x00)) {
		sensor_config[index].type = sensor_dev_tps53689;
	} else if ((msg.data[0] == 0x02) && (msg.data[2] == 0x8A)) {
		sensor_config[index].type = sensor_dev_xdpe15284;
	} else if ((msg.data[0] == 0x04) && (msg.data[1] == 0x00) && (msg.data[2] == 0x81) &&
		   (msg.data[3] == 0xD2) && (msg.data[4] == 0x49)) {
	} else {
		printf("Unknown VR type\n");
	}
}

void pal_extend_sensor_config()
{
	uint8_t sensor_count = 0;
	uint8_t hsc_module = get_hsc_module();

	/* Check the VR sensor type */
	sensor_count = ARRAY_SIZE(plat_sensor_config);
	for (uint8_t index = 0; index < sensor_count; index++) {
		if (sensor_config[index].type == sensor_dev_isl69259) {
			check_vr_type(index);
		}
	}

	CARD_STATUS _2ou_status = get_2ou_status();
	switch (hsc_module) {
	case HSC_MODULE_ADM1278:
		sensor_count = ARRAY_SIZE(adm1278_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(adm1278_sensor_config_table[index]);
		}
		break;
	case HSC_MODULE_MP5990:
		sensor_count = ARRAY_SIZE(mp5990_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			if (_2ou_status.present) {
				/* For the class type 1 and 2OU system,
        * set the IMON based total over current fault limit to 70A(0x0046),
        * set the gain for output current reporting to 0x01BF following the power team's experiment
        * and set GPIOA7(HSC_SET_EN_R) to high.
        */
				mp5990_sensor_config_table[index].init_args = &mp5990_init_args[1];
				gpio_set(HSC_SET_EN_R, GPIO_HIGH);
			} else {
				/* For the class type 1 and 2OU system,
        * set the IMON based total over current fault limit to 40A(0x0028),
        * set the gain for output current reporting to 0x0104 following the power team's experiment
        * and set GPIOA7(HSC_SET_EN_R) to low.
        */
				mp5990_sensor_config_table[index].init_args = &mp5990_init_args[0];
				gpio_set(HSC_SET_EN_R, GPIO_LOW);
			}
			add_sensor_config(mp5990_sensor_config_table[index]);
		}
		break;
	case HSC_MODULE_LTC4282:
	case HSC_MODULE_LTC4286:
	default:
		printf("[%s] not support on this hsc module, hsc module: 0x%x\n", __func__,
		       hsc_module);
		break;
	}
	if (get_board_revision() == SYS_BOARD_EVT3_HOTSWAP) {
		/* Replace the temperature sensors configuration including "HSC Temp" and "MB Outlet Temp."
    * For these two sensors, the reading values are read from TMP431 chip.data.num
    */
		sensor_count = ARRAY_SIZE(evt3_class1_adi_temperature_sensor_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(evt3_class1_adi_temperature_sensor_table[index]);
		}
	}

	/* Fix sensor table if 2ou card is present */
	if (_2ou_status.present) {
		// Add DPV2 sensor config if DPV2_16 is present
		if ((_2ou_status.card_type & TYPE_2OU_DPV2_16) == TYPE_2OU_DPV2_16) {
			sensor_count = ARRAY_SIZE(DPV2_sensor_config_table);
			for (int index = 0; index < sensor_count; index++) {
				add_sensor_config(DPV2_sensor_config_table[index]);
			}
		}
	}

	if (sensor_config_count != sdr_count) {
		printf("[%s] extend sensor SDR and config table not match, sdr size: 0x%x, sensor config size: 0x%x\n",
		       __func__, sdr_count, sensor_config_count);
	}
}

bool pal_is_time_to_poll(uint8_t sensor_num, int poll_time)
{
	int i = 0;
	int table_size = sizeof(diff_poll_time_sensor_table) / sizeof(sensor_poll_time_cfg);
	int64_t current_access_time = k_uptime_get();
	int64_t last_access_time = diff_poll_time_sensor_table[i].last_access_time;
	int64_t diff_time = (current_access_time - last_access_time) / 1000; // sec

	for (i = 0; i < table_size; i++) {
		if (sensor_num == diff_poll_time_sensor_table[i].sensor_num) {
			if ((last_access_time != 0) && (diff_time < poll_time)) {
				return false;
			} else {
				diff_poll_time_sensor_table[i].last_access_time =
					current_access_time;
				return true;
			}
		}
	}

	printf("[%s] can't find sensor 0x%x last accest time\n", __func__, sensor_num);
	return true;
}
