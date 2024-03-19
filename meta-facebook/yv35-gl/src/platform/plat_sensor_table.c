/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <string.h>

#include "ast_adc.h"
#include "sensor.h"
#include "pmbus.h"
#include "intel_peci.h"
#include "hal_gpio.h"

#include "plat_class.h"
#include "plat_gpio.h"
#include "plat_i2c.h"
#include "plat_i3c.h"
#include "plat_dimm.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(plat_sensor_table);

dimm_pmic_mapping_cfg dimm_pmic_map_table[] = {
	// dimm_sensor_num, mapping_pmic_sensor_num
	{ SENSOR_NUM_MB_DIMMA0_TEMP_C, SENSOR_NUM_MB_VR_DIMMA0_PMIC_PWR_W },
	{ SENSOR_NUM_MB_DIMMA1_TEMP_C, SENSOR_NUM_MB_VR_DIMMA1_PMIC_PWR_W },
	{ SENSOR_NUM_MB_DIMMA2_TEMP_C, SENSOR_NUM_MB_VR_DIMMA2_PMIC_PWR_W },
	{ SENSOR_NUM_MB_DIMMA3_TEMP_C, SENSOR_NUM_MB_VR_DIMMA3_PMIC_PWR_W },
	{ SENSOR_NUM_MB_DIMMA4_TEMP_C, SENSOR_NUM_MB_VR_DIMMA4_PMIC_PWR_W },
	{ SENSOR_NUM_MB_DIMMA5_TEMP_C, SENSOR_NUM_MB_VR_DIMMA5_PMIC_PWR_W },
	{ SENSOR_NUM_MB_DIMMA6_TEMP_C, SENSOR_NUM_MB_VR_DIMMA6_PMIC_PWR_W },
	{ SENSOR_NUM_MB_DIMMA7_TEMP_C, SENSOR_NUM_MB_VR_DIMMA7_PMIC_PWR_W },
};

sensor_cfg plat_sensor_config[] = {
	/*  number,
        type,
        port,
        address,
        offset,
        access check,
        arg0,
        arg1,
        sample_count,
        poll time,
        enable polling,
        cache,
        cache_status,
        pre_sensor_read_fn,
        pre_sensor_read_args,
        post_sensor_read_fn,
        post_sensor_read_args,
        initial args
    */
	//Temp
	{ SENSOR_NUM_MB_INLET_TEMP_C, sensor_dev_tmp75, I2C_BUS2, INLET_TEP75_ADDR,
	  TMP75_TEMP_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_MB_OUTLET_TEMP_C, sensor_dev_tmp75, I2C_BUS2, OUTLET_TEP75_ADDR,
	  TMP75_TEMP_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_FIO_FRONT_TEMP_C, sensor_dev_tmp75, I2C_BUS2, FIO_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	{ SENSOR_NUM_MB_SOC_CPU_TEMP_C, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR, PECI_TEMP_CPU,
	  post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	{ SENSOR_NUM_MB_DIMMA0_TEMP_C, sensor_dev_i3c_dimm, I3C_BUS4, DIMM_SPD_A0_A4_ADDR,
	  DIMM_SPD_TEMP, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_dimm_i3c_read, NULL,
	  post_intel_dimm_i3c_read, &dimm_post_proc_args[0], NULL },
	{ SENSOR_NUM_MB_DIMMA1_TEMP_C, sensor_dev_i3c_dimm, I3C_BUS4, DIMM_SPD_A1_A5_ADDR,
	  DIMM_SPD_TEMP, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_dimm_i3c_read, NULL,
	  post_intel_dimm_i3c_read, &dimm_post_proc_args[1], NULL },
	{ SENSOR_NUM_MB_DIMMA2_TEMP_C, sensor_dev_i3c_dimm, I3C_BUS4, DIMM_SPD_A2_A6_ADDR,
	  DIMM_SPD_TEMP, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_dimm_i3c_read, NULL,
	  post_intel_dimm_i3c_read, &dimm_post_proc_args[2], NULL },
	{ SENSOR_NUM_MB_DIMMA3_TEMP_C, sensor_dev_i3c_dimm, I3C_BUS4, DIMM_SPD_A3_A7_ADDR,
	  DIMM_SPD_TEMP, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_dimm_i3c_read, NULL,
	  post_intel_dimm_i3c_read, &dimm_post_proc_args[3], NULL },
	{ SENSOR_NUM_MB_DIMMA4_TEMP_C, sensor_dev_i3c_dimm, I3C_BUS4, DIMM_SPD_A0_A4_ADDR,
	  DIMM_SPD_TEMP, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_dimm_i3c_read, NULL,
	  post_intel_dimm_i3c_read, &dimm_post_proc_args[4], NULL },
	{ SENSOR_NUM_MB_DIMMA5_TEMP_C, sensor_dev_i3c_dimm, I3C_BUS4, DIMM_SPD_A1_A5_ADDR,
	  DIMM_SPD_TEMP, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_dimm_i3c_read, NULL,
	  post_intel_dimm_i3c_read, &dimm_post_proc_args[5], NULL },
	{ SENSOR_NUM_MB_DIMMA6_TEMP_C, sensor_dev_i3c_dimm, I3C_BUS4, DIMM_SPD_A2_A6_ADDR,
	  DIMM_SPD_TEMP, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_dimm_i3c_read, NULL,
	  post_intel_dimm_i3c_read, &dimm_post_proc_args[6], NULL },
	{ SENSOR_NUM_MB_DIMMA7_TEMP_C, sensor_dev_i3c_dimm, I3C_BUS4, DIMM_SPD_A3_A7_ADDR,
	  DIMM_SPD_TEMP, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_dimm_i3c_read, NULL,
	  post_intel_dimm_i3c_read, &dimm_post_proc_args[7], NULL },

	{ SENSOR_NUM_MB_SSD0_TEMP_C, sensor_dev_nvme, I2C_BUS2, SSD0_ADDR, SSD0_OFFSET, post_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_nvme_read, &mux_conf_addr_0xe2[1], NULL, NULL, NULL },

	{ SENSOR_NUM_MB_VR_VCCIN_TEMP_C, sensor_dev_xdpe15284, I2C_BUS5, PVCCIN_ADDR,
	  VR_TEMP_OFFSET, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe15284_read,
	  &xdpe15284_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_EHV_TEMP_C, sensor_dev_xdpe15284, I2C_BUS5, EHV_ADDR, VR_TEMP_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_FIVRA_TEMP_C, sensor_dev_xdpe15284, I2C_BUS5, FIVRA_ADDR, VR_TEMP_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_VCCINF_TEMP_C, sensor_dev_xdpe15284, I2C_BUS5, INF_ADDR, VR_TEMP_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_VCCD0_TEMP_C, sensor_dev_xdpe15284, I2C_BUS5, PVCCD0_ADDR,
	  VR_TEMP_OFFSET, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe15284_read,
	  &xdpe15284_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_VCCD1_TEMP_C, sensor_dev_xdpe15284, I2C_BUS5, PVCCD1_ADDR,
	  VR_TEMP_OFFSET, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_xdpe15284_read,
	  &xdpe15284_pre_read_args[1], NULL, NULL, NULL },

	{ SENSOR_NUM_MB_SOC_THERMAL_MARGIN_C, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CPU_MARGIN, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, post_cpu_margin_read, NULL,
	  NULL },
	{ SENSOR_NUM_MB_SOC_TJMAX_C, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CPU_TJMAX, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	//ADC Voltage
	{ SENSOR_NUM_MB_ADC_P12V_STBY_VOLT_V, sensor_dev_ast_adc, ADC_PORT0, NONE, NONE,
	  stby_access, 667, 100, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_MB_ADC_P3V_BAT_VOLT_V, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE, stby_access,
	  3, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_BAT3V, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_vol_bat3v_read, NULL, post_vol_bat3v_read, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_MB_ADC_P3V3_STBY_VOLT_V, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE,
	  stby_access, 2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_MB_ADC_P1V8_STBY_VOLT_V, sensor_dev_ast_adc, ADC_PORT15, NONE, NONE,
	  stby_access, 1, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_MB_ADC_P1V05_STBY_VOLT_V, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE,
	  stby_access, 1, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_MB_ADC_P5V_STBY_VOLT_V, sensor_dev_ast_adc, ADC_PORT9, NONE, NONE, stby_access,
	  711, 200, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_MB_ADC_P12V_DIMM_VOLT_V, sensor_dev_ast_adc, ADC_PORT11, NONE, NONE, dc_access,
	  667, 100, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_MB_ADC_P1V2_STBY_VOLT_V, sensor_dev_ast_adc, ADC_PORT13, NONE, NONE,
	  stby_access, 1, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_MB_ADC_P3V3_M2_VOLT_V, sensor_dev_ast_adc, ADC_PORT14, NONE, NONE, dc_access,
	  2, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_MB_ADC_VNN_VOLT_V, sensor_dev_ast_adc, ADC_PORT8, NONE, NONE, stby_access, 1,
	  1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	//Voltage
	{ SENSOR_NUM_MB_VR_VCCIN_VOLT_V, sensor_dev_xdpe15284, I2C_BUS5, PVCCIN_ADDR, VR_VOL_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_EHV_VOLT_V, sensor_dev_xdpe15284, I2C_BUS5, EHV_ADDR, VR_VOL_OFFSET,
	  vr_stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_FIVRA_VOLT_V, sensor_dev_xdpe15284, I2C_BUS5, FIVRA_ADDR, VR_VOL_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_VCCINF_VOLT_V, sensor_dev_xdpe15284, I2C_BUS5, INF_ADDR, VR_VOL_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_VCCD0_VOLT_V, sensor_dev_xdpe15284, I2C_BUS5, PVCCD0_ADDR, VR_VOL_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_VCCD1_VOLT_V, sensor_dev_xdpe15284, I2C_BUS5, PVCCD1_ADDR, VR_VOL_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[1], NULL, NULL, NULL },

	//Current
	{ SENSOR_NUM_MB_VR_VCCIN_CURR_A, sensor_dev_xdpe15284, I2C_BUS5, PVCCIN_ADDR, VR_CUR_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_EHV_CURR_A, sensor_dev_xdpe15284, I2C_BUS5, EHV_ADDR, VR_CUR_OFFSET,
	  vr_stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_FIVRA_CURR_A, sensor_dev_xdpe15284, I2C_BUS5, FIVRA_ADDR, VR_CUR_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_VCCINF_CURR_A, sensor_dev_xdpe15284, I2C_BUS5, INF_ADDR, VR_CUR_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_VCCD0_CURR_A, sensor_dev_xdpe15284, I2C_BUS5, PVCCD0_ADDR, VR_CUR_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_VCCD1_CURR_A, sensor_dev_xdpe15284, I2C_BUS5, PVCCD1_ADDR, VR_CUR_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[1], NULL, NULL, NULL },

	//Power
	{ SENSOR_NUM_MB_VR_VCCIN_PWR_W, sensor_dev_xdpe15284, I2C_BUS5, PVCCIN_ADDR, VR_PWR_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_EHV_PWR_W, sensor_dev_xdpe15284, I2C_BUS5, EHV_ADDR, VR_PWR_OFFSET,
	  vr_stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_FIVRA_PWR_W, sensor_dev_xdpe15284, I2C_BUS5, FIVRA_ADDR, VR_PWR_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_VCCINF_PWR_W, sensor_dev_xdpe15284, I2C_BUS5, INF_ADDR, VR_PWR_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_VCCD0_PWR_W, sensor_dev_xdpe15284, I2C_BUS5, PVCCD0_ADDR, VR_PWR_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_VR_VCCD1_PWR_W, sensor_dev_xdpe15284, I2C_BUS5, PVCCD1_ADDR, VR_PWR_OFFSET,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_xdpe15284_read, &xdpe15284_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_MB_SOC_CPU_PWR_W, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR, PECI_PWR_CPU,
	  post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	// DIMM PMIC power
	{ SENSOR_NUM_MB_VR_DIMMA0_PMIC_PWR_W, sensor_dev_i3c_dimm, I3C_BUS4, PMIC_A0_A4_ADDR,
	  DIMM_PMIC_SWA_PWR, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_dimm_i3c_read, NULL, NULL, NULL,
	  NULL },
	{ SENSOR_NUM_MB_VR_DIMMA1_PMIC_PWR_W, sensor_dev_i3c_dimm, I3C_BUS4, PMIC_A1_A5_ADDR,
	  DIMM_PMIC_SWA_PWR, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_dimm_i3c_read, NULL, NULL, NULL,
	  NULL },
	{ SENSOR_NUM_MB_VR_DIMMA2_PMIC_PWR_W, sensor_dev_i3c_dimm, I3C_BUS4, PMIC_A2_A6_ADDR,
	  DIMM_PMIC_SWA_PWR, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_dimm_i3c_read, NULL, NULL, NULL,
	  NULL },
	{ SENSOR_NUM_MB_VR_DIMMA3_PMIC_PWR_W, sensor_dev_i3c_dimm, I3C_BUS4, PMIC_A3_A7_ADDR,
	  DIMM_PMIC_SWA_PWR, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_dimm_i3c_read, NULL, NULL, NULL,
	  NULL },
	{ SENSOR_NUM_MB_VR_DIMMA4_PMIC_PWR_W, sensor_dev_i3c_dimm, I3C_BUS4, PMIC_A0_A4_ADDR,
	  DIMM_PMIC_SWA_PWR, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_dimm_i3c_read, NULL, NULL, NULL,
	  NULL },
	{ SENSOR_NUM_MB_VR_DIMMA5_PMIC_PWR_W, sensor_dev_i3c_dimm, I3C_BUS4, PMIC_A1_A5_ADDR,
	  DIMM_PMIC_SWA_PWR, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_dimm_i3c_read, NULL, NULL, NULL,
	  NULL },
	{ SENSOR_NUM_MB_VR_DIMMA6_PMIC_PWR_W, sensor_dev_i3c_dimm, I3C_BUS4, PMIC_A2_A6_ADDR,
	  DIMM_PMIC_SWA_PWR, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_dimm_i3c_read, NULL, NULL, NULL,
	  NULL },
	{ SENSOR_NUM_MB_VR_DIMMA7_PMIC_PWR_W, sensor_dev_i3c_dimm, I3C_BUS4, PMIC_A3_A7_ADDR,
	  DIMM_PMIC_SWA_PWR, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_intel_dimm_i3c_read, NULL, NULL, NULL,
	  NULL },

	// Total DIMM power
	{ SENSOR_NUM_MB_TOTAL_DIMM_PWR_W, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_POWER_TOTAL_DIMM, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
};

sensor_cfg adm1278_sensor_config_table[] = {
	{ SENSOR_NUM_MB_HSC_TEMP_C, sensor_dev_adm1278, I2C_BUS2, HSC_ADM1278_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adm1278_init_args[0] },
	{ SENSOR_NUM_MB_HSC_INPUT_VOLT_V, sensor_dev_adm1278, I2C_BUS2, HSC_ADM1278_ADDR,
	  PMBUS_READ_VIN, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adm1278_init_args[0] },
	{ SENSOR_NUM_MB_HSC_OUTPUT_CURR_A, sensor_dev_adm1278, I2C_BUS2, HSC_ADM1278_ADDR,
	  PMBUS_READ_IOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adm1278_init_args[0] },
	{ SENSOR_NUM_MB_HSC_INPUT_PWR_W, sensor_dev_adm1278, I2C_BUS2, HSC_ADM1278_ADDR,
	  PMBUS_READ_PIN, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &adm1278_init_args[0] },
};

sensor_cfg mp5990_sensor_config_table[] = {
	{ SENSOR_NUM_MB_HSC_TEMP_C, sensor_dev_mp5990, I2C_BUS2, HSC_MP5990_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &mp5990_init_args[0] },
	{ SENSOR_NUM_MB_HSC_INPUT_VOLT_V, sensor_dev_mp5990, I2C_BUS2, HSC_MP5990_ADDR,
	  PMBUS_READ_VIN, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &mp5990_init_args[0] },
	{ SENSOR_NUM_MB_HSC_OUTPUT_CURR_A, sensor_dev_mp5990, I2C_BUS2, HSC_MP5990_ADDR,
	  PMBUS_READ_IOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &mp5990_init_args[0] },
	{ SENSOR_NUM_MB_HSC_INPUT_PWR_W, sensor_dev_mp5990, I2C_BUS2, HSC_MP5990_ADDR,
	  PMBUS_READ_PIN, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &mp5990_init_args[0] },
};

const int SENSOR_CONFIG_SIZE = ARRAY_SIZE(plat_sensor_config);

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
	default:
		LOG_ERR("Unsupported HSC module, HSC module: 0x%x", hsc_module);
		break;
	}

	return extend_sensor_config_size;
}

static void check_vr_type(uint8_t index)
{
	uint8_t retry = 5;
	I2C_MSG msg;

	xdpe15284_pre_read_arg *args = sensor_config[index].pre_sensor_read_args;
	memset(&msg, 0, sizeof(msg));
	msg.bus = sensor_config[index].port;
	msg.target_addr = sensor_config[index].target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = args->vr_page;
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Failed to switch to VR page %d", args->vr_page);
		return;
	}

	/* Get IC Device ID from VR chip */
	memset(&msg, 0, sizeof(msg));
	msg.bus = sensor_config[index].port;
	msg.target_addr = sensor_config[index].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 7;
	msg.data[0] = PMBUS_IC_DEVICE_ID;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to read VR IC_DEVICE_ID: register(0x%x)", PMBUS_IC_DEVICE_ID);
		return;
	}

	if ((msg.data[0] == 0x01) && (msg.data[1] == 0x85)) {
		sensor_config[index].type = sensor_dev_mp2985;
		sensor_config[index].init_args = &mp2985_init_args[0];
	} else if ((msg.data[0] == 0x02) && (msg.data[2] == 0x8A)) {
		sensor_config[index].type = sensor_dev_xdpe15284;
	} else if ((msg.data[0] == 0x04) && (msg.data[1] == 0x00) && (msg.data[2] == 0x81) &&
		   (msg.data[3] == 0xD2) && (msg.data[4] == 0x49)) {
		sensor_config[index].type = sensor_dev_isl69259;
	} else {
		LOG_ERR("Unknown VR type");
	}
}

void pal_extend_sensor_config()
{
	/* Follow the hardware design,
	 * the GPIOA7(HSC_SET_EN_R) should be set to "H"
	 * and the 2OU configuration is set if the 2OU is present.
	 */
	CARD_STATUS _2ou_status = get_2ou_status();

	int arg_index = (_2ou_status.present) ? 1 : 0;
	int gpio_state = (_2ou_status.present) ? GPIO_HIGH : GPIO_LOW;
	gpio_set(HSC_SET_EN_R, gpio_state);

	uint8_t sensor_count = 0;
	uint8_t hsc_module = get_hsc_module();
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
			mp5990_sensor_config_table[index].init_args = &mp5990_init_args[arg_index];
			add_sensor_config(mp5990_sensor_config_table[index]);
		}
		break;
	default:
		LOG_ERR("Unsupported HSC module, HSC module: 0x%x", hsc_module);
		break;
	}

	/* Following the hardware design,
	 * MPS and Renesas VR chips are used after EVT stage.
	 * And the slave address is changed after EVT stage.
	 * So, BIC revises VR slave address according to server board revision.
	 * Besides, BIC judges VR chip type by getting device id
	 * and then replace sensor config table before loading.
	 */
	uint8_t board_revision = get_board_revision();
	sensor_count = ARRAY_SIZE(plat_sensor_config);
	for (uint8_t index = 0; index < sensor_count; index++) {
		if (sensor_config[index].type == sensor_dev_xdpe15284) {
			if ((board_revision >= SYS_BOARD_EVT) && (board_revision <= SYS_BOARD_MP)) {
				switch (sensor_config[index].target_addr) {
				case FIVRA_ADDR:
					sensor_config[index].target_addr = EVT_FIVRA_ADDR;
					break;
				case PVCCD0_ADDR:
					sensor_config[index].target_addr = EVT_PVCCD0_ADDR;
					break;

				default:
					break;
				};
			};
			check_vr_type(index);
		}
	}
}

static int sensor_get_idx_by_sensor_num(uint16_t sensor_num)
{
	int sensor_idx = 0;
	for (sensor_idx = 0; sensor_idx < sensor_config_count; sensor_idx++) {
		if (sensor_num == sensor_config[sensor_idx].num)
			return sensor_idx;
	}

	return -1;
}

uint8_t get_dimm_status(uint8_t dimm_index)
{
	int sensor_index =
		sensor_get_idx_by_sensor_num(dimm_pmic_map_table[dimm_index].dimm_sensor_num);
	if (sensor_index < 0) {
		return SENSOR_NOT_SUPPORT;
	}

	return sensor_config[sensor_index].cache_status;
}
