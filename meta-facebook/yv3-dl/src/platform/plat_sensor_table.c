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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <logging/log.h>
#include "plat_sensor_table.h"
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
#include "util_sys.h"

LOG_MODULE_DECLARE(sensor);

struct k_mutex vr_page_mutex;

sensor_cfg plat_sensor_config[] = {
	/* number, type, port, address, offset, access check arg0, arg1, sample_count,
           polling time, polling enable, cache, cache_status, mux_ADDRess, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_args  */

	// temperature
	{ SENSOR_NUM_T_MB1, sensor_dev_tmp75, I2C_BUS5, TMP75_IN_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_T_FIO, sensor_dev_tmp75, I2C_BUS5, TMP75_FIO_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	// NVME
	{ SENSOR_NUM_T_NVME1, sensor_dev_nvme, I2C_BUS5, SSD0_ADDR, SSD0_OFFSET, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_nvme_read, &mux_conf_addr_0xe2[1], NULL, NULL, NULL },

	// VR voltage
	{ SENSOR_NUM_VOL_PVCCIO_VR, sensor_dev_xdpe12284c, I2C_BUS8, VCCIO_P3V3_STBY_ADDR,
	  VR_VOL_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0],
	  post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_VOL_P3V3_STBY_VR, sensor_dev_xdpe12284c, I2C_BUS8, VCCIO_P3V3_STBY_ADDR,
	  VR_VOL_CMD, vr_stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1],
	  post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_V_DIMM_ABC_VR, sensor_dev_xdpe12284c, I2C_BUS8, VDDQ_ABC_ADDR, VR_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_V_DIMM_DEF_VR, sensor_dev_xdpe12284c, I2C_BUS8, VDDQ_DEF_ADDR, VR_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCIN_VR, sensor_dev_xdpe12284c, I2C_BUS8, VCCIN_VCCSA_ADDR, VR_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCSA_VR, sensor_dev_xdpe12284c, I2C_BUS8, VCCIN_VCCSA_ADDR, VR_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1], post_xdpe12284c_read, NULL, NULL },

	// VR current
	{ SENSOR_NUM_CUR_PVCCIO_VR, sensor_dev_xdpe12284c, I2C_BUS8, VCCIO_P3V3_STBY_ADDR,
	  VR_CUR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0],
	  post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_CUR_P3V3_STBY_VR, sensor_dev_xdpe12284c, I2C_BUS8, VCCIO_P3V3_STBY_ADDR,
	  VR_CUR_CMD, vr_stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1],
	  post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_CURR_DIMM_ABC_VR, sensor_dev_xdpe12284c, I2C_BUS8, VDDQ_ABC_ADDR, VR_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_CURR_DIMM_DEF_VR, sensor_dev_xdpe12284c, I2C_BUS8, VDDQ_DEF_ADDR, VR_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCIN_VR, sensor_dev_xdpe12284c, I2C_BUS8, VCCIN_VCCSA_ADDR, VR_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCSA_VR, sensor_dev_xdpe12284c, I2C_BUS8, VCCIN_VCCSA_ADDR, VR_CUR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1], post_xdpe12284c_read, NULL, NULL },

	// VR temperature
	{ SENSOR_NUM_TEP_PVCCIO_VR, sensor_dev_xdpe12284c, I2C_BUS8, VCCIO_P3V3_STBY_ADDR,
	  VR_TEMP_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0],
	  post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_TEP_P3V3_STBY_VR, sensor_dev_xdpe12284c, I2C_BUS8, VCCIO_P3V3_STBY_ADDR,
	  VR_TEMP_CMD, vr_stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1],
	  post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_T_DIMM_ABC_VR, sensor_dev_xdpe12284c, I2C_BUS8, VDDQ_ABC_ADDR, VR_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_T_DIMM_DEF_VR, sensor_dev_xdpe12284c, I2C_BUS8, VDDQ_DEF_ADDR, VR_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_TEP_PVCCIN_VR, sensor_dev_xdpe12284c, I2C_BUS8, VCCIN_VCCSA_ADDR, VR_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_TEP_PVCCSA_VR, sensor_dev_xdpe12284c, I2C_BUS8, VCCIN_VCCSA_ADDR, VR_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1], post_xdpe12284c_read, NULL, NULL },

	// VR power
	{ SENSOR_NUM_PWR_PVCCIO_VR, sensor_dev_xdpe12284c, I2C_BUS8, VCCIO_P3V3_STBY_ADDR,
	  VR_PWR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0],
	  post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_PWR_P3V3_STBY_VR, sensor_dev_xdpe12284c, I2C_BUS8, VCCIO_P3V3_STBY_ADDR,
	  VR_PWR_CMD, vr_stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1],
	  post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_PWR_DIMM_ABC_VR, sensor_dev_xdpe12284c, I2C_BUS8, VDDQ_ABC_ADDR, VR_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_PWR_DIMM_DEF_VR, sensor_dev_xdpe12284c, I2C_BUS8, VDDQ_DEF_ADDR, VR_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCIN_VR, sensor_dev_xdpe12284c, I2C_BUS8, VCCIN_VCCSA_ADDR, VR_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], post_xdpe12284c_read, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCSA_VR, sensor_dev_xdpe12284c, I2C_BUS8, VCCIN_VCCSA_ADDR, VR_PWR_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1], post_xdpe12284c_read, NULL, NULL },

	// PECI
	{ SENSOR_NUM_T_CPU0, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR, PECI_TEMP_CPU, post_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_T_CPU0_THERMAL_MARGIN, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CPU_MARGIN, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, post_cpu_margin_read, NULL,
	  NULL },
	{ SENSOR_NUM_T_CPU0_TJMAX, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR, PECI_TEMP_CPU_TJMAX,
	  post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_T_CPU0_PKG_PWR, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR, PECI_PWR_CPU,
	  post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_NM_T_DIMMA, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL0_DIMM0, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_NM_T_DIMMB, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL1_DIMM0, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_NM_T_DIMMC, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL2_DIMM0, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_NM_T_DIMMD, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL3_DIMM0, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_NM_T_DIMME, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL4_DIMM0, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_NM_T_DIMMF, sensor_dev_intel_peci, NONE, CPU_PECI_ADDR,
	  PECI_TEMP_CHANNEL5_DIMM0, post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	// adc voltage
	{ SENSOR_NUM_V_12, sensor_dev_ast_adc, ADC_PORT0, NONE, NONE, stby_access, 178, 20,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_V_3_3_S, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE, stby_access, 487, 200,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_V_1_5, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_V_BAT, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE, stby_access, 3, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_vol_bat3v_read, NULL, post_vol_bat3v_read, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_V_PCH, sensor_dev_ast_adc, ADC_PORT5, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	// ME
	{ SENSOR_NUM_NM_T_PCH, sensor_dev_pch, I2C_BUS6, PCH_ADDR, ME_SENSOR_NUM_TEMP_PCH,
	  me_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
};

sensor_cfg ltc4282_sensor_config_table[] = {
	{ SENSOR_NUM_T_MB2, sensor_dev_tmp431, I2C_BUS5, TMP431_ADDR, TMP431_LOCAL_TEMPERATRUE,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_HSC_TEMP, sensor_dev_tmp431, I2C_BUS5, TMP431_ADDR, TMP431_REMOTE_TEMPERATRUE,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_HSC_VIN, sensor_dev_ltc4282, I2C_BUS5, LTC4282_ADDR, LTC4282_VSOURCE_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4282_init_args[0] },
	{ SENSOR_NUM_HSC_PIN, sensor_dev_ltc4282, I2C_BUS5, LTC4282_ADDR, LTC4282_POWER_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4282_init_args[0] },
	{ SENSOR_NUM_HSC_EIN, sensor_dev_ltc4282, I2C_BUS5, LTC4282_ADDR, LTC4282_ENERGY_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4282_init_args[0] },
	{ SENSOR_NUM_HSC_COUT, sensor_dev_ltc4282, I2C_BUS5, LTC4282_ADDR, LTC4282_VSENSE_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4282_init_args[0] },
};

sensor_cfg mp5990_sensor_config_table[] = {
	{ SENSOR_NUM_T_MB2, sensor_dev_tmp75, I2C_BUS5, TMP75_OUT_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_HSC_TEMP, sensor_dev_mp5990, I2C_BUS5, MPS_MP5990_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &mp5990_init_args[0] },
	{ SENSOR_NUM_HSC_VIN, sensor_dev_mp5990, I2C_BUS5, MPS_MP5990_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_HSC_PIN, sensor_dev_mp5990, I2C_BUS5, MPS_MP5990_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_HSC_COUT, sensor_dev_mp5990, I2C_BUS5, MPS_MP5990_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_HSC_EIN, sensor_dev_mp5990, I2C_BUS5, MPS_MP5990_ADDR, PMBUS_READ_EIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
};

sensor_cfg isl69254_sensor_config_table[] = {
	// VR voltage
	{ SENSOR_NUM_VOL_PVCCIO_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VCCIO_P3V3_STBY_ADDR,
	  VR_VOL_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0],
	  post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_VOL_P3V3_STBY_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VCCIO_P3V3_STBY_ADDR,
	  VR_VOL_CMD, vr_stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1],
	  post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_V_DIMM_ABC_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VDDQ_ABC_ADDR, VR_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_V_DIMM_DEF_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VDDQ_DEF_ADDR, VR_VOL_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCIN_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VCCIN_VCCSA_ADDR,
	  VR_VOL_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0],
	  post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_VOL_PVCCSA_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VCCIN_VCCSA_ADDR,
	  VR_VOL_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1],
	  post_isl69254_read, NULL, NULL },

	// VR current
	{ SENSOR_NUM_CUR_PVCCIO_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VCCIO_P3V3_STBY_ADDR,
	  VR_CUR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0],
	  post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_CUR_P3V3_STBY_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VCCIO_P3V3_STBY_ADDR,
	  VR_CUR_CMD, vr_stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1],
	  post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_CURR_DIMM_ABC_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VDDQ_ABC_ADDR,
	  VR_CUR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0],
	  post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_CURR_DIMM_DEF_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VDDQ_DEF_ADDR,
	  VR_CUR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0],
	  post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCIN_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VCCIN_VCCSA_ADDR,
	  VR_CUR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0],
	  post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_CUR_PVCCSA_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VCCIN_VCCSA_ADDR,
	  VR_CUR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1],
	  post_isl69254_read, NULL, NULL },

	// VR temperature
	{ SENSOR_NUM_TEP_PVCCIO_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VCCIO_P3V3_STBY_ADDR,
	  VR_TEMP_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0],
	  post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_TEP_P3V3_STBY_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VCCIO_P3V3_STBY_ADDR,
	  VR_TEMP_CMD, vr_stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1],
	  post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_T_DIMM_ABC_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VDDQ_ABC_ADDR, VR_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_T_DIMM_DEF_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VDDQ_DEF_ADDR, VR_TEMP_CMD,
	  vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0], post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_TEP_PVCCIN_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VCCIN_VCCSA_ADDR,
	  VR_TEMP_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0],
	  post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_TEP_PVCCSA_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VCCIN_VCCSA_ADDR,
	  VR_TEMP_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1],
	  post_isl69254_read, NULL, NULL },

	// VR power
	{ SENSOR_NUM_PWR_PVCCIO_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VCCIO_P3V3_STBY_ADDR,
	  VR_PWR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0],
	  post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_PWR_P3V3_STBY_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VCCIO_P3V3_STBY_ADDR,
	  VR_PWR_CMD, vr_stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1],
	  post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_PWR_DIMM_ABC_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VDDQ_ABC_ADDR,
	  VR_PWR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0],
	  post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_PWR_DIMM_DEF_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VDDQ_DEF_ADDR,
	  VR_PWR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0],
	  post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCIN_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VCCIN_VCCSA_ADDR,
	  VR_PWR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[0],
	  post_isl69254_read, NULL, NULL },
	{ SENSOR_NUM_PWR_PVCCSA_VR, sensor_dev_isl69254iraz_t, I2C_BUS8, VCCIN_VCCSA_ADDR,
	  VR_PWR_CMD, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_page_select[1],
	  post_isl69254_read, NULL, NULL },
};

const int SENSOR_CONFIG_SIZE = ARRAY_SIZE(plat_sensor_config);

void load_sensor_config(void)
{
	if (k_mutex_init(&vr_page_mutex)) {
		printf("vr_page_mutex mutex init fail\n");
	}

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
	case HSC_MODULE_MP5990:
		extend_sensor_config_size += ARRAY_SIZE(mp5990_sensor_config_table);
		break;
	case HSC_MODULE_LTC4282:
		extend_sensor_config_size += ARRAY_SIZE(ltc4282_sensor_config_table);
		break;
	default:
		LOG_ERR("unsupported HSC module, HSC module: 0x%x", hsc_module);
		break;
	}

	return extend_sensor_config_size;
}

static int check_vr_type(void)
{
	uint8_t retry = 5;
	I2C_MSG msg;

	memset(&msg, 0, sizeof(msg));
	msg.bus = I2C_BUS8;
	msg.target_addr = VCCIN_VCCSA_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 7;
	msg.data[0] = PMBUS_IC_DEVICE_ID;

	if (i2c_master_read(&msg, retry)) {
		printf("Failed to read VR IC_DEVICE_ID: register(0x%x)\n", PMBUS_IC_DEVICE_ID);
		return -1;
	}

	if (memcmp(msg.data, ISL69254_DEVICE_ID, sizeof(ISL69254_DEVICE_ID)) == 0) {
		printf("VR type: RNS\n");
		return VR_RNS;
	} else if (memcmp(msg.data, XDPE12284C_DEVICE_ID, sizeof(XDPE12284C_DEVICE_ID)) == 0) {
		printf("VR type: INF\n");
		return VR_INF;
	}

	printf("Unknown VR type\n");
	return -1;
}

void check_outlet_temp_type(uint8_t index)
{
	if (index >= sensor_config_count) {
		printf("Out of sensor_config_count\n");
		return;
	}

	uint8_t retry = 5;
	I2C_MSG msg;
	uint8_t CID = 0;
	uint8_t VID = 0;

	/* Get Chip ID and Manufacturer ID
	 * - Command code: 0xFD, 0xFE
	 * TMP431: 0x31 0x55
	 * NCT7718W: 0x50 0x50
	 * G788P81U: 0x50 0x47
	 */
	memset(&msg, 0, sizeof(msg));
	msg.bus = sensor_config[index].port;
	msg.target_addr = sensor_config[index].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = NCT7718W_CHIP_ID_OFFSET;

	if (i2c_master_read(&msg, retry)) {
		printf("Failed to read Outlet_Temp chip ID: register(0x%x)\n",
		       NCT7718W_CHIP_ID_OFFSET);
		return;
	}
	CID = msg.data[0];

	memset(&msg, 0, sizeof(msg));
	msg.bus = sensor_config[index].port;
	msg.target_addr = sensor_config[index].target_addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = NCT7718W_VENDOR_ID_OFFSET;

	if (i2c_master_read(&msg, retry)) {
		printf("Failed to read Outlet_Temp vendor ID: register(0x%x)\n",
		       NCT7718W_VENDOR_ID_OFFSET);
		return;
	}
	VID = msg.data[0];

	if ((CID == 0x31) && (VID == 0x55)) {
		sensor_config[index].type = sensor_dev_tmp431;
	} else if ((CID == 0x50) && (VID == 0x50)) {
		sensor_config[index].type = sensor_dev_nct7718w;
	} else if ((CID == 0x50) && (VID == 0x47)) {
		sensor_config[index].type = sensor_dev_g788p81u;
	} else {
		printf("Unknown Outlet_Temp type\n");
	}
}

void pal_extend_sensor_config()
{
	uint8_t sensor_count = 0;
	uint8_t hsc_module = get_hsc_module();

	switch (check_vr_type()) {
	case VR_RNS:
		sensor_count = ARRAY_SIZE(isl69254_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(isl69254_sensor_config_table[index]);
		}
		break;
	case VR_INF:
	default:
		printf("Using default VR(INF) sensor table\n");
		break;
	}

	/* Check outlet temperature sensor type */
	for (uint8_t index = 0; index < sensor_config_count; index++) {
		if (sensor_config[index].type == sensor_dev_tmp431) {
			check_outlet_temp_type(index);
		}
	}

	switch (hsc_module) {
	case HSC_MODULE_MP5990:
		sensor_count = ARRAY_SIZE(mp5990_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(mp5990_sensor_config_table[index]);
		}
		break;
	case HSC_MODULE_LTC4282:
		sensor_count = ARRAY_SIZE(ltc4282_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(ltc4282_sensor_config_table[index]);
		}
		break;
	default:
		LOG_ERR("unsupported HSC module, HSC module: 0x%x", hsc_module);
		break;
	}

	return;
}

uint8_t get_hsc_pwr_reading(int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);
	return get_sensor_reading(SENSOR_NUM_HSC_PIN, reading, GET_FROM_CACHE);
}
