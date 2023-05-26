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
#include <logging/log.h>
#include "plat_sensor_table.h"
#include "sensor.h"
#include "ast_adc.h"
#include "plat_hook.h"
#include "pmbus.h"
#include "plat_i2c.h"
#include "apml.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_sensor_table);

sensor_poll_time_cfg diff_poll_time_sensor_table[] = {
	// sensor_number, last_access_time
	{ SENSOR_NUM_VOL_P3V_BAT, 0 },
};

sensor_cfg plat_sensor_config[] = {
	/* number, type, port, address, offset, access check, arg0, arg1, cache, cache_status,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn,
	   init_arg */

	/* temperature */
	{ SENSOR_NUM_TEMP_TMP75_IN, sensor_dev_tmp75, I2C_BUS2, TMP75_IN_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_TMP75_OUT, sensor_dev_tmp75, I2C_BUS2, TMP75_OUT_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_TMP75_FIO, sensor_dev_tmp75, I2C_BUS2, TMP75_FIO_ADDR, TMP75_TEMP_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_nct7718w, I2C_BUS5, TEMP_HSC_ADDR,
	  NCT7718W_REMOTE_TEMP_MSB_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT,
	  POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &nct7718w_init_args[0] },

	/* NVME */
	{ SENSOR_NUM_TEMP_SSD, sensor_dev_nvme, I2C_BUS2, SSD_ADDR, SSD_TEMP_OFFSET, post_access, 0,
	  0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_nvme_read, &mux_conf_addr_0xe2[1], NULL, NULL, NULL },

	/* CPU */
	{ SENSOR_NUM_TEMP_CPU, sensor_dev_amd_tsi, I2C_BUS14, TSI_ADDR, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_amd_tsi_read, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_A0, sensor_dev_ddr5_temp, I2C_BUS11, NONE, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_ddr5_temp_read, NULL, &ddr5_init_temp_args[0] },
	{ SENSOR_NUM_TEMP_DIMM_A1, sensor_dev_ddr5_temp, I2C_BUS11, NONE, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_ddr5_temp_read, NULL, &ddr5_init_temp_args[1] },
	{ SENSOR_NUM_TEMP_DIMM_A2, sensor_dev_ddr5_temp, I2C_BUS11, NONE, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_ddr5_temp_read, NULL, &ddr5_init_temp_args[2] },
	{ SENSOR_NUM_TEMP_DIMM_A4, sensor_dev_ddr5_temp, I2C_BUS11, NONE, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_ddr5_temp_read, NULL, &ddr5_init_temp_args[3] },
	{ SENSOR_NUM_TEMP_DIMM_A6, sensor_dev_ddr5_temp, I2C_BUS12, NONE, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_ddr5_temp_read, NULL, &ddr5_init_temp_args[4] },
	{ SENSOR_NUM_TEMP_DIMM_A7, sensor_dev_ddr5_temp, I2C_BUS12, NONE, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_ddr5_temp_read, NULL, &ddr5_init_temp_args[5] },
	{ SENSOR_NUM_TEMP_DIMM_A8, sensor_dev_ddr5_temp, I2C_BUS12, NONE, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_ddr5_temp_read, NULL, &ddr5_init_temp_args[6] },
	{ SENSOR_NUM_TEMP_DIMM_A10, sensor_dev_ddr5_temp, I2C_BUS12, NONE, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_ddr5_temp_read, NULL, &ddr5_init_temp_args[7] },
	{ SENSOR_NUM_PWR_CPU, sensor_dev_apml_mailbox, I2C_BUS14, APML_ADDR, SBRMI_MAILBOX_PKGPWR,
	  post_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &apml_mailbox_init_args[0] },
	{ SENSOR_NUM_PWR_DIMM_A0, sensor_dev_ddr5_power, I2C_BUS11, NONE, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_ddr5_pwr_read, NULL, &ddr5_init_power_args[0] },
	{ SENSOR_NUM_PWR_DIMM_A1, sensor_dev_ddr5_power, I2C_BUS11, NONE, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_ddr5_pwr_read, NULL, &ddr5_init_power_args[1] },
	{ SENSOR_NUM_PWR_DIMM_A2, sensor_dev_ddr5_power, I2C_BUS11, NONE, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_ddr5_pwr_read, NULL, &ddr5_init_power_args[2] },
	{ SENSOR_NUM_PWR_DIMM_A4, sensor_dev_ddr5_power, I2C_BUS11, NONE, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_ddr5_pwr_read, NULL, &ddr5_init_power_args[3] },
	{ SENSOR_NUM_PWR_DIMM_A6, sensor_dev_ddr5_power, I2C_BUS12, NONE, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_ddr5_pwr_read, NULL, &ddr5_init_power_args[4] },
	{ SENSOR_NUM_PWR_DIMM_A7, sensor_dev_ddr5_power, I2C_BUS12, NONE, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_ddr5_pwr_read, NULL, &ddr5_init_power_args[5] },
	{ SENSOR_NUM_PWR_DIMM_A8, sensor_dev_ddr5_power, I2C_BUS12, NONE, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_ddr5_pwr_read, NULL, &ddr5_init_power_args[6] },
	{ SENSOR_NUM_PWR_DIMM_A10, sensor_dev_ddr5_power, I2C_BUS12, NONE, NONE, post_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, post_ddr5_pwr_read, NULL, &ddr5_init_power_args[7] },

	/* adc voltage */
	{ SENSOR_NUM_VOL_P12V_STBY, sensor_dev_ast_adc, ADC_PORT0, NONE, NONE, stby_access, 66, 10,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_PVDD18_S5, sensor_dev_ast_adc, ADC_PORT1, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_P3V3_STBY, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE, stby_access, 2, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_PVDD11_S3, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_P3V_BAT, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE, stby_access, 31, 10,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_BAT3V, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_vol_bat3v_read, NULL, post_vol_bat3v_read, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_PVDD33_S5, sensor_dev_ast_adc, ADC_PORT5, NONE, NONE, dc_access, 2, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_P5V_STBY, sensor_dev_ast_adc, ADC_PORT14, NONE, NONE, stby_access, 711,
	  200, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ast_adc_init_args[1] },
	{ SENSOR_NUM_VOL_P12V_MEM_1, sensor_dev_ast_adc, ADC_PORT13, NONE, NONE, dc_access, 66, 10,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[1] },
	{ SENSOR_NUM_VOL_P12V_MEM_0, sensor_dev_ast_adc, ADC_PORT12, NONE, NONE, dc_access, 66, 10,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[1] },
	{ SENSOR_NUM_VOL_P1V2_STBY, sensor_dev_ast_adc, ADC_PORT10, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[1] },
	{ SENSOR_NUM_VOL_P3V3_M2, sensor_dev_ast_adc, ADC_PORT9, NONE, NONE, dc_access, 2, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[1] },
	{ SENSOR_NUM_VOL_P1V8_STBY, sensor_dev_ast_adc, ADC_PORT8, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[1] },

	/* VR voltage */
	{ SENSOR_NUM_VOL_PVDDCR_CPU0_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDDCR_CPU0_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_VOL_PVDDCR_SOC_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDDCR_SOC_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_VOL_PVDDCR_CPU1_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDDCR_CPU1_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_VOL_PVDDIO_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDDIO_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_VOL_PVDD11_S3_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDD11_S3_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },

	/* VR current */
	{ SENSOR_NUM_CUR_PVDDCR_CPU0_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDDCR_CPU0_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_CUR_PVDDCR_SOC_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDDCR_SOC_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_CUR_PVDDCR_CPU1_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDDCR_CPU1_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_CUR_PVDDIO_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDDIO_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_CUR_PVDD11_S3_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDD11_S3_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },

	/* VR temperature */
	{ SENSOR_NUM_TEMP_PVDDCR_CPU0_VR, sensor_dev_raa229621, I2C_BUS5,
	  RAA229621_PVDDCR_CPU0_ADDR, PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDDCR_SOC_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDDCR_SOC_ADDR,
	  PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDDCR_CPU1_VR, sensor_dev_raa229621, I2C_BUS5,
	  RAA229621_PVDDCR_CPU1_ADDR, PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDDIO_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDDIO_ADDR,
	  PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDD11_S3_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDD11_S3_ADDR,
	  PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },

	/* VR power */
	{ SENSOR_NUM_PWR_PVDDCR_CPU0_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDDCR_CPU0_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_PWR_PVDDCR_SOC_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDDCR_SOC_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_PWR_PVDDCR_CPU1_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDDCR_CPU1_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_PWR_PVDDIO_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDDIO_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_PWR_PVDD11_S3_VR, sensor_dev_raa229621, I2C_BUS5, RAA229621_PVDD11_S3_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
};

sensor_cfg adm1278_sensor_config_table[] = {
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_adm1278, I2C_BUS5, ADM1278_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_adm1278, I2C_BUS5, ADM1278_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_adm1278_cur_read, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_adm1278, I2C_BUS5, ADM1278_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_adm1278_pwr_read, NULL, &adm1278_init_args[0] },

};

sensor_cfg ltc4282_sensor_config_table[] = {
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_ltc4282, I2C_BUS5, LTC4282_ADDR, LTC4282_VSOURCE_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4282_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_ltc4282, I2C_BUS5, LTC4282_ADDR, LTC4282_VSENSE_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_ltc4282_cur_read, NULL, &ltc4282_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_ltc4282, I2C_BUS5, LTC4282_ADDR, LTC4282_POWER_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_ltc4282_pwr_read, NULL, &ltc4282_init_args[0] },
};

sensor_cfg mp5990_sensor_config_table[] = {
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_mp5990, I2C_BUS5, MP5990_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_mp5990, I2C_BUS5, MP5990_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_mp5990_cur_read, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_mp5990, I2C_BUS5, MP5990_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_mp5990_pwr_read, NULL, &mp5990_init_args[0] },
};

sensor_cfg xdpe19283b_sensor_config_table[] = {
	/* VR voltage */
	{ SENSOR_NUM_VOL_PVDDCR_CPU0_VR, sensor_dev_xdpe19283b, I2C_BUS5,
	  XDPE19283B_PVDDCR_CPU0_ADDR, PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT,
	  POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read,
	  &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVDDCR_SOC_VR, sensor_dev_xdpe19283b, I2C_BUS5, XDPE19283B_PVDDCR_SOC_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_VOL_PVDDCR_CPU1_VR, sensor_dev_xdpe19283b, I2C_BUS5,
	  XDPE19283B_PVDDCR_CPU1_ADDR, PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT,
	  POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read,
	  &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PVDDIO_VR, sensor_dev_xdpe19283b, I2C_BUS5, XDPE19283B_PVDDIO_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_VOL_PVDD11_S3_VR, sensor_dev_xdpe19283b, I2C_BUS5, XDPE19283B_PVDD11_S3_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },

	/* VR current */
	{ SENSOR_NUM_CUR_PVDDCR_CPU0_VR, sensor_dev_xdpe19283b, I2C_BUS5,
	  XDPE19283B_PVDDCR_CPU0_ADDR, PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT,
	  POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read,
	  &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVDDCR_SOC_VR, sensor_dev_xdpe19283b, I2C_BUS5, XDPE19283B_PVDDCR_SOC_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_CUR_PVDDCR_CPU1_VR, sensor_dev_xdpe19283b, I2C_BUS5,
	  XDPE19283B_PVDDCR_CPU1_ADDR, PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT,
	  POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read,
	  &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PVDDIO_VR, sensor_dev_xdpe19283b, I2C_BUS5, XDPE19283B_PVDDIO_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_CUR_PVDD11_S3_VR, sensor_dev_xdpe19283b, I2C_BUS5, XDPE19283B_PVDD11_S3_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },

	/* VR temperature */
	{ SENSOR_NUM_TEMP_PVDDCR_CPU0_VR, sensor_dev_xdpe19283b, I2C_BUS5,
	  XDPE19283B_PVDDCR_CPU0_ADDR, PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDDCR_SOC_VR, sensor_dev_xdpe19283b, I2C_BUS5,
	  XDPE19283B_PVDDCR_SOC_ADDR, PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_vr_read, &vr_pre_read_args[1], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDDCR_CPU1_VR, sensor_dev_xdpe19283b, I2C_BUS5,
	  XDPE19283B_PVDDCR_CPU1_ADDR, PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDDIO_VR, sensor_dev_xdpe19283b, I2C_BUS5, XDPE19283B_PVDDIO_ADDR,
	  PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDD11_S3_VR, sensor_dev_xdpe19283b, I2C_BUS5, XDPE19283B_PVDD11_S3_ADDR,
	  PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },

	/* VR power */
	{ SENSOR_NUM_PWR_PVDDCR_CPU0_VR, sensor_dev_xdpe19283b, I2C_BUS5,
	  XDPE19283B_PVDDCR_CPU0_ADDR, PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT,
	  POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read,
	  &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVDDCR_SOC_VR, sensor_dev_xdpe19283b, I2C_BUS5, XDPE19283B_PVDDCR_SOC_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_PWR_PVDDCR_CPU1_VR, sensor_dev_xdpe19283b, I2C_BUS5,
	  XDPE19283B_PVDDCR_CPU1_ADDR, PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT,
	  POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read,
	  &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PVDDIO_VR, sensor_dev_xdpe19283b, I2C_BUS5, XDPE19283B_PVDDIO_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_PWR_PVDD11_S3_VR, sensor_dev_xdpe19283b, I2C_BUS5, XDPE19283B_PVDD11_S3_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
};

sensor_cfg mp2856gut_sensor_config_table[] = {
	/* VR voltage */
	{ SENSOR_NUM_VOL_PVDDCR_CPU0_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDDCR_CPU0_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_VOL_PVDDCR_SOC_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDDCR_SOC_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_VOL_PVDDCR_CPU1_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDDCR_CPU1_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_VOL_PVDDIO_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDDIO_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_VOL_PVDD11_S3_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDD11_S3_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },

	/* VR current */
	{ SENSOR_NUM_CUR_PVDDCR_CPU0_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDDCR_CPU0_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_CUR_PVDDCR_SOC_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDDCR_SOC_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_CUR_PVDDCR_CPU1_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDDCR_CPU1_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_CUR_PVDDIO_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDDIO_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_CUR_PVDD11_S3_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDD11_S3_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },

	/* VR temperature */
	{ SENSOR_NUM_TEMP_PVDDCR_CPU0_VR, sensor_dev_mp2856gut, I2C_BUS5,
	  MP2856GUT_PVDDCR_CPU0_ADDR, PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDDCR_SOC_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDDCR_SOC_ADDR,
	  PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDDCR_CPU1_VR, sensor_dev_mp2856gut, I2C_BUS5,
	  MP2856GUT_PVDDCR_CPU1_ADDR, PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_vr_read, &vr_pre_read_args[0], NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDDIO_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDDIO_ADDR,
	  PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDD11_S3_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDD11_S3_ADDR,
	  PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },

	/* VR power */
	{ SENSOR_NUM_PWR_PVDDCR_CPU0_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDDCR_CPU0_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_PWR_PVDDCR_SOC_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDDCR_SOC_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_PWR_PVDDCR_CPU1_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDDCR_CPU1_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_PWR_PVDDIO_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDDIO_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_PWR_PVDD11_S3_VR, sensor_dev_mp2856gut, I2C_BUS5, MP2856GUT_PVDD11_S3_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
};

sensor_cfg tps53689_sensor_config_table[] = {
	/* VR voltage */
	{ SENSOR_NUM_VOL_PVDDCR_CPU0_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDDCR_CPU0_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_VOL_PVDDCR_SOC_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDDCR_SOC_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_VOL_PVDDCR_CPU1_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDDCR_CPU1_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_VOL_PVDDIO_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDDIO_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_VOL_PVDD11_S3_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDD11_S3_ADDR,
	  PMBUS_READ_VOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },

	/* VR current */
	{ SENSOR_NUM_CUR_PVDDCR_CPU0_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDDCR_CPU0_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_CUR_PVDDCR_SOC_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDDCR_SOC_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_CUR_PVDDCR_CPU1_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDDCR_CPU1_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_CUR_PVDDIO_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDDIO_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_CUR_PVDD11_S3_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDD11_S3_ADDR,
	  PMBUS_READ_IOUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },

	/* VR temperature */
	{ SENSOR_NUM_TEMP_PVDDCR_CPU0_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDDCR_CPU0_ADDR,
	  PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDDCR_SOC_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDDCR_SOC_ADDR,
	  PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDDCR_CPU1_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDDCR_CPU1_ADDR,
	  PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDDIO_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDDIO_ADDR,
	  PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_TEMP_PVDD11_S3_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDD11_S3_ADDR,
	  PMBUS_READ_TEMPERATURE_1, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },

	/* VR power */
	{ SENSOR_NUM_PWR_PVDDCR_CPU0_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDDCR_CPU0_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_PWR_PVDDCR_SOC_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDDCR_SOC_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_PWR_PVDDCR_CPU1_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDDCR_CPU1_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_PWR_PVDDIO_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDDIO_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1], NULL,
	  NULL, NULL },
	{ SENSOR_NUM_PWR_PVDD11_S3_VR, sensor_dev_tps53689, I2C_BUS5, TPS53685_PVDD11_S3_ADDR,
	  PMBUS_READ_POUT, vr_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0], NULL,
	  NULL, NULL },
};

sensor_cfg g788p81u_sensor_config_table[] = {
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_g788p81u, I2C_BUS5, TEMP_HSC_ADDR,
	  G788P81U_REMOTE_TEMP_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &g788p81u_init_args[0] },
};

sensor_cfg mp5990_temp_sensor_config_table[] = {
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_mp5990, I2C_BUS5, MP5990_ADDR, PMBUS_READ_TEMPERATURE_1,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
};

void pal_extend_sensor_config()
{
	uint8_t sensor_count = 0;
	uint8_t hsc_module = get_hsc_module();

	/* Determine which HSC module is used */
	switch (hsc_module) {
	case HSC_MODULE_ADM1278:
		sensor_count = ARRAY_SIZE(adm1278_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(adm1278_sensor_config_table[index]);
		}
		break;
	case HSC_MODULE_LTC4282:
		sensor_count = ARRAY_SIZE(ltc4282_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(ltc4282_sensor_config_table[index]);
		}
		break;
	case HSC_MODULE_MP5990:
		sensor_count = ARRAY_SIZE(mp5990_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(mp5990_sensor_config_table[index]);
		}
		break;
	default:
		LOG_ERR("Unsupported HSC module, HSC module: 0x%x", hsc_module);
		break;
	}

	uint8_t board_revision = get_board_revision();
	uint8_t vr_vender = (board_revision & 0x30) >> 4;
	uint8_t board_stage = (board_revision & 0x0F);

	/* Determine which VR is used */
	switch (vr_vender) {
	case VR_VENDER_INFINEON:
		sensor_count = ARRAY_SIZE(xdpe19283b_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(xdpe19283b_sensor_config_table[index]);
		}
		break;
	case VR_VENDER_MPS:
		sensor_count = ARRAY_SIZE(mp2856gut_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(mp2856gut_sensor_config_table[index]);
		}
		break;
	case VR_VENDER_TI:
		sensor_count = ARRAY_SIZE(tps53689_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(tps53689_sensor_config_table[index]);
		}
		break;
	default:
		LOG_DBG("Main VR source(RAA229621)");
		break;
	}

	/* Determine which temperature sensor is used to monitor HSC temperature */
	if (((board_stage == SYS_BOARD_EVT) && (vr_vender == VR_VENDER_INFINEON)) ||
	    ((board_stage == SYS_BOARD_EVT) && (vr_vender == VR_VENDER_MPS)) ||
	    ((board_stage == SYS_BOARD_DVT) && (vr_vender == VR_VENDER_MPS))) {
		add_sensor_config(g788p81u_sensor_config_table[0]);
	} else if (hsc_module == HSC_MODULE_MP5990) {
		add_sensor_config(mp5990_temp_sensor_config_table[0]);
	} else {
		LOG_DBG("Main HSC temperature sensor(NCT7718W)");
	}

	if (sensor_config_count != sdr_count) {
		LOG_ERR("Extend sensor SDR and config table not match, sdr size: 0x%x, sensor config size: 0x%x",
			sdr_count, sensor_config_count);
	}
}

uint8_t pal_get_extend_sensor_config()
{
	uint8_t extend_sensor_config_size = 0;
	uint8_t hsc_module = get_hsc_module();
	switch (hsc_module) {
	case HSC_MODULE_ADM1278:
		extend_sensor_config_size += ARRAY_SIZE(adm1278_sensor_config_table);
		break;
	case HSC_MODULE_LTC4282:
		extend_sensor_config_size += ARRAY_SIZE(ltc4282_sensor_config_table);
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

	LOG_ERR("Can't find sensor 0x%x last access time", sensor_num);
	return true;
}

const int SENSOR_CONFIG_SIZE = ARRAY_SIZE(plat_sensor_config);

void load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, SENSOR_CONFIG_SIZE * sizeof(sensor_cfg));
	sensor_config_count = SENSOR_CONFIG_SIZE;

	pal_extend_sensor_config();
}
