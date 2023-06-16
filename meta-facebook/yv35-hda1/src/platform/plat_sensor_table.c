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
#include "pmbus.h"
#include "mpro.h"
#include "plat_hook.h"
#include "plat_i2c.h"
#include "plat_class.h"
#include "plat_mctp.h"
#include "plat_power_status.h"

LOG_MODULE_REGISTER(plat_sensor_table);

struct plat_mpro_sensor_mapping mpro_sensor_map[] = {
	{ SENSOR_NUM_TEMP_DIMM_CH0, MPRO_SENSOR_NUM_TMP_DIMM0, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_DIMM_CH1, MPRO_SENSOR_NUM_TMP_DIMM2, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_DIMM_CH2, MPRO_SENSOR_NUM_TMP_DIMM4, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_DIMM_CH3, MPRO_SENSOR_NUM_TMP_DIMM6, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_DIMM_CH4, MPRO_SENSOR_NUM_TMP_DIMM8, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_DIMM_CH5, MPRO_SENSOR_NUM_TMP_DIMM10, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_DIMM_CH6, MPRO_SENSOR_NUM_TMP_DIMM12, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_DIMM_CH7, MPRO_SENSOR_NUM_TMP_DIMM14, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_PCP_VR, MPRO_SENSOR_NUM_TMP_VRD0, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_SOC_VR, MPRO_SENSOR_NUM_TMP_VRD1, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_VDDQ_DDR0123_VR, MPRO_SENSOR_NUM_TMP_VRD2, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_VDDQ_DDR4567_VR, MPRO_SENSOR_NUM_TMP_VRD3, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_D2D_VR, MPRO_SENSOR_NUM_TMP_VRD4, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_RC_DDR0_VR, MPRO_SENSOR_NUM_TMP_VRD5, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_RC_DDR1_VR, MPRO_SENSOR_NUM_TMP_VRD6, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_PCI_D_VR, MPRO_SENSOR_NUM_TMP_VRD7, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_PCI_A_VR, MPRO_SENSOR_NUM_TMP_VRD8, { 1, 0, 0 } },
	{ SENSOR_NUM_TEMP_CPU, MPRO_SENSOR_NUM_TMP_SOC_PKG, { 1, 0, 0 } },

	{ SENSOR_NUM_PWR_PCP_VR, MPRO_SENSOR_NUM_PWR_VRD0, { 1, 0, -3 } },
	{ SENSOR_NUM_PWR_SOC_VR, MPRO_SENSOR_NUM_PWR_VRD1, { 1, 0, -3 } },
	{ SENSOR_NUM_PWR_VDDQ_DDR0123_VR, MPRO_SENSOR_NUM_PWR_VRD2, { 1, 0, -3 } },
	{ SENSOR_NUM_PWR_VDDQ_DDR4567_VR, MPRO_SENSOR_NUM_PWR_VRD3, { 1, 0, -3 } },
	{ SENSOR_NUM_PWR_D2D_VR, MPRO_SENSOR_NUM_PWR_VRD4, { 1, 0, -3 } },
	{ SENSOR_NUM_PWR_RC_DDR0_VR, MPRO_SENSOR_NUM_PWR_VRD5, { 1, 0, -3 } },
	{ SENSOR_NUM_PWR_RC_DDR1_VR, MPRO_SENSOR_NUM_PWR_VRD6, { 1, 0, -3 } },
	{ SENSOR_NUM_PWR_PCI_D_VR, MPRO_SENSOR_NUM_PWR_VRD7, { 1, 0, -3 } },
	{ SENSOR_NUM_PWR_PCI_A_VR, MPRO_SENSOR_NUM_PWR_VRD8, { 1, 0, -3 } },
	{ SENSOR_NUM_PWR_CPU, MPRO_SENSOR_NUM_PWR_SOC_PKG, { 1, 0, -3 } },
	{ SENSOR_NUM_PWR_DIMM_TOTAL, MPRO_SENSOR_NUM_PWR_DRAM, { 1, 0, -3 } },

	{ SENSOR_NUM_VOL_PCP_VR, MPRO_SENSOR_NUM_VOL_VRD0, { 1, 0, -3 } },
	{ SENSOR_NUM_VOL_SOC_VR, MPRO_SENSOR_NUM_VOL_VRD1, { 1, 0, -3 } },
	{ SENSOR_NUM_VOL_VDDQ_DDR0123_VR, MPRO_SENSOR_NUM_VOL_VRD2, { 1, 0, -3 } },
	{ SENSOR_NUM_VOL_VDDQ_DDR4567_VR, MPRO_SENSOR_NUM_VOL_VRD3, { 1, 0, -3 } },
	{ SENSOR_NUM_VOL_D2D_VR, MPRO_SENSOR_NUM_VOL_VRD4, { 1, 0, -3 } },
	{ SENSOR_NUM_VOL_RC_DDR0_VR, MPRO_SENSOR_NUM_VOL_VRD5, { 1, 0, -3 } },
	{ SENSOR_NUM_VOL_RC_DDR1_VR, MPRO_SENSOR_NUM_VOL_VRD6, { 1, 0, -3 } },
	{ SENSOR_NUM_VOL_PCI_D_VR, MPRO_SENSOR_NUM_VOL_VRD7, { 1, 0, -3 } },
	{ SENSOR_NUM_VOL_PCI_A_VR, MPRO_SENSOR_NUM_VOL_VRD8, { 1, 0, -3 } },

	{ SENSOR_NUM_CUR_PCP_VR, MPRO_SENSOR_NUM_CUR_VRD0, { 1, 0, -3 } },
	{ SENSOR_NUM_CUR_SOC_VR, MPRO_SENSOR_NUM_CUR_VRD1, { 1, 0, -3 } },
	{ SENSOR_NUM_CUR_VDDQ_DDR0123_VR, MPRO_SENSOR_NUM_CUR_VRD2, { 1, 0, -3 } },
	{ SENSOR_NUM_CUR_VDDQ_DDR4567_VR, MPRO_SENSOR_NUM_CUR_VRD3, { 1, 0, -3 } },
	{ SENSOR_NUM_CUR_D2D_VR, MPRO_SENSOR_NUM_CUR_VRD4, { 1, 0, -3 } },
	{ SENSOR_NUM_CUR_RC_DDR0_VR, MPRO_SENSOR_NUM_CUR_VRD5, { 1, 0, -3 } },
	{ SENSOR_NUM_CUR_RC_DDR1_VR, MPRO_SENSOR_NUM_CUR_VRD6, { 1, 0, -3 } },
	{ SENSOR_NUM_CUR_PCI_D_VR, MPRO_SENSOR_NUM_CUR_VRD7, { 1, 0, -3 } },
	{ SENSOR_NUM_CUR_PCI_A_VR, MPRO_SENSOR_NUM_CUR_VRD8, { 1, 0, -3 } },
};
const int MPRO_MAP_TAB_SIZE = ARRAY_SIZE(mpro_sensor_map);

sensor_poll_time_cfg diff_poll_time_sensor_table[] = {
	// sensor_number, last_access_time
	{ SENSOR_NUM_VOL_ADC4_P3V_BAT, 0 },
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

	/* NVME */
	{ SENSOR_NUM_TEMP_SSD, sensor_dev_nvme, I2C_BUS2, SSD_ADDR, SSD_TEMP_OFFSET, post_access, 0,
	  0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_nvme_read, &mux_conf_addr_0xe2[1], NULL, NULL, NULL },

	/* adc voltage */
	{ SENSOR_NUM_VOL_ADC0_P12V_STBY, sensor_dev_ast_adc, ADC_PORT0, NONE, NONE, stby_access, 66,
	  10, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC1_SOC_RC_DDR0, sensor_dev_ast_adc, ADC_PORT1, NONE, NONE, dc_access, 1,
	  1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC2_P3V3_STBY, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE, stby_access, 2,
	  1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC3_P0V75_PCP, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC4_P3V_BAT, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE, stby_access, 301,
	  100, SAMPLE_COUNT_DEFAULT, POLL_TIME_BAT3V, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_vol_bat3v_read, NULL, post_vol_bat3v_read, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC5_P0V8_D2D, sensor_dev_ast_adc, ADC_PORT5, NONE, NONE, dc_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },

	{ SENSOR_NUM_VOL_ADC8_EXT_VREF_ADC_S0, sensor_dev_ast_adc, ADC_PORT8, NONE, NONE, dc_access,
	  1, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC9_P3V3_M2, sensor_dev_ast_adc, ADC_PORT9, NONE, NONE, dc_access, 2, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC10_P1V2_STBY, sensor_dev_ast_adc, ADC_PORT10, NONE, NONE, stby_access,
	  1, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC11_SOC_RC_DDR1, sensor_dev_ast_adc, ADC_PORT11, NONE, NONE, stby_access,
	  1, 1, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC12_P12V_S0_DIMM0, sensor_dev_ast_adc, ADC_PORT12, NONE, NONE, dc_access,
	  66, 10, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC13_P12V_S0_DIMM1, sensor_dev_ast_adc, ADC_PORT13, NONE, NONE, dc_access,
	  66, 10, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },
	{ SENSOR_NUM_VOL_ADC14_P5V_STBY, sensor_dev_ast_adc, ADC_PORT14, NONE, NONE, stby_access,
	  711, 200, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ast_adc_init_args[0] },

	/* MPRO - soc */
	{ SENSOR_NUM_TEMP_CPU, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_CPU, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },

	/* MPRO - vrd */
	{ SENSOR_NUM_TEMP_PCP_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_SOC_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_VDDQ_DDR0123_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_VDDQ_DDR4567_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_D2D_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_RC_DDR0_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_RC_DDR1_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PCI_D_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_PCI_A_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },

	{ SENSOR_NUM_PWR_PCP_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_SOC_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_VDDQ_DDR0123_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_VDDQ_DDR4567_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_D2D_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_RC_DDR0_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_RC_DDR1_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PCI_D_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_PCI_A_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },

	{ SENSOR_NUM_VOL_PCP_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_SOC_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_VDDQ_DDR0123_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_VDDQ_DDR4567_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_D2D_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_RC_DDR0_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_RC_DDR1_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PCI_D_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_VOL_PCI_A_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },

	{ SENSOR_NUM_CUR_PCP_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_SOC_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_VDDQ_DDR0123_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_VDDQ_DDR4567_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_D2D_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_RC_DDR0_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_RC_DDR1_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PCI_D_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_CUR_PCI_A_VR, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },

	/* MPRO - dimm */
	{ SENSOR_NUM_TEMP_DIMM_CH0, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_CH1, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_CH2, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_CH3, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_CH4, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_CH5, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_CH6, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_DIMM_CH7, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_PWR_DIMM_TOTAL, sensor_dev_mpro, MCTP_EID_MPRO, NONE, NONE, mpro_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, NULL },
};

sensor_cfg adm1278_sensor_config_table[] = {
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_adm1278, I2C_BUS2, ADM1278_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_adm1278, I2C_BUS2, ADM1278_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_adm1278_cur_read, NULL, &adm1278_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_adm1278, I2C_BUS2, ADM1278_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_adm1278_pwr_read, NULL, &adm1278_init_args[0] },
};

sensor_cfg ltc4282_sensor_config_table[] = {
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_ltc4282, I2C_BUS2, LTC4282_ADDR, LTC4282_VSOURCE_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &ltc4282_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_ltc4282, I2C_BUS2, LTC4282_ADDR, LTC4282_VSENSE_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_ltc4282_cur_read, NULL, &ltc4282_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_ltc4282, I2C_BUS2, LTC4282_ADDR, LTC4282_POWER_OFFSET,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_ltc4282_pwr_read, NULL, &ltc4282_init_args[0] },
};

sensor_cfg mp5990_sensor_config_table[] = {
	{ SENSOR_NUM_VOL_HSCIN, sensor_dev_mp5990, I2C_BUS2, MP5990_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_CUR_HSCOUT, sensor_dev_mp5990, I2C_BUS2, MP5990_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_mp5990_cur_read, NULL, &mp5990_init_args[0] },
	{ SENSOR_NUM_PWR_HSCIN, sensor_dev_mp5990, I2C_BUS2, MP5990_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, post_mp5990_pwr_read, NULL, &mp5990_init_args[0] },
};

sensor_cfg mp5990_temp_sensor_config_table[] = {
	{ SENSOR_NUM_TEMP_HSC, sensor_dev_mp5990, I2C_BUS2, MP5990_ADDR, PMBUS_READ_TEMPERATURE_1,
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
		LOG_INF("HSC vendor: ADM1278");
		sensor_count = ARRAY_SIZE(adm1278_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(adm1278_sensor_config_table[index]);
		}
		break;
	case HSC_MODULE_LTC4282:
		LOG_INF("HSC vendor: LTC4282");
		sensor_count = ARRAY_SIZE(ltc4282_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(ltc4282_sensor_config_table[index]);
		}
		break;
	case HSC_MODULE_MP5990:
		LOG_INF("HSC vendor: MP5990");
		sensor_count = ARRAY_SIZE(mp5990_sensor_config_table);
		for (int index = 0; index < sensor_count; index++) {
			add_sensor_config(mp5990_sensor_config_table[index]);
		}
		/* MP5990 can read HSC temperature */
		add_sensor_config(mp5990_temp_sensor_config_table[0]);
		break;
	default:
		LOG_ERR("Unsupported HSC module, HSC module: 0x%x", hsc_module);
		break;
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
		LOG_INF("HSC vendor: ADM1278");
		extend_sensor_config_size += ARRAY_SIZE(adm1278_sensor_config_table);
		break;
	case HSC_MODULE_LTC4282:
		LOG_INF("HSC vendor: LTC4282");
		extend_sensor_config_size += ARRAY_SIZE(ltc4282_sensor_config_table);
		break;
	case HSC_MODULE_MP5990:
		LOG_INF("HSC vendor: MP5990");
		extend_sensor_config_size += ARRAY_SIZE(mp5990_sensor_config_table);
		/* MP5990 can read HSC temperature */
		extend_sensor_config_size += ARRAY_SIZE(mp5990_temp_sensor_config_table);
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
