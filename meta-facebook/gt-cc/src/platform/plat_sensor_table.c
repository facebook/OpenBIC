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

#include "plat_sensor_table.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <logging/log.h>

#include "sensor.h"
#include "ast_adc.h"
#include "intel_peci.h"
#include "hal_gpio.h"
#include "plat_gpio.h"
#include "plat_hook.h"
#include "plat_i2c.h"
#include "power_status.h"
#include "pmbus.h"
#include "tmp431.h"
#include "libutil.h"
#include "i2c-mux-tca9548.h"
#include "isl28022.h"
#include "pex89000.h"
#include "util_sys.h"
#include "plat_class.h"

LOG_MODULE_REGISTER(plat_sensor_table);

static int check_vr_type(void);
static void load_hsc_sensor_table(void);
static void load_vr_sensor_table(void);
static void load_power_ic_sensor_table(void);
static void change_p1v8_sensor_i2c_addr(void);
bool e1s_access(uint8_t sensor_num);
bool nic_access(uint8_t sensor_num);

sensor_cfg plat_sensor_config[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, cache, cache_status, mux_ADDRess, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

	/* NIC 0-7 temperature sensor */
	{ SENSOR_NUM_TEMP_NIC_0, sensor_dev_tmp75, I2C_BUS1, NIC_ADDR, NIC_TEMP_OFFSET, nic_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_NIC_1, sensor_dev_tmp75, I2C_BUS2, NIC_ADDR, NIC_TEMP_OFFSET, nic_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_NIC_2, sensor_dev_tmp75, I2C_BUS3, NIC_ADDR, NIC_TEMP_OFFSET, nic_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_NIC_3, sensor_dev_tmp75, I2C_BUS4, NIC_ADDR, NIC_TEMP_OFFSET, nic_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_NIC_4, sensor_dev_tmp75, I2C_BUS11, NIC_ADDR, NIC_TEMP_OFFSET, nic_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_NIC_5, sensor_dev_tmp75, I2C_BUS12, NIC_ADDR, NIC_TEMP_OFFSET, nic_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_NIC_6, sensor_dev_tmp75, I2C_BUS13, NIC_ADDR, NIC_TEMP_OFFSET, nic_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	{ SENSOR_NUM_TEMP_NIC_7, sensor_dev_tmp75, I2C_BUS14, NIC_ADDR, NIC_TEMP_OFFSET, nic_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },

	/* ADC voltage */
	{ SENSOR_NUM_BB_P12V_AUX, sensor_dev_ast_adc, ADC_PORT0, NONE, NONE, stby_access, 1780, 200,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_BB_P5V_AUX, sensor_dev_ast_adc, ADC_PORT1, NONE, NONE, stby_access, 736, 200,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_BB_P3V3_AUX, sensor_dev_ast_adc, ADC_PORT2, NONE, NONE, stby_access, 487, 200,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_BB_P1V2_AUX, sensor_dev_ast_adc, ADC_PORT3, NONE, NONE, stby_access, 1, 1,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_BB_P3V3, sensor_dev_ast_adc, ADC_PORT4, NONE, NONE, stby_access, 487, 200,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_BB_P1V8_PEX0, sensor_dev_ast_adc, ADC_PORT5, NONE, NONE, stby_access, 4, 3,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_BB_P1V8_PEX1, sensor_dev_ast_adc, ADC_PORT6, NONE, NONE, stby_access, 4, 3,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_BB_P1V8_PEX2, sensor_dev_ast_adc, ADC_PORT15, NONE, NONE, stby_access, 4, 3,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },
	{ SENSOR_NUM_BB_P1V8_PEX3, sensor_dev_ast_adc, ADC_PORT14, NONE, NONE, stby_access, 4, 3,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  NULL, NULL, NULL, NULL, &adc_asd_init_args[0] },

	/* SYSTEM INLET TEMP */
	{ SENSOR_NUM_SYSTEM_INLET_TEMP, sensor_dev_tmp75, I2C_BUS5, SYSTEM_INLET_TEMP_ADDR,
	  TMP75_TEMP_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL, NULL },
	/* SYSTEM OUTLET TEMP LEFT */
	{ SENSOR_NUM_OUTLET_TEMP_L1, sensor_dev_tmp75, I2C_BUS6, SYSTEM_OUTLET_TEMP_L1_ADDR,
	  TMP75_TEMP_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[2],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_OUTLET_TEMP_L2, sensor_dev_tmp75, I2C_BUS6, SYSTEM_OUTLET_TEMP_L2_ADDR,
	  TMP75_TEMP_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[2],
	  post_i2c_bus_read, NULL, NULL },
	/* SYSTEM OUTLET TEMP RIGHT */
	{ SENSOR_NUM_OUTLET_TEMP_R1, sensor_dev_tmp75, I2C_BUS6, SYSTEM_OUTLET_TEMP_R1_ADDR,
	  TMP75_TEMP_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[2],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_OUTLET_TEMP_R2, sensor_dev_tmp75, I2C_BUS6, SYSTEM_OUTLET_TEMP_R2_ADDR,
	  TMP75_TEMP_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[2],
	  post_i2c_bus_read, NULL, NULL },

	/* SSD 0-15 temperature */
	{ SENSOR_NUM_TEMP_E1S_0, sensor_dev_nvme, I2C_BUS9, SSD_COMMON_ADDR, SSD_OFFSET, e1s_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0], post_i2c_bus_read, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_E1S_1, sensor_dev_nvme, I2C_BUS9, SSD_COMMON_ADDR, SSD_OFFSET, e1s_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1], post_i2c_bus_read, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_E1S_2, sensor_dev_nvme, I2C_BUS9, SSD_COMMON_ADDR, SSD_OFFSET, e1s_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[2], post_i2c_bus_read, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_E1S_3, sensor_dev_nvme, I2C_BUS9, SSD_COMMON_ADDR, SSD_OFFSET, e1s_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3], post_i2c_bus_read, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_E1S_4, sensor_dev_nvme, I2C_BUS9, SSD_COMMON_ADDR, SSD_OFFSET, e1s_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4], post_i2c_bus_read, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_E1S_5, sensor_dev_nvme, I2C_BUS9, SSD_COMMON_ADDR, SSD_OFFSET, e1s_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[5], post_i2c_bus_read, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_E1S_6, sensor_dev_nvme, I2C_BUS9, SSD_COMMON_ADDR, SSD_OFFSET, e1s_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[6], post_i2c_bus_read, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_E1S_7, sensor_dev_nvme, I2C_BUS9, SSD_COMMON_ADDR, SSD_OFFSET, e1s_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[7], post_i2c_bus_read, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_E1S_8, sensor_dev_nvme, I2C_BUS9, SSD_COMMON_ADDR, SSD_OFFSET, e1s_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe2[0], post_i2c_bus_read, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_E1S_9, sensor_dev_nvme, I2C_BUS9, SSD_COMMON_ADDR, SSD_OFFSET, e1s_access,
	  0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe2[1], post_i2c_bus_read, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_E1S_10, sensor_dev_nvme, I2C_BUS9, SSD_COMMON_ADDR, SSD_OFFSET,
	  e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe2[2], post_i2c_bus_read, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_E1S_11, sensor_dev_nvme, I2C_BUS9, SSD_COMMON_ADDR, SSD_OFFSET,
	  e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe2[3], post_i2c_bus_read, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_E1S_12, sensor_dev_nvme, I2C_BUS9, SSD_COMMON_ADDR, SSD_OFFSET,
	  e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe2[4], post_i2c_bus_read, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_E1S_13, sensor_dev_nvme, I2C_BUS9, SSD_COMMON_ADDR, SSD_OFFSET,
	  e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe2[5], post_i2c_bus_read, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_E1S_14, sensor_dev_nvme, I2C_BUS9, SSD_COMMON_ADDR, SSD_OFFSET,
	  e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe2[6], post_i2c_bus_read, NULL,
	  NULL },
	{ SENSOR_NUM_TEMP_E1S_15, sensor_dev_nvme, I2C_BUS9, SSD_COMMON_ADDR, SSD_OFFSET,
	  e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe2[7], post_i2c_bus_read, NULL,
	  NULL },
};

sensor_cfg evt_pex_sensor_config_table[] = {
	/* PEX on-chip temperature */
	{ SENSOR_NUM_BB_TEMP_PEX_0, sensor_dev_pex89000, I2C_BUS10, EVT_PEX_SWITCH_I2C_ADDR,
	  PEX_TEMP, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_pex89000_read,
	  &pex89000_pre_read_args[0], post_i2c_bus_read, NULL, &pex_sensor_init_args[0] },
	{ SENSOR_NUM_BB_TEMP_PEX_1, sensor_dev_pex89000, I2C_BUS10, EVT_PEX_SWITCH_I2C_ADDR,
	  PEX_TEMP, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_pex89000_read,
	  &pex89000_pre_read_args[1], post_i2c_bus_read, NULL, &pex_sensor_init_args[1] },
	{ SENSOR_NUM_BB_TEMP_PEX_2, sensor_dev_pex89000, I2C_BUS10, EVT_PEX_SWITCH_I2C_ADDR,
	  PEX_TEMP, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_pex89000_read,
	  &pex89000_pre_read_args[2], post_i2c_bus_read, NULL, &pex_sensor_init_args[2] },
	{ SENSOR_NUM_BB_TEMP_PEX_3, sensor_dev_pex89000, I2C_BUS10, EVT_PEX_SWITCH_I2C_ADDR,
	  PEX_TEMP, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_pex89000_read,
	  &pex89000_pre_read_args[3], post_i2c_bus_read, NULL, &pex_sensor_init_args[3] },
};

sensor_cfg dvt_pex_sensor_config_table[] = {
	/* PEX on-chip temperature */
	{ SENSOR_NUM_BB_TEMP_PEX_0, sensor_dev_pex89000, I2C_BUS10, DVT_PEX_SWITCH_0_I2C_ADDR,
	  PEX_TEMP, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &pex_sensor_init_args[0] },
	{ SENSOR_NUM_BB_TEMP_PEX_1, sensor_dev_pex89000, I2C_BUS10, DVT_PEX_SWITCH_1_I2C_ADDR,
	  PEX_TEMP, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &pex_sensor_init_args[1] },
	{ SENSOR_NUM_BB_TEMP_PEX_2, sensor_dev_pex89000, I2C_BUS10, DVT_PEX_SWITCH_2_I2C_ADDR,
	  PEX_TEMP, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &pex_sensor_init_args[2] },
	{ SENSOR_NUM_BB_TEMP_PEX_3, sensor_dev_pex89000, I2C_BUS10, DVT_PEX_SWITCH_3_I2C_ADDR,
	  PEX_TEMP, is_dc_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &pex_sensor_init_args[3] },
};

sensor_cfg mp5990_hsc_sensor_config_table[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, sample_count, cache, cache_status, mux_address, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */

	/* HSC */
	{ SENSOR_NUM_TEMP_PDB_HSC, sensor_dev_mp5990, I2C_BUS6, HSC_MP5990_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[6],
	  post_i2c_bus_read, NULL, &mp5990_hsc_init_args[0] },
	{ SENSOR_NUM_VOUT_PDB_HSC, sensor_dev_mp5990, I2C_BUS6, HSC_MP5990_ADDR, PMBUS_READ_VIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[6], post_i2c_bus_read, NULL,
	  &mp5990_hsc_init_args[0] },
	{ SENSOR_NUM_IOUT_PDB_HSC, sensor_dev_mp5990, I2C_BUS6, HSC_MP5990_ADDR, PMBUS_READ_IOUT,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[6], post_mp5990_read, NULL,
	  &mp5990_hsc_init_args[0] },
	{ SENSOR_NUM_POUT_PDB_HSC, sensor_dev_mp5990, I2C_BUS6, HSC_MP5990_ADDR, PMBUS_READ_PIN,
	  stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0,
	  SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[6], post_mp5990_read, NULL,
	  &mp5990_hsc_init_args[0] },
};

sensor_cfg ltc4282_hsc_sensor_config_table[] = {
	/* number,                  type,       port,      address,      offset,
    access check arg0, arg1, sample_count, cache, cache_status, mux_address, mux_offset,
    pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */
	/* HSC */
	/* TBD: Because LTC4282 have no register for temperature, confirming with EE 
          whether has temperature sensor in front of the HSC */
	{ SENSOR_NUM_TEMP_PDB_HSC, sensor_dev_nct7718w, I2C_BUS6, HSC_TEMP_NCT7718W_ADDR,
	  NCT7718W_LOCAL_TEMP_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_VOUT_PDB_HSC, sensor_dev_ltc4282, I2C_BUS6, HSC_LTC4282_ADDR,
	  LTC4282_VSOURCE_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[6],
	  post_ltc4282_read, NULL, &ltc4282_hsc_init_args[0] },
	{ SENSOR_NUM_IOUT_PDB_HSC, sensor_dev_ltc4282, I2C_BUS6, HSC_LTC4282_ADDR,
	  LTC4282_VSENSE_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[6],
	  post_ltc4282_read, NULL, &ltc4282_hsc_init_args[0] },
	{ SENSOR_NUM_POUT_PDB_HSC, sensor_dev_ltc4282, I2C_BUS6, HSC_LTC4282_ADDR,
	  LTC4282_POWER_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[6],
	  post_ltc4282_read, NULL, &ltc4282_hsc_init_args[0] },
};

sensor_cfg isl69259_vr_sensor_config_table[] = {
	/* number,                  type,       port,      address,      offset,
    access check arg0, arg1, sample_count, cache, cache_status, mux_address, mux_offset,
    pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */
	/* PEX 0 */
	{ SENSOR_NUM_TEMP_PEX_0, sensor_dev_isl69259, I2C_BUS6, PEX_0_1_P0V8_VR_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_VOLT_PEX_0, sensor_dev_isl69259, I2C_BUS6, PEX_0_1_P0V8_VR_ADDR,
	  PMBUS_READ_VOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_IOUT_PEX_0, sensor_dev_isl69259, I2C_BUS6, PEX_0_1_P0V8_VR_ADDR,
	  PMBUS_READ_IOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_POUT_PEX_0, sensor_dev_isl69259, I2C_BUS6, PEX_0_1_P0V8_VR_ADDR,
	  PMBUS_READ_POUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0],
	  post_i2c_bus_read, NULL, NULL },
	/* PEX 1 */
	{ SENSOR_NUM_TEMP_PEX_1, sensor_dev_isl69259, I2C_BUS6, PEX_0_1_P0V8_VR_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_VOLT_PEX_1, sensor_dev_isl69259, I2C_BUS6, PEX_0_1_P0V8_VR_ADDR,
	  PMBUS_READ_VOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_IOUT_PEX_1, sensor_dev_isl69259, I2C_BUS6, PEX_0_1_P0V8_VR_ADDR,
	  PMBUS_READ_IOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_POUT_PEX_1, sensor_dev_isl69259, I2C_BUS6, PEX_0_1_P0V8_VR_ADDR,
	  PMBUS_READ_POUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1],
	  post_i2c_bus_read, NULL, NULL },
	/* PEX 2 */
	{ SENSOR_NUM_TEMP_PEX_2, sensor_dev_isl69259, I2C_BUS6, PEX_2_3_P0V8_VR_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[2],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_VOLT_PEX_2, sensor_dev_isl69259, I2C_BUS6, PEX_2_3_P0V8_VR_ADDR,
	  PMBUS_READ_VOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[2],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_IOUT_PEX_2, sensor_dev_isl69259, I2C_BUS6, PEX_2_3_P0V8_VR_ADDR,
	  PMBUS_READ_IOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[2],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_POUT_PEX_2, sensor_dev_isl69259, I2C_BUS6, PEX_2_3_P0V8_VR_ADDR,
	  PMBUS_READ_POUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[2],
	  post_i2c_bus_read, NULL, NULL },
	/* PEX 3 */
	{ SENSOR_NUM_TEMP_PEX_3, sensor_dev_isl69259, I2C_BUS6, PEX_2_3_P0V8_VR_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[3],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_VOLT_PEX_3, sensor_dev_isl69259, I2C_BUS6, PEX_2_3_P0V8_VR_ADDR,
	  PMBUS_READ_VOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[3],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_IOUT_PEX_3, sensor_dev_isl69259, I2C_BUS6, PEX_2_3_P0V8_VR_ADDR,
	  PMBUS_READ_IOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[3],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_POUT_PEX_3, sensor_dev_isl69259, I2C_BUS6, PEX_2_3_P0V8_VR_ADDR,
	  PMBUS_READ_POUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[3],
	  post_i2c_bus_read, NULL, NULL },
};

sensor_cfg xdpe12284_vr_sensor_config_table[] = {
	/* number,                  type,       port,      address,      offset,
    access check arg0, arg1, sample_count, cache, cache_status, mux_address, mux_offset,
    pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */
	/* PEX 0 */
	{ SENSOR_NUM_TEMP_PEX_0, sensor_dev_xdpe12284c, I2C_BUS6, PEX_0_1_P0V8_VR_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_VOLT_PEX_0, sensor_dev_xdpe12284c, I2C_BUS6, PEX_0_1_P0V8_VR_ADDR,
	  PMBUS_READ_VOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_IOUT_PEX_0, sensor_dev_xdpe12284c, I2C_BUS6, PEX_0_1_P0V8_VR_ADDR,
	  PMBUS_READ_IOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_POUT_PEX_0, sensor_dev_xdpe12284c, I2C_BUS6, PEX_0_1_P0V8_VR_ADDR,
	  PMBUS_READ_POUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[0],
	  post_i2c_bus_read, NULL, NULL },
	/* PEX 1 */
	{ SENSOR_NUM_TEMP_PEX_1, sensor_dev_xdpe12284c, I2C_BUS6, PEX_0_1_P0V8_VR_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_VOLT_PEX_1, sensor_dev_xdpe12284c, I2C_BUS6, PEX_0_1_P0V8_VR_ADDR,
	  PMBUS_READ_VOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_IOUT_PEX_1, sensor_dev_xdpe12284c, I2C_BUS6, PEX_0_1_P0V8_VR_ADDR,
	  PMBUS_READ_IOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_POUT_PEX_1, sensor_dev_xdpe12284c, I2C_BUS6, PEX_0_1_P0V8_VR_ADDR,
	  PMBUS_READ_POUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[1],
	  post_i2c_bus_read, NULL, NULL },
	/* PEX 2 */
	{ SENSOR_NUM_TEMP_PEX_2, sensor_dev_xdpe12284c, I2C_BUS6, PEX_2_3_P0V8_VR_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[2],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_VOLT_PEX_2, sensor_dev_xdpe12284c, I2C_BUS6, PEX_2_3_P0V8_VR_ADDR,
	  PMBUS_READ_VOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[2],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_IOUT_PEX_2, sensor_dev_xdpe12284c, I2C_BUS6, PEX_2_3_P0V8_VR_ADDR,
	  PMBUS_READ_IOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[2],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_POUT_PEX_2, sensor_dev_xdpe12284c, I2C_BUS6, PEX_2_3_P0V8_VR_ADDR,
	  PMBUS_READ_POUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[2],
	  post_i2c_bus_read, NULL, NULL },
	/* PEX 3 */
	{ SENSOR_NUM_TEMP_PEX_3, sensor_dev_xdpe12284c, I2C_BUS6, PEX_2_3_P0V8_VR_ADDR,
	  PMBUS_READ_TEMPERATURE_1, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[3],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_VOLT_PEX_3, sensor_dev_xdpe12284c, I2C_BUS6, PEX_2_3_P0V8_VR_ADDR,
	  PMBUS_READ_VOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[3],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_IOUT_PEX_3, sensor_dev_xdpe12284c, I2C_BUS6, PEX_2_3_P0V8_VR_ADDR,
	  PMBUS_READ_IOUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[3],
	  post_i2c_bus_read, NULL, NULL },
	{ SENSOR_NUM_P0V8_POUT_PEX_3, sensor_dev_xdpe12284c, I2C_BUS6, PEX_2_3_P0V8_VR_ADDR,
	  PMBUS_READ_POUT, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_vr_read, &vr_pre_read_args[3],
	  post_i2c_bus_read, NULL, NULL },
};

sensor_cfg isl28022_power_monitor_sensor_config_table[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, cache, cache_status, mux_ADDRess, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */
	/* NIC 0 */
	{ SENSOR_NUM_VOLT_NIC_0, sensor_dev_isl28022, I2C_BUS6, NIC_0_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[0] },
	{ SENSOR_NUM_IOUT_NIC_0, sensor_dev_isl28022, I2C_BUS6, NIC_0_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[0] },
	{ SENSOR_NUM_POUT_NIC_0, sensor_dev_isl28022, I2C_BUS6, NIC_0_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[0] },
	/* NIC 1 */
	{ SENSOR_NUM_VOLT_NIC_1, sensor_dev_isl28022, I2C_BUS6, NIC_1_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[1] },
	{ SENSOR_NUM_IOUT_NIC_1, sensor_dev_isl28022, I2C_BUS6, NIC_1_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[1] },
	{ SENSOR_NUM_POUT_NIC_1, sensor_dev_isl28022, I2C_BUS6, NIC_1_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[1] },
	/* NIC 2 */
	{ SENSOR_NUM_VOLT_NIC_2, sensor_dev_isl28022, I2C_BUS6, NIC_2_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[2] },
	{ SENSOR_NUM_IOUT_NIC_2, sensor_dev_isl28022, I2C_BUS6, NIC_2_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[2] },
	{ SENSOR_NUM_POUT_NIC_2, sensor_dev_isl28022, I2C_BUS6, NIC_2_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[2] },
	/* NIC 3 */
	{ SENSOR_NUM_VOLT_NIC_3, sensor_dev_isl28022, I2C_BUS6, NIC_3_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[3] },
	{ SENSOR_NUM_IOUT_NIC_3, sensor_dev_isl28022, I2C_BUS6, NIC_3_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[3] },
	{ SENSOR_NUM_POUT_NIC_3, sensor_dev_isl28022, I2C_BUS6, NIC_3_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[3] },
	/* NIC 4 */
	{ SENSOR_NUM_VOLT_NIC_4, sensor_dev_isl28022, I2C_BUS6, NIC_4_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[4] },
	{ SENSOR_NUM_IOUT_NIC_4, sensor_dev_isl28022, I2C_BUS6, NIC_4_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[4] },
	{ SENSOR_NUM_POUT_NIC_4, sensor_dev_isl28022, I2C_BUS6, NIC_4_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[4] },
	/* NIC 5 */
	{ SENSOR_NUM_VOLT_NIC_5, sensor_dev_isl28022, I2C_BUS6, NIC_5_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[5] },
	{ SENSOR_NUM_IOUT_NIC_5, sensor_dev_isl28022, I2C_BUS6, NIC_5_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[5] },
	{ SENSOR_NUM_POUT_NIC_5, sensor_dev_isl28022, I2C_BUS6, NIC_5_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[5] },
	/* NIC 6 */
	{ SENSOR_NUM_VOLT_NIC_6, sensor_dev_isl28022, I2C_BUS6, NIC_6_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[6] },
	{ SENSOR_NUM_IOUT_NIC_6, sensor_dev_isl28022, I2C_BUS6, NIC_6_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[6] },
	{ SENSOR_NUM_POUT_NIC_6, sensor_dev_isl28022, I2C_BUS6, NIC_6_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[6] },
	/* NIC 7 */
	{ SENSOR_NUM_VOLT_NIC_7, sensor_dev_isl28022, I2C_BUS6, NIC_7_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[7] },
	{ SENSOR_NUM_IOUT_NIC_7, sensor_dev_isl28022, I2C_BUS6, NIC_7_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[7] },
	{ SENSOR_NUM_POUT_NIC_7, sensor_dev_isl28022, I2C_BUS6, NIC_7_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &isl28022_nic_sensor_init_args[7] },
	/* PEX 0 P1V25 */
	{ SENSOR_NUM_P1V25_VOLT_PEX_0, sensor_dev_isl28022, I2C_BUS6,
	  PEX_0_P1V25_POWER_MONITOR_ADDR, ISL28022_BUS_VOLTAGE_REG, stby_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_i2c_bus_read, &mux_conf_addr_0xe0[1], post_i2c_bus_read, NULL,
	  &isl28022_pex_p1v25_sensor_init_args[0] },
	{ SENSOR_NUM_P1V25_IOUT_PEX_0, sensor_dev_isl28022, I2C_BUS6,
	  PEX_0_P1V25_POWER_MONITOR_ADDR, ISL28022_CURRENT_REG, stby_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_i2c_bus_read, &mux_conf_addr_0xe0[1], post_i2c_bus_read, NULL,
	  &isl28022_pex_p1v25_sensor_init_args[0] },
	{ SENSOR_NUM_P1V25_POUT_PEX_0, sensor_dev_isl28022, I2C_BUS6,
	  PEX_0_P1V25_POWER_MONITOR_ADDR, ISL28022_POWER_REG, stby_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_i2c_bus_read, &mux_conf_addr_0xe0[1], post_i2c_bus_read, NULL,
	  &isl28022_pex_p1v25_sensor_init_args[0] },
	/* PEX 1 P1V25 */
	{ SENSOR_NUM_P1V25_VOLT_PEX_1, sensor_dev_isl28022, I2C_BUS6,
	  PEX_1_P1V25_POWER_MONITOR_ADDR, ISL28022_BUS_VOLTAGE_REG, stby_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_i2c_bus_read, &mux_conf_addr_0xe0[1], post_i2c_bus_read, NULL,
	  &isl28022_pex_p1v25_sensor_init_args[1] },
	{ SENSOR_NUM_P1V25_IOUT_PEX_1, sensor_dev_isl28022, I2C_BUS6,
	  PEX_1_P1V25_POWER_MONITOR_ADDR, ISL28022_CURRENT_REG, stby_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_i2c_bus_read, &mux_conf_addr_0xe0[1], post_i2c_bus_read, NULL,
	  &isl28022_pex_p1v25_sensor_init_args[1] },
	{ SENSOR_NUM_P1V25_POUT_PEX_1, sensor_dev_isl28022, I2C_BUS6,
	  PEX_1_P1V25_POWER_MONITOR_ADDR, ISL28022_POWER_REG, stby_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_i2c_bus_read, &mux_conf_addr_0xe0[1], post_i2c_bus_read, NULL,
	  &isl28022_pex_p1v25_sensor_init_args[1] },
	/* PEX 2 P1V25 */
	{ SENSOR_NUM_P1V25_VOLT_PEX_2, sensor_dev_isl28022, I2C_BUS6,
	  PEX_2_P1V25_POWER_MONITOR_ADDR, ISL28022_BUS_VOLTAGE_REG, stby_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_i2c_bus_read, &mux_conf_addr_0xe0[1], post_i2c_bus_read, NULL,
	  &isl28022_pex_p1v25_sensor_init_args[2] },
	{ SENSOR_NUM_P1V25_IOUT_PEX_2, sensor_dev_isl28022, I2C_BUS6,
	  PEX_2_P1V25_POWER_MONITOR_ADDR, ISL28022_CURRENT_REG, stby_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_i2c_bus_read, &mux_conf_addr_0xe0[1], post_i2c_bus_read, NULL,
	  &isl28022_pex_p1v25_sensor_init_args[2] },
	{ SENSOR_NUM_P1V25_POUT_PEX_2, sensor_dev_isl28022, I2C_BUS6,
	  PEX_2_P1V25_POWER_MONITOR_ADDR, ISL28022_POWER_REG, stby_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_i2c_bus_read, &mux_conf_addr_0xe0[1], post_i2c_bus_read, NULL,
	  &isl28022_pex_p1v25_sensor_init_args[2] },
	/* PEX 3 P1V25 */
	{ SENSOR_NUM_P1V25_VOLT_PEX_3, sensor_dev_isl28022, I2C_BUS6,
	  PEX_3_P1V25_POWER_MONITOR_ADDR, ISL28022_BUS_VOLTAGE_REG, stby_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_i2c_bus_read, &mux_conf_addr_0xe0[1], post_i2c_bus_read, NULL,
	  &isl28022_pex_p1v25_sensor_init_args[3] },
	{ SENSOR_NUM_P1V25_IOUT_PEX_3, sensor_dev_isl28022, I2C_BUS6,
	  PEX_3_P1V25_POWER_MONITOR_ADDR, ISL28022_CURRENT_REG, stby_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_i2c_bus_read, &mux_conf_addr_0xe0[1], post_i2c_bus_read, NULL,
	  &isl28022_pex_p1v25_sensor_init_args[3] },
	{ SENSOR_NUM_P1V25_POUT_PEX_3, sensor_dev_isl28022, I2C_BUS6,
	  PEX_3_P1V25_POWER_MONITOR_ADDR, ISL28022_POWER_REG, stby_access, 0, 0,
	  SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS,
	  pre_i2c_bus_read, &mux_conf_addr_0xe0[1], post_i2c_bus_read, NULL,
	  &isl28022_pex_p1v25_sensor_init_args[3] },
	/* PEX P1V8 */
	{ SENSOR_NUM_P1V8_VOLT_PEX, sensor_dev_isl28022, I2C_BUS6, PEX_P1V8_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &isl28022_pex_p1v8_sensor_init_args[0] },
	{ SENSOR_NUM_P1V8_IOUT_PEX, sensor_dev_isl28022, I2C_BUS6, PEX_P1V8_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &isl28022_pex_p1v8_sensor_init_args[0] },
	{ SENSOR_NUM_P1V8_POUT_PEX, sensor_dev_isl28022, I2C_BUS6, PEX_P1V8_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &isl28022_pex_p1v8_sensor_init_args[0] },
	/* SSD 0 */
	{ SENSOR_NUM_VOLT_E1S_0, sensor_dev_isl28022, I2C_BUS6, SSD_0_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[0] },
	{ SENSOR_NUM_CURR_E1S_0, sensor_dev_isl28022, I2C_BUS6, SSD_0_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[0] },
	{ SENSOR_NUM_POUT_E1S_0, sensor_dev_isl28022, I2C_BUS6, SSD_0_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[0] },
	/* SSD 1 */
	{ SENSOR_NUM_VOLT_E1S_1, sensor_dev_isl28022, I2C_BUS6, SSD_1_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[1] },
	{ SENSOR_NUM_CURR_E1S_1, sensor_dev_isl28022, I2C_BUS6, SSD_1_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[1] },
	{ SENSOR_NUM_POUT_E1S_1, sensor_dev_isl28022, I2C_BUS6, SSD_1_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[1] },
	/* SSD 2 */
	{ SENSOR_NUM_VOLT_E1S_2, sensor_dev_isl28022, I2C_BUS6, SSD_2_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[2] },
	{ SENSOR_NUM_CURR_E1S_2, sensor_dev_isl28022, I2C_BUS6, SSD_2_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[2] },
	{ SENSOR_NUM_POUT_E1S_2, sensor_dev_isl28022, I2C_BUS6, SSD_2_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[2] },
	/* SSD 3 */
	{ SENSOR_NUM_VOLT_E1S_3, sensor_dev_isl28022, I2C_BUS6, SSD_3_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[3] },
	{ SENSOR_NUM_CURR_E1S_3, sensor_dev_isl28022, I2C_BUS6, SSD_3_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[3] },
	{ SENSOR_NUM_POUT_E1S_3, sensor_dev_isl28022, I2C_BUS6, SSD_3_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[3] },
	/* SSD4 */
	{ SENSOR_NUM_VOLT_E1S_4, sensor_dev_isl28022, I2C_BUS6, SSD_4_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[4] },
	{ SENSOR_NUM_CURR_E1S_4, sensor_dev_isl28022, I2C_BUS6, SSD_4_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[4] },
	{ SENSOR_NUM_POUT_E1S_4, sensor_dev_isl28022, I2C_BUS6, SSD_4_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[4] },
	/* SSD5 */
	{ SENSOR_NUM_VOLT_E1S_5, sensor_dev_isl28022, I2C_BUS6, SSD_5_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[5] },
	{ SENSOR_NUM_CURR_E1S_5, sensor_dev_isl28022, I2C_BUS6, SSD_5_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[5] },
	{ SENSOR_NUM_POUT_E1S_5, sensor_dev_isl28022, I2C_BUS6, SSD_5_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[5] },
	/* SSD6 */
	{ SENSOR_NUM_VOLT_E1S_6, sensor_dev_isl28022, I2C_BUS6, SSD_6_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[6] },
	{ SENSOR_NUM_CURR_E1S_6, sensor_dev_isl28022, I2C_BUS6, SSD_6_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[6] },
	{ SENSOR_NUM_POUT_E1S_6, sensor_dev_isl28022, I2C_BUS6, SSD_6_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[6] },
	/* SSD7 */
	{ SENSOR_NUM_VOLT_E1S_7, sensor_dev_isl28022, I2C_BUS6, SSD_7_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[7] },
	{ SENSOR_NUM_CURR_E1S_7, sensor_dev_isl28022, I2C_BUS6, SSD_7_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[7] },
	{ SENSOR_NUM_POUT_E1S_7, sensor_dev_isl28022, I2C_BUS6, SSD_7_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[7] },
	/* SSD8 */
	{ SENSOR_NUM_VOLT_E1S_8, sensor_dev_isl28022, I2C_BUS6, SSD_8_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[8] },
	{ SENSOR_NUM_CURR_E1S_8, sensor_dev_isl28022, I2C_BUS6, SSD_8_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[8] },
	{ SENSOR_NUM_POUT_E1S_8, sensor_dev_isl28022, I2C_BUS6, SSD_8_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[8] },
	/* SSD9 */
	{ SENSOR_NUM_VOLT_E1S_9, sensor_dev_isl28022, I2C_BUS6, SSD_9_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[9] },
	{ SENSOR_NUM_CURR_E1S_9, sensor_dev_isl28022, I2C_BUS6, SSD_9_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[9] },
	{ SENSOR_NUM_POUT_E1S_9, sensor_dev_isl28022, I2C_BUS6, SSD_9_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[9] },
	/* SSD10 */
	{ SENSOR_NUM_VOLT_E1S_10, sensor_dev_isl28022, I2C_BUS6, SSD_10_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[10] },
	{ SENSOR_NUM_CURR_E1S_10, sensor_dev_isl28022, I2C_BUS6, SSD_10_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[10] },
	{ SENSOR_NUM_POUT_E1S_10, sensor_dev_isl28022, I2C_BUS6, SSD_10_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[10] },
	/* SSD11 */
	{ SENSOR_NUM_VOLT_E1S_11, sensor_dev_isl28022, I2C_BUS6, SSD_11_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[11] },
	{ SENSOR_NUM_CURR_E1S_11, sensor_dev_isl28022, I2C_BUS6, SSD_11_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[11] },
	{ SENSOR_NUM_POUT_E1S_11, sensor_dev_isl28022, I2C_BUS6, SSD_11_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[11] },
	/* SSD12 */
	{ SENSOR_NUM_VOLT_E1S_12, sensor_dev_isl28022, I2C_BUS6, SSD_12_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[11] },
	{ SENSOR_NUM_CURR_E1S_12, sensor_dev_isl28022, I2C_BUS6, SSD_12_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[12] },
	{ SENSOR_NUM_POUT_E1S_12, sensor_dev_isl28022, I2C_BUS6, SSD_12_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[12] },
	/* SSD13 */
	{ SENSOR_NUM_VOLT_E1S_13, sensor_dev_isl28022, I2C_BUS6, SSD_13_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[13] },
	{ SENSOR_NUM_CURR_E1S_13, sensor_dev_isl28022, I2C_BUS6, SSD_13_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[13] },
	{ SENSOR_NUM_POUT_E1S_13, sensor_dev_isl28022, I2C_BUS6, SSD_13_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[13] },
	/* SSD14 */
	{ SENSOR_NUM_VOLT_E1S_14, sensor_dev_isl28022, I2C_BUS6, SSD_14_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[14] },
	{ SENSOR_NUM_CURR_E1S_14, sensor_dev_isl28022, I2C_BUS6, SSD_14_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[14] },
	{ SENSOR_NUM_POUT_E1S_14, sensor_dev_isl28022, I2C_BUS6, SSD_14_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[14] },
	/* SSD15 */
	{ SENSOR_NUM_VOLT_E1S_15, sensor_dev_isl28022, I2C_BUS6, SSD_15_POWER_MONITOR_ADDR,
	  ISL28022_BUS_VOLTAGE_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[15] },
	{ SENSOR_NUM_CURR_E1S_15, sensor_dev_isl28022, I2C_BUS6, SSD_15_POWER_MONITOR_ADDR,
	  ISL28022_CURRENT_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[15] },
	{ SENSOR_NUM_POUT_E1S_15, sensor_dev_isl28022, I2C_BUS6, SSD_15_POWER_MONITOR_ADDR,
	  ISL28022_POWER_REG, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &isl28022_ssd_sensor_init_args[15] },
};

sensor_cfg ina230_power_monitor_sensor_config_table[] = {
	/* number,                  type,       port,      address,      offset,
	   access check arg0, arg1, cache, cache_status, mux_ADDRess, mux_offset,
	   pre_sensor_read_fn, pre_sensor_read_args, post_sensor_read_fn, post_sensor_read_fn  */
	/* NIC 0 */
	{ SENSOR_NUM_VOLT_NIC_0, sensor_dev_ina230, I2C_BUS6, NIC_0_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[0] },
	{ SENSOR_NUM_IOUT_NIC_0, sensor_dev_ina230, I2C_BUS6, NIC_0_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[0] },
	{ SENSOR_NUM_POUT_NIC_0, sensor_dev_ina230, I2C_BUS6, NIC_0_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[0] },
	/* NIC 1 */
	{ SENSOR_NUM_VOLT_NIC_1, sensor_dev_ina230, I2C_BUS6, NIC_1_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[1] },
	{ SENSOR_NUM_IOUT_NIC_1, sensor_dev_ina230, I2C_BUS6, NIC_1_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[1] },
	{ SENSOR_NUM_POUT_NIC_1, sensor_dev_ina230, I2C_BUS6, NIC_1_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[1] },
	/* NIC 2 */
	{ SENSOR_NUM_VOLT_NIC_2, sensor_dev_ina230, I2C_BUS6, NIC_2_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[2] },
	{ SENSOR_NUM_IOUT_NIC_2, sensor_dev_ina230, I2C_BUS6, NIC_2_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[2] },
	{ SENSOR_NUM_POUT_NIC_2, sensor_dev_ina230, I2C_BUS6, NIC_2_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[2] },
	/* NIC 3 */
	{ SENSOR_NUM_VOLT_NIC_3, sensor_dev_ina230, I2C_BUS6, NIC_3_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[3] },
	{ SENSOR_NUM_IOUT_NIC_3, sensor_dev_ina230, I2C_BUS6, NIC_3_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[3] },
	{ SENSOR_NUM_POUT_NIC_3, sensor_dev_ina230, I2C_BUS6, NIC_3_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[3] },
	/* NIC 4 */
	{ SENSOR_NUM_VOLT_NIC_4, sensor_dev_ina230, I2C_BUS6, NIC_4_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[4] },
	{ SENSOR_NUM_IOUT_NIC_4, sensor_dev_ina230, I2C_BUS6, NIC_4_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[4] },
	{ SENSOR_NUM_POUT_NIC_4, sensor_dev_ina230, I2C_BUS6, NIC_4_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[4] },
	/* NIC 5 */
	{ SENSOR_NUM_VOLT_NIC_5, sensor_dev_ina230, I2C_BUS6, NIC_5_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[5] },
	{ SENSOR_NUM_IOUT_NIC_5, sensor_dev_ina230, I2C_BUS6, NIC_5_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[5] },
	{ SENSOR_NUM_POUT_NIC_5, sensor_dev_ina230, I2C_BUS6, NIC_5_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[5] },
	/* NIC 6 */
	{ SENSOR_NUM_VOLT_NIC_6, sensor_dev_ina230, I2C_BUS6, NIC_6_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[6] },
	{ SENSOR_NUM_IOUT_NIC_6, sensor_dev_ina230, I2C_BUS6, NIC_6_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[6] },
	{ SENSOR_NUM_POUT_NIC_6, sensor_dev_ina230, I2C_BUS6, NIC_6_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[6] },
	/* NIC 7 */
	{ SENSOR_NUM_VOLT_NIC_7, sensor_dev_ina230, I2C_BUS6, NIC_7_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[7] },
	{ SENSOR_NUM_IOUT_NIC_7, sensor_dev_ina230, I2C_BUS6, NIC_7_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[7] },
	{ SENSOR_NUM_POUT_NIC_7, sensor_dev_ina230, I2C_BUS6, NIC_7_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, nic_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[0],
	  post_i2c_bus_read, NULL, &ina230_nic_sensor_init_args[7] },
	/* PEX 0 P1V25 */
	{ SENSOR_NUM_P1V25_VOLT_PEX_0, sensor_dev_ina230, I2C_BUS6, PEX_0_P1V25_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &ina230_pex_p1v25_sensor_init_args[0] },
	{ SENSOR_NUM_P1V25_IOUT_PEX_0, sensor_dev_ina230, I2C_BUS6, PEX_0_P1V25_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &ina230_pex_p1v25_sensor_init_args[0] },
	{ SENSOR_NUM_P1V25_POUT_PEX_0, sensor_dev_ina230, I2C_BUS6, PEX_0_P1V25_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &ina230_pex_p1v25_sensor_init_args[0] },
	/* PEX 1 P1V25 */
	{ SENSOR_NUM_P1V25_VOLT_PEX_1, sensor_dev_ina230, I2C_BUS6, PEX_1_P1V25_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &ina230_pex_p1v25_sensor_init_args[1] },
	{ SENSOR_NUM_P1V25_IOUT_PEX_1, sensor_dev_ina230, I2C_BUS6, PEX_1_P1V25_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &ina230_pex_p1v25_sensor_init_args[1] },
	{ SENSOR_NUM_P1V25_POUT_PEX_1, sensor_dev_ina230, I2C_BUS6, PEX_1_P1V25_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &ina230_pex_p1v25_sensor_init_args[1] },
	/* PEX 2 P1V25 */
	{ SENSOR_NUM_P1V25_VOLT_PEX_2, sensor_dev_ina230, I2C_BUS6, PEX_2_P1V25_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &ina230_pex_p1v25_sensor_init_args[2] },
	{ SENSOR_NUM_P1V25_IOUT_PEX_2, sensor_dev_ina230, I2C_BUS6, PEX_2_P1V25_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &ina230_pex_p1v25_sensor_init_args[2] },
	{ SENSOR_NUM_P1V25_POUT_PEX_2, sensor_dev_ina230, I2C_BUS6, PEX_2_P1V25_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &ina230_pex_p1v25_sensor_init_args[2] },
	/* PEX 3 P1V25 */
	{ SENSOR_NUM_P1V25_VOLT_PEX_3, sensor_dev_ina230, I2C_BUS6, PEX_3_P1V25_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &ina230_pex_p1v25_sensor_init_args[3] },
	{ SENSOR_NUM_P1V25_IOUT_PEX_3, sensor_dev_ina230, I2C_BUS6, PEX_3_P1V25_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &ina230_pex_p1v25_sensor_init_args[3] },
	{ SENSOR_NUM_P1V25_POUT_PEX_3, sensor_dev_ina230, I2C_BUS6, PEX_3_P1V25_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &ina230_pex_p1v25_sensor_init_args[3] },
	/* PEX P1V8 */
	{ SENSOR_NUM_P1V8_VOLT_PEX, sensor_dev_ina230, I2C_BUS6, PEX_P1V8_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &ina230_pex_p1v8_sensor_init_args[0] },
	{ SENSOR_NUM_P1V8_IOUT_PEX, sensor_dev_ina230, I2C_BUS6, PEX_P1V8_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &ina230_pex_p1v8_sensor_init_args[0] },
	{ SENSOR_NUM_P1V8_POUT_PEX, sensor_dev_ina230, I2C_BUS6, PEX_P1V8_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, stby_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[1],
	  post_i2c_bus_read, NULL, &ina230_pex_p1v8_sensor_init_args[0] },
	/* SSD 0 */
	{ SENSOR_NUM_VOLT_E1S_0, sensor_dev_ina230, I2C_BUS6, SSD_0_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[0] },
	{ SENSOR_NUM_CURR_E1S_0, sensor_dev_ina230, I2C_BUS6, SSD_0_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[0] },
	{ SENSOR_NUM_POUT_E1S_0, sensor_dev_ina230, I2C_BUS6, SSD_0_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[0] },
	/* SSD 1 */
	{ SENSOR_NUM_VOLT_E1S_1, sensor_dev_ina230, I2C_BUS6, SSD_1_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[1] },
	{ SENSOR_NUM_CURR_E1S_1, sensor_dev_ina230, I2C_BUS6, SSD_1_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[1] },
	{ SENSOR_NUM_POUT_E1S_1, sensor_dev_ina230, I2C_BUS6, SSD_1_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[1] },
	/* SSD 2 */
	{ SENSOR_NUM_VOLT_E1S_2, sensor_dev_ina230, I2C_BUS6, SSD_2_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[2] },
	{ SENSOR_NUM_CURR_E1S_2, sensor_dev_ina230, I2C_BUS6, SSD_2_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[2] },
	{ SENSOR_NUM_POUT_E1S_2, sensor_dev_ina230, I2C_BUS6, SSD_2_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[2] },
	/* SSD 3 */
	{ SENSOR_NUM_VOLT_E1S_3, sensor_dev_ina230, I2C_BUS6, SSD_3_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[3] },
	{ SENSOR_NUM_CURR_E1S_3, sensor_dev_ina230, I2C_BUS6, SSD_3_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[3] },
	{ SENSOR_NUM_POUT_E1S_3, sensor_dev_ina230, I2C_BUS6, SSD_3_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[3] },
	/* SSD4 */
	{ SENSOR_NUM_VOLT_E1S_4, sensor_dev_ina230, I2C_BUS6, SSD_4_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[4] },
	{ SENSOR_NUM_CURR_E1S_4, sensor_dev_ina230, I2C_BUS6, SSD_4_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[4] },
	{ SENSOR_NUM_POUT_E1S_4, sensor_dev_ina230, I2C_BUS6, SSD_4_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[4] },
	/* SSD5 */
	{ SENSOR_NUM_VOLT_E1S_5, sensor_dev_ina230, I2C_BUS6, SSD_5_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[5] },
	{ SENSOR_NUM_CURR_E1S_5, sensor_dev_ina230, I2C_BUS6, SSD_5_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[5] },
	{ SENSOR_NUM_POUT_E1S_5, sensor_dev_ina230, I2C_BUS6, SSD_5_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[5] },
	/* SSD6 */
	{ SENSOR_NUM_VOLT_E1S_6, sensor_dev_ina230, I2C_BUS6, SSD_6_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[6] },
	{ SENSOR_NUM_CURR_E1S_6, sensor_dev_ina230, I2C_BUS6, SSD_6_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[6] },
	{ SENSOR_NUM_POUT_E1S_6, sensor_dev_ina230, I2C_BUS6, SSD_6_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[6] },
	/* SSD7 */
	{ SENSOR_NUM_VOLT_E1S_7, sensor_dev_ina230, I2C_BUS6, SSD_7_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[7] },
	{ SENSOR_NUM_CURR_E1S_7, sensor_dev_ina230, I2C_BUS6, SSD_7_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[7] },
	{ SENSOR_NUM_POUT_E1S_7, sensor_dev_ina230, I2C_BUS6, SSD_7_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[3],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[7] },
	/* SSD8 */
	{ SENSOR_NUM_VOLT_E1S_8, sensor_dev_ina230, I2C_BUS6, SSD_8_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[8] },
	{ SENSOR_NUM_CURR_E1S_8, sensor_dev_ina230, I2C_BUS6, SSD_8_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[8] },
	{ SENSOR_NUM_POUT_E1S_8, sensor_dev_ina230, I2C_BUS6, SSD_8_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[8] },
	/* SSD9 */
	{ SENSOR_NUM_VOLT_E1S_9, sensor_dev_ina230, I2C_BUS6, SSD_9_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[9] },
	{ SENSOR_NUM_CURR_E1S_9, sensor_dev_ina230, I2C_BUS6, SSD_9_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[9] },
	{ SENSOR_NUM_POUT_E1S_9, sensor_dev_ina230, I2C_BUS6, SSD_9_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[9] },
	/* SSD10 */
	{ SENSOR_NUM_VOLT_E1S_10, sensor_dev_ina230, I2C_BUS6, SSD_10_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[10] },
	{ SENSOR_NUM_CURR_E1S_10, sensor_dev_ina230, I2C_BUS6, SSD_10_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[10] },
	{ SENSOR_NUM_POUT_E1S_10, sensor_dev_ina230, I2C_BUS6, SSD_10_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[10] },
	/* SSD11 */
	{ SENSOR_NUM_VOLT_E1S_11, sensor_dev_ina230, I2C_BUS6, SSD_11_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[11] },
	{ SENSOR_NUM_CURR_E1S_11, sensor_dev_ina230, I2C_BUS6, SSD_11_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[11] },
	{ SENSOR_NUM_POUT_E1S_11, sensor_dev_ina230, I2C_BUS6, SSD_11_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[11] },
	/* SSD12 */
	{ SENSOR_NUM_VOLT_E1S_12, sensor_dev_ina230, I2C_BUS6, SSD_12_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[11] },
	{ SENSOR_NUM_CURR_E1S_12, sensor_dev_ina230, I2C_BUS6, SSD_12_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[12] },
	{ SENSOR_NUM_POUT_E1S_12, sensor_dev_ina230, I2C_BUS6, SSD_12_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[12] },
	/* SSD13 */
	{ SENSOR_NUM_VOLT_E1S_13, sensor_dev_ina230, I2C_BUS6, SSD_13_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[13] },
	{ SENSOR_NUM_CURR_E1S_13, sensor_dev_ina230, I2C_BUS6, SSD_13_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[13] },
	{ SENSOR_NUM_POUT_E1S_13, sensor_dev_ina230, I2C_BUS6, SSD_13_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[13] },
	/* SSD14 */
	{ SENSOR_NUM_VOLT_E1S_14, sensor_dev_ina230, I2C_BUS6, SSD_14_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[14] },
	{ SENSOR_NUM_CURR_E1S_14, sensor_dev_ina230, I2C_BUS6, SSD_14_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[14] },
	{ SENSOR_NUM_POUT_E1S_14, sensor_dev_ina230, I2C_BUS6, SSD_14_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[14] },
	/* SSD15 */
	{ SENSOR_NUM_VOLT_E1S_15, sensor_dev_ina230, I2C_BUS6, SSD_15_POWER_MONITOR_ADDR,
	  INA230_BUS_VOL_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[15] },
	{ SENSOR_NUM_CURR_E1S_15, sensor_dev_ina230, I2C_BUS6, SSD_15_POWER_MONITOR_ADDR,
	  INA230_CUR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[15] },
	{ SENSOR_NUM_POUT_E1S_15, sensor_dev_ina230, I2C_BUS6, SSD_15_POWER_MONITOR_ADDR,
	  INA230_PWR_OFFSET, e1s_access, 0, 0, SAMPLE_COUNT_DEFAULT, POLL_TIME_DEFAULT,
	  ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, pre_i2c_bus_read, &mux_conf_addr_0xe0[4],
	  post_i2c_bus_read, NULL, &ina230_ssd_sensor_init_args[15] },
};
const int SENSOR_CONFIG_SIZE = ARRAY_SIZE(plat_sensor_config);

void load_sensor_config(void)
{
	memcpy(sensor_config, plat_sensor_config, sizeof(plat_sensor_config));
	sensor_config_count = ARRAY_SIZE(plat_sensor_config);

	pal_extend_sensor_config();
}
/**
 * EVT2 switch board has two configurations that will cause the need to load different
 * sensor table.
 * 
 * 1st  : ISL69259(VR) + MPS5990(HSC) + ISL28022(Power monitor IC)
 * 2nd  : XDPE12284(VR) + LTC4282(HSC) + INA230(Power monitor IC)
 * 
 * Since the BOARD_ID pins were originally used to identify which configuration 
 * has no function in this stage, so the current configuration is identified by 
 * IC_DEVICE_ID which gets from the VR chip.
 * 
 * The way to identify different configurations by BOARD_ID will be added back in 
 * the next stage.
 */
void pal_extend_sensor_config()
{
	uint8_t stage = get_stage_by_rev_id();

	if (stage == EVT) {
		LOG_INF("The board is in EVT stage");
		memcpy(&sensor_config[sensor_config_count], evt_pex_sensor_config_table,
		       ARRAY_SIZE(evt_pex_sensor_config_table) * sizeof(sensor_cfg));
		sensor_config_count += ARRAY_SIZE(evt_pex_sensor_config_table);

		switch (check_vr_type()) {
		case VR_RNS_ISL69259:
			LOG_INF("VR type: RNS_ISL69259");

			memcpy(&sensor_config[sensor_config_count], mp5990_hsc_sensor_config_table,
			       ARRAY_SIZE(mp5990_hsc_sensor_config_table) * sizeof(sensor_cfg));
			sensor_config_count += ARRAY_SIZE(mp5990_hsc_sensor_config_table);

			memcpy(&sensor_config[sensor_config_count], isl69259_vr_sensor_config_table,
			       ARRAY_SIZE(isl69259_vr_sensor_config_table) * sizeof(sensor_cfg));
			sensor_config_count += ARRAY_SIZE(isl69259_vr_sensor_config_table);

			memcpy(&sensor_config[sensor_config_count],
			       isl28022_power_monitor_sensor_config_table,
			       ARRAY_SIZE(isl28022_power_monitor_sensor_config_table) *
				       sizeof(sensor_cfg));
			sensor_config_count +=
				ARRAY_SIZE(isl28022_power_monitor_sensor_config_table);
			break;
		case VR_INF_XDPE12284:
			LOG_INF("VR type: INF_XDPE12284");

			memcpy(&sensor_config[sensor_config_count], ltc4282_hsc_sensor_config_table,
			       ARRAY_SIZE(ltc4282_hsc_sensor_config_table) * sizeof(sensor_cfg));
			sensor_config_count += ARRAY_SIZE(ltc4282_hsc_sensor_config_table);

			memcpy(&sensor_config[sensor_config_count],
			       xdpe12284_vr_sensor_config_table,
			       ARRAY_SIZE(xdpe12284_vr_sensor_config_table) * sizeof(sensor_cfg));
			sensor_config_count += ARRAY_SIZE(xdpe12284_vr_sensor_config_table);

			memcpy(&sensor_config[sensor_config_count],
			       ina230_power_monitor_sensor_config_table,
			       ARRAY_SIZE(ina230_power_monitor_sensor_config_table) *
				       sizeof(sensor_cfg));
			sensor_config_count += ARRAY_SIZE(ina230_power_monitor_sensor_config_table);
			break;
		default:
			LOG_ERR("Unsupported VR type");
			break;
		}
	} else if (stage == DVT) {
		LOG_INF("The board is in DVT stage");
		memcpy(&sensor_config[sensor_config_count], dvt_pex_sensor_config_table,
		       ARRAY_SIZE(dvt_pex_sensor_config_table) * sizeof(sensor_cfg));
		sensor_config_count += ARRAY_SIZE(dvt_pex_sensor_config_table);
		load_hsc_sensor_table();
		load_vr_sensor_table();
		load_power_ic_sensor_table();
		change_p1v8_sensor_i2c_addr();
	} else {
		LOG_ERR("Unsupport stage, (%d)", stage);
	}
}

uint8_t pal_get_extend_sensor_config()
{
	uint8_t extend_sensor_config_size = 0;
	uint8_t stage = get_stage_by_rev_id();

	if (stage == EVT) {
		extend_sensor_config_size += ARRAY_SIZE(evt_pex_sensor_config_table);
		switch (check_vr_type()) {
		case VR_RNS_ISL69259:
			extend_sensor_config_size += ARRAY_SIZE(mp5990_hsc_sensor_config_table);
			extend_sensor_config_size += ARRAY_SIZE(isl69259_vr_sensor_config_table);
			extend_sensor_config_size +=
				ARRAY_SIZE(isl28022_power_monitor_sensor_config_table);
			break;
		case VR_INF_XDPE12284:
			extend_sensor_config_size += ARRAY_SIZE(ltc4282_hsc_sensor_config_table);
			extend_sensor_config_size += ARRAY_SIZE(xdpe12284_vr_sensor_config_table);
			extend_sensor_config_size +=
				ARRAY_SIZE(ina230_power_monitor_sensor_config_table);
			break;
		default:
			LOG_ERR("Unsupported VR type");
			break;
		}
	} else if (stage == DVT) {
		extend_sensor_config_size += ARRAY_SIZE(dvt_pex_sensor_config_table);
		extend_sensor_config_size += ARRAY_SIZE(mp5990_hsc_sensor_config_table);
		extend_sensor_config_size += ARRAY_SIZE(isl69259_vr_sensor_config_table);
		extend_sensor_config_size += ARRAY_SIZE(isl28022_power_monitor_sensor_config_table);
	} else {
		LOG_ERR("Unsupport stage, (%d)", stage);
	}
	return extend_sensor_config_size;
}

void change_p1v8_sensor_i2c_addr()
{
	LOG_INF("Change p1v8_pex sensor to the i2c address of DVT stage");
	for (int i = 0; i < sensor_config_count; i++) {
		switch (sensor_config[i].num) {
		case SENSOR_NUM_P1V8_VOLT_PEX:
		case SENSOR_NUM_P1V8_IOUT_PEX:
		case SENSOR_NUM_P1V8_POUT_PEX:
			sensor_config[i].target_addr = DVT_PEX_P1V8_POWER_MONITOR_ADDR;
			break;
		default:
			break;
		}
	}
}
/**
 * Follow the schematic diagram, the HSC device can be distinguished by ADC11 (HSC_TYPE_ADC_R).
 * 0.0V(+/- 15%), the hotswap model is MP5990.
 * 1.0V(+/- 15%), the hotswap model is LTC4282.
 * 1.5V(+/- 15%), the hotswap model is LTC4286.
 */
static void load_hsc_sensor_table()
{
	float voltage_hsc_type_adc = 0;

	if (!get_adc_voltage(HSC_TYPE_ADC_CHANNEL, &voltage_hsc_type_adc)) {
		LOG_ERR("Failed to get hsc type by ADC");
		return;
	}

	if (voltage_hsc_type_adc < 0.15) {
		memcpy(&sensor_config[sensor_config_count], mp5990_hsc_sensor_config_table,
		       ARRAY_SIZE(mp5990_hsc_sensor_config_table) * sizeof(sensor_cfg));
		sensor_config_count += ARRAY_SIZE(mp5990_hsc_sensor_config_table);
	} else if ((voltage_hsc_type_adc > 1.0 - (1.0 * 0.15)) &&
		   (voltage_hsc_type_adc < 1.0 + (1.0 * 0.15))) {
		memcpy(&sensor_config[sensor_config_count], ltc4282_hsc_sensor_config_table,
		       ARRAY_SIZE(ltc4282_hsc_sensor_config_table) * sizeof(sensor_cfg));
		sensor_config_count += ARRAY_SIZE(ltc4282_hsc_sensor_config_table);
	} else {
		LOG_ERR("Unknown hotswap model type, HSC_TYPE_ADC voltage: %d.%d V",
			(uint16_t)voltage_hsc_type_adc,
			(uint16_t)((voltage_hsc_type_adc - (uint16_t)voltage_hsc_type_adc) * 100));
	}

	return;
}
/**
 * Follow the schematic diagram, the VR chip can be distinguished by ADC12 (VR_TYPE_ADC_R).
 * 0.0V(+/- 15%), the VR chip is ISL69259.
 * 0.5V(+/- 15%), the VR chip is XDPE12284.
 */
static void load_vr_sensor_table()
{
	float voltage_vr_type_adc = 0;

	if (!get_adc_voltage(VR_TYPE_ADC_CHANNEL, &voltage_vr_type_adc)) {
		LOG_ERR("Failed to get VR type by ADC");
		return;
	}

	if (voltage_vr_type_adc < 0.15) {
		memcpy(&sensor_config[sensor_config_count], isl69259_vr_sensor_config_table,
		       ARRAY_SIZE(isl69259_vr_sensor_config_table) * sizeof(sensor_cfg));
		sensor_config_count += ARRAY_SIZE(isl69259_vr_sensor_config_table);
	} else if ((voltage_vr_type_adc > 0.5 - (0.5 * 0.15)) &&
		   (voltage_vr_type_adc < 0.5 + (0.5 * 0.15))) {
		memcpy(&sensor_config[sensor_config_count], xdpe12284_vr_sensor_config_table,
		       ARRAY_SIZE(xdpe12284_vr_sensor_config_table) * sizeof(sensor_cfg));
		sensor_config_count += ARRAY_SIZE(xdpe12284_vr_sensor_config_table);
	} else {
		LOG_ERR("Unknown VR type, vr_type_adc voltage: %d.%d V",
			(uint16_t)voltage_vr_type_adc,
			(uint16_t)((voltage_vr_type_adc - (uint16_t)voltage_vr_type_adc) * 100));
	}

	return;
}

/**
 * Follow the schematic diagram, the power monitor ic can be distinguished by ADC13 (ADC_TYPE_ADC_R).
 * 0.0V(+/- 15%), the power monitor ic is ISL28022.
 * 0.5V(+/- 15%), the power monitor ic is INA230.
 */

static void load_power_ic_sensor_table()
{
	float voltage_power_ic_type_adc = 0;

	if (!get_adc_voltage(POWER_IC_TYPE_ADC_CHANNEL, &voltage_power_ic_type_adc)) {
		LOG_ERR("Failed to get power monitor IC type by ADC");
		return;
	}

	if (voltage_power_ic_type_adc < 0.15) {
		memcpy(&sensor_config[sensor_config_count],
		       isl28022_power_monitor_sensor_config_table,
		       ARRAY_SIZE(isl28022_power_monitor_sensor_config_table) * sizeof(sensor_cfg));
		sensor_config_count += ARRAY_SIZE(isl28022_power_monitor_sensor_config_table);
	} else if ((voltage_power_ic_type_adc > 0.5 - (0.5 * 0.15)) &&
		   (voltage_power_ic_type_adc < 0.5 + (0.5 * 0.15))) {
		memcpy(&sensor_config[sensor_config_count],
		       ina230_power_monitor_sensor_config_table,
		       ARRAY_SIZE(ina230_power_monitor_sensor_config_table) * sizeof(sensor_cfg));
		sensor_config_count += ARRAY_SIZE(ina230_power_monitor_sensor_config_table);
	} else {
		LOG_ERR("Unknown power monitor IC type, power_ic_type_adc voltage: %d.%dV",
			(uint16_t)voltage_power_ic_type_adc,
			(uint16_t)(
				(voltage_power_ic_type_adc - (uint16_t)voltage_power_ic_type_adc) *
				100));
	}

	return;
}

static int check_vr_type(void)
{
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	/* Change i2c mux to the channel of VR */
	msg.bus = I2C_BUS6;
	msg.target_addr = (0xe0 >> 1);
	msg.tx_len = 1;
	msg.data[0] = (1 << 6);

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Change i2c mux channel on bus 6 failed");
		return -1;
	}

	msg.target_addr = PEX_0_1_P0V8_VR_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 7;
	msg.data[0] = PMBUS_IC_DEVICE_ID;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("Failed to read VR IC_DEVICE_ID");
		return -1;
	}

	if (!memcmp(msg.data, ISL69259_DEVICE_ID, sizeof(ISL69259_DEVICE_ID))) {
		return VR_RNS_ISL69259;
	} else if (!memcmp(msg.data, XDPE12284C_DEVICE_ID, sizeof(XDPE12284C_DEVICE_ID))) {
		return VR_INF_XDPE12284;
	}

	LOG_ERR("Unsupported VR type");
	return -1;
}

bool e1s_access(uint8_t sensor_num)
{
	uint8_t e1s_prsnt_pin[4][4] = {
		{ PRSNT_SSD0_R_N, PRSNT_SSD1_R_N, PRSNT_SSD2_R_N, PRSNT_SSD3_R_N },
		{ PRSNT_SSD4_R_N, PRSNT_SSD5_R_N, PRSNT_SSD6_R_N, PRSNT_SSD7_R_N },
		{ PRSNT_SSD8_R_N, PRSNT_SSD9_R_N, PRSNT_SSD10_R_N, PRSNT_SSD11_R_N },
		{ PRSNT_SSD12_R_N, PRSNT_SSD13_R_N, PRSNT_SSD14_R_N, PRSNT_SSD15_R_N },
	};
	uint8_t group = (sensor_num >> 4) % 8;
	uint8_t index = ((sensor_num & BIT_MASK(4)) / 4);

	return (!gpio_get(e1s_prsnt_pin[group][index]) && is_mb_dc_on());
}

bool nic_access(uint8_t sensor_num)
{
	uint8_t nic_prsnt_pin[8] = {
		PRSNT_NIC0_R_N, PRSNT_NIC1_R_N, PRSNT_NIC2_R_N, PRSNT_NIC3_R_N,
		PRSNT_NIC4_R_N, PRSNT_NIC5_R_N, PRSNT_NIC6_R_N, PRSNT_NIC7_R_N,
	};

	uint8_t pin_index = ((sensor_num >> 4) * 3) + ((sensor_num & BIT_MASK(4)) / 5);

	return !gpio_get(nic_prsnt_pin[pin_index]) ? true : false;
}