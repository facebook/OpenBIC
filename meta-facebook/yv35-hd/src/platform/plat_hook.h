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

#ifndef PLAT_HOOK_H
#define PLAT_HOOK_H

typedef struct _raa229621_pre_proc_arg {
	/* vr page to set */
	uint8_t vr_page;
} vr_pre_proc_arg;

typedef struct _report_dimm_power_data_in {
	uint32_t dimm_addr : 8;
	uint32_t update_rate : 9;
	uint32_t dimm_power_mw : 15;
} report_dimm_power_data_in;

typedef struct _report_dimm_temp_data_in {
	uint32_t dimm_addr : 8;
	uint32_t update_rate : 9;
	uint32_t reserved : 4;
	uint32_t dimm_temp : 11;
} report_dimm_temp_data_in;

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
extern adc_asd_init_arg ast_adc_init_args[];
extern adm1278_init_arg adm1278_init_args[];
extern apml_mailbox_init_arg apml_mailbox_init_args[];
extern ltc4282_init_arg ltc4282_init_args[];
extern ddr5_init_temp_arg ddr5_init_temp_args[];
extern ddr5_init_power_arg ddr5_init_power_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
extern struct tca9548 mux_conf_addr_0xe2[];
extern vr_pre_proc_arg vr_pre_read_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
bool pre_nvme_read(uint8_t sensor_num, void *args);
bool pre_vr_read(uint8_t sensor_num, void *args);
bool pre_vol_bat3v_read(uint8_t sensor_num, void *args);
bool post_vol_bat3v_read(uint8_t sensor_num, void *args, int *reading);
bool post_adm1278_cur_read(uint8_t sensor_num, void *args, int *reading);
bool post_adm1278_pwr_read(uint8_t sensor_num, void *args, int *reading);
bool post_ltc4282_cur_read(uint8_t sensor_num, void *args, int *reading);
bool post_ltc4282_pwr_read(uint8_t sensor_num, void *args, int *reading);
bool post_ddr5_pwr_read(uint8_t sensor_num, void *args, int *reading);
bool post_ddr5_temp_read(uint8_t sensor_num, void *args, int *reading);
bool post_amd_tsi_read(uint8_t sensor_num, void *args, int *reading);

#endif
