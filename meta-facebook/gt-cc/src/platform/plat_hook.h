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

#include "sensor.h"

typedef struct _vr_pre_proc_arg {
	struct tca9548 *mux_info_p;
	/* vr page to set */
	uint8_t vr_page;
} vr_pre_proc_arg;

typedef struct _pex89000_pre_proc_arg {
	struct tca9548 *mux_info_p;
} pex89000_pre_proc_arg;

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
extern isl28022_init_arg isl28022_nic_sensor_init_args[];
extern ina230_init_arg ina230_nic_sensor_init_args[];

extern mp5990_init_arg mp5990_hsc_init_args[];
extern adc_asd_init_arg ast_adc_init_args[];
extern isl28022_init_arg isl28022_pex_p1v25_sensor_init_args[];
extern ina230_init_arg ina230_pex_p1v25_sensor_init_args[];
extern isl28022_init_arg isl28022_pex_p1v8_sensor_init_args[];
extern ina230_init_arg ina230_pex_p1v8_sensor_init_args[];
extern isl28022_init_arg isl28022_ssd_sensor_init_args[];
extern ina230_init_arg ina230_ssd_sensor_init_args[];
extern pex89000_init_arg pex_sensor_init_args[];
extern ltc4282_init_arg ltc4282_hsc_init_args[];
extern ltc4286_init_arg ltc4286_hsc_init_args[];
extern nct7718w_init_arg nct7718w_init_args[];
extern cx7_init_arg cx7_init_args[];
/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
extern struct tca9548 mux_conf_addr_0xe0[];
extern struct tca9548 mux_conf_addr_0xe2[];
extern vr_pre_proc_arg vr_pre_read_args[];
extern pex89000_pre_proc_arg pex89000_pre_read_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
bool pre_vr_read(sensor_cfg *cfg, void *args);
bool pre_pex89000_read(sensor_cfg *cfg, void *args);
bool pre_i2c_bus_read(sensor_cfg *cfg, void *args);
bool post_i2c_bus_read(sensor_cfg *cfg, void *args, int *reading);
bool post_mp5990_read(sensor_cfg *cfg, void *args, int *reading);
bool post_ltc4282_read(sensor_cfg *cfg, void *args, int *reading);
bool post_ltc4286_read(sensor_cfg *cfg, void *args, int *reading);

struct k_mutex *find_bus_mutex(sensor_cfg *cfg);
void ssd_drive_reinit(void);
void nic_drive_reinit_for_pollara(void);
void nic_optics_drive_reinit_for_pollara(void);
bool is_mb_dc_on();
void set_cx7_init_arg_to_thor2();

#endif
