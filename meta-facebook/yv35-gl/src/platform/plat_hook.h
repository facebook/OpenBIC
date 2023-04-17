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

typedef struct _xdpe15284_pre_read_arg {
	/* vr page to set */
	uint8_t vr_page;
} xdpe15284_pre_read_arg;

typedef struct _dimm_pre_proc_arg {
	bool is_present_checked;
} dimm_pre_proc_arg;

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
extern adc_asd_init_arg adc_asd_init_args[];
extern adm1278_init_arg adm1278_init_args[];
extern mp5990_init_arg mp5990_init_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
extern struct tca9548 mux_conf_addr_0xe2[];
extern xdpe15284_pre_read_arg xdpe15284_pre_read_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
bool pre_xdpe15284_read(uint8_t sensor_num, void *args);
bool pre_vol_bat3v_read(uint8_t sensor_num, void *args);
bool post_vol_bat3v_read(uint8_t sensor_num, void *args, int *reading);
bool post_cpu_margin_read(uint8_t sensor_num, void *args, int *reading);
bool pre_nvme_read(uint8_t sensor_num, void *args);
bool pre_intel_peci_dimm_read(uint8_t sensor_num, void *args);

#endif
