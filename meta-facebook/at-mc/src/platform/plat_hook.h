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

#include "common_i2c_mux.h"

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
extern mp5990_init_arg mp5990_init_args[];
extern adc_asd_init_arg adc_asd_init_args[];
extern sq52205_init_arg sq52205_init_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
extern mux_config bus_3_pca9546_configs[];
extern mux_config bus_4_pca9548_configs[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
bool pre_nvme_read(uint8_t sensor_num, void *args);
bool post_nvme_read(uint8_t sensor_num, void *args, int *reading);
bool pre_sq52205_read(uint8_t sensor_num, void *args);
bool post_sq52205_read(uint8_t sensor_num, void *args, int *reading);

#endif
