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

#include "pmbus.h"
#include "sensor.h"
#include "common_i2c_mux.h"

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
extern adc_asd_init_arg adc_asd_init_args[];
extern adm1272_init_arg adm1272_init_args[];
extern ina233_init_arg ina233_init_args[];
extern pex89000_init_arg pex_sensor_init_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
extern mux_config tca9543_configs[];
extern mux_config pi4msd5v9542_configs[];
extern vr_page_cfg xdpe15284_page[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
bool pre_ina233_read(uint8_t sensor_num, void *args);
bool post_ina233_read(uint8_t sensor_num, void *args, int *reading);
bool pre_pex89000_read(uint8_t sensor_num, void *args);
bool post_pex89000_read(uint8_t sensor_num, void *args, int *reading);
bool pre_xdpe15284_read(uint8_t sensor_num, void *args);
bool post_xdpe15284_read(uint8_t sensor_num, void *args, int *reading);

#endif
