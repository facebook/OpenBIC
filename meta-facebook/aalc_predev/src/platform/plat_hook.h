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
#include "i2c-mux-pca954x.h"
#include "ina238.h"
#include "nct7363.h"
#include "nct214.h"
#include "ads112c.h"
#include "tmp461.h"
#include "hdc1080.h"
#include "ast_tach.h"
#include "xdp710.h"

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
extern adm1272_init_arg adm1272_init_args[];
extern nct7363_init_arg nct7363_init_args[];
extern ads112c_init_arg ads112c_init_args[];
extern adc_asd_init_arg adc_asd_init_args[];
extern ina238_init_arg ina238_init_args[];
extern nct214_init_arg nct214_init_args[];
extern xdp710_init_arg xdp710_init_args[];

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
extern mux_config bus_1_PCA9546A_configs[];
extern mux_config bus_2_PCA9546A_configs[];
extern mux_config bus_6_PCA9546A_configs[];
extern mux_config bus_7_PCA9546A_configs[];
extern mux_config bus_8_PCA9546A_configs[];
extern mux_config bus_9_PCA9546A_configs[];

bool post_adm1272_read(sensor_cfg *cfg, void *args, int *reading);
bool post_ads112c_read(sensor_cfg *cfg, void *args, int *reading);
bool pre_PCA9546A_read(sensor_cfg *cfg, void *args);
bool post_PCA9546A_read(sensor_cfg *cfg, void *args, int *reading);
#endif
