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

typedef struct _vr_pre_read_arg {
	/* vr page to set */
	uint8_t vr_page;
} vr_pre_read_arg;

extern vr_pre_read_arg vr_pre_read_args[];
extern adc_asd_init_arg ast_adc_init_args[];
extern ina233_init_arg ina233_init_args[];
bool pre_vr_read(sensor_cfg *cfg, void *args);

#endif
