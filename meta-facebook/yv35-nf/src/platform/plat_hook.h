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

typedef struct _xdpe12284cpre_proc_arg {
	uint8_t vr_page;
} xdpe12284c_pre_arg;

extern adc_asd_init_arg adc_asd_init_args[];
extern ina233_init_arg ina233_init_args[];
extern xdpe12284c_pre_arg xdpe12284c_pre_read_args[];

bool pre_xdpe12284c_read(uint8_t sensor_num, void *args);

#endif
