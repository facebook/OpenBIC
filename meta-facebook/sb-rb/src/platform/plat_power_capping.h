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
#ifndef PLAT_POWER_CAPPING_H
#define PLAT_POWER_CAPPING_H

#define CPLD_LV1_TIME_WINDOW_NUM 8

enum { CAPPING_M_LOOK_UP_TABLE = 0,
       CAPPING_M_CREDIT_BASE,
       CAPPING_M_MAX,
};

enum { CAPPING_SOURCE_VR = 0,
       CAPPING_SOURCE_ADC,
       CAPPING_SOURCE_MAX,
};

enum { CAPPING_VR_IDX_MEDHA0 = 0,
       CAPPING_VR_IDX_MEDHA1,
       CAPPING_VR_IDX_MAX,
};

enum { CAPPING_LV_IDX_LV1 = 0,
       CAPPING_LV_IDX_LV2,
       CAPPING_LV_IDX_LV3,
       CAPPING_LV_IDX_MAX,
};

#endif

void power_capping_syn_vr_oc_warn_limit();
void add_sync_oc_warn_to_work();
bool find_cpld_lv1_time_window_idx_by_value(uint8_t *idx, uint16_t value);
uint16_t get_power_capping_avg_power(uint8_t vr_idx, uint8_t lv);
uint8_t get_power_capping_method();
void set_power_capping_method(uint8_t value);
uint8_t get_power_capping_source();
void set_power_capping_source(uint8_t value);
uint16_t get_power_capping_current_threshold(uint8_t vr_idx);
uint16_t get_power_capping_time_w(uint8_t vr_idx, uint8_t lv);
void set_power_capping_time_w(uint8_t vr_idx, uint8_t lv, uint16_t value);
uint16_t get_power_capping_threshold(uint8_t vr_idx, uint8_t lv);
void set_power_capping_threshold(uint8_t vr_idx, uint8_t lv, uint16_t value);
void plat_power_capping_init();
