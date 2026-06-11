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

#ifndef IRIS_SMBUS_H
#define IRIS_SMBUS_H

//asic reg define
#define ASIC_REG_STATUS_REG 0x00
#define ASIC_MONITOR_HBM_TEMP_REG 0x8F // Max of 8 HBM temperature sensors in history
#define ASIC_MONITOR_TEMP_REG 0x70
#define ASIC_VERSION_REG 0x68
//asic reg len
#define ASIC_STATUS_REG_LEN 8
#define ASIC_VERSION_REG_LEN 10
#define ASIC_MONITOR_TEMP_REG_LEN 10
#define ASIC_MONITOR_HBM_TEMP_REG_LEN 10


typedef struct {
    uint8_t temp[ASIC_MONITOR_TEMP_REG_LEN];
    uint8_t hbm_temp[ASIC_MONITOR_HBM_TEMP_REG_LEN];
} iris_priv_data_t;


#endif
