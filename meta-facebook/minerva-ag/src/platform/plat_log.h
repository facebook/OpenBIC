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

#ifndef PLAT_LOG_H
#define PLAT_LOG_H

#include "plat_pldm_sensor.h"

#define AEGIS_FRU_LOG_SIZE sizeof(plat_err_log_mapping)

#define LOG_ASSERT 1
#define LOG_DEASSERT 0

uint16_t error_log_count(void);
void init_load_eeprom_log(void);

void plat_log_read(uint8_t *log_data, uint8_t cmd_size, uint16_t order);
void error_log_event(uint16_t error_code, bool log_status);
void plat_clear_log();
uint8_t plat_log_get_num(void);

typedef struct __attribute__((packed)) _plat_err_log_mapping {
	uint16_t index;
	uint16_t err_code;
	uint64_t sys_time;
	uint8_t error_data[20];
	uint8_t cpld_dump[96];
} plat_err_log_mapping;

enum LOG_ERROR_TRIGGER_CAUSE {
	CPLD_UNEXPECTED_VAL_TRIGGER_CAUSE = 0b100,
	POWER_ON_SEQUENCE_TRIGGER_CAUSE = 0b001,
	MAX_TRIGGER_CAUSE = 0b1000, //trigger cause maxium 3 bit
};

#endif
