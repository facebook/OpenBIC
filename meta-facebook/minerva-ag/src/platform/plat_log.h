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

#define AEGIS_CPLD_REGISTER_MAX_NUM 96
#define AEGIS_CPLD_REGISTER_1ST_PART_START_OFFSET                                                  \
	0x00 // first part of cpld register offset from 0x00 to 0x47
#define AEGIS_CPLD_REGISTER_1ST_PART_NUM 72
#define AEGIS_CPLD_REGISTER_2ND_PART_START_OFFSET 0x97
#define AEGIS_CPLD_REGISTER_2ND_PART_NUM                                                           \
	(AEGIS_CPLD_REGISTER_MAX_NUM - AEGIS_CPLD_REGISTER_1ST_PART_NUM)
#define AEGIS_FRU_LOG_SIZE sizeof(plat_err_log_mapping)

#define LOG_ASSERT 1
#define LOG_DEASSERT 0

uint16_t error_log_count(void);
void init_load_eeprom_log(void);

void plat_log_read(uint8_t *log_data, uint8_t cmd_size, uint16_t order);
void error_log_event(uint16_t error_code, bool log_status);
void plat_clear_log();
uint8_t plat_log_get_num(void);
void reset_error_log_event(uint8_t err_type);
bool vr_fault_get_error_data(uint8_t sensor_id, uint8_t device_id, uint8_t *data);

struct rtc_datetime {
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t _padding;
};

typedef struct __attribute__((packed)) _plat_err_log_mapping {
	uint16_t index;
	uint16_t err_code;
	struct rtc_datetime sys_datetime;
	uint8_t error_data[20];
	uint8_t cpld_dump[AEGIS_CPLD_REGISTER_MAX_NUM];
} plat_err_log_mapping;

enum LOG_ERROR_TRIGGER_CAUSE {
	CPLD_UNEXPECTED_VAL_TRIGGER_CAUSE = 0b100,
	POWER_ON_SEQUENCE_TRIGGER_CAUSE = 0b001,
	AC_ON_TRIGGER_CAUSE = 0b010,
	DC_ON_TRIGGER_CAUSE = 0b011,
	MAX_TRIGGER_CAUSE = 0b1000, //trigger cause maxium 3 bit
};

enum VR_UBC_INDEX_E {
	UBC_1 = 1,
	UBC_2,
	VR_1,
	VR_2,
	VR_3,
	VR_4,
	VR_5,
	VR_6,
	VR_7,
	VR_8,
	VR_9,
	VR_10,
	VR_11,
	VR_MAX,
};

#endif
