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

#ifndef LATTICE_H
#define LATTICE_H

#include "stdint.h"

typedef enum lattice_dev_type {
	LATTICE_LCMX02_2000HC,
	LATTICE_LCMX02_4000HC,
	LATTICE_LCMX02_7000HC,
	LATTICE_LCMX03_2100C,
	LATTICE_LCMX03_4300C,
	LATTICE_LCMX03_9400C,
	LATTICE_LFMNX_50,
	LATTICE_UNKNOWN,
} lattice_dev_type_t;

enum sector_type {
	CFG0 = 0,
	CFG1,
	UFM0,
	UFM1,
	UFM2,
};

enum {
	CPLD_TAR_I2C = 0x01,
	CPLD_TAR_JTAG = 0x02,
};

struct lattice_img_config {
	unsigned long int QF;
	uint32_t *CF;
	uint32_t CF_Line;
	uint32_t *UFM;
	uint32_t UFM_Line;
	uint32_t Version;
	uint32_t CheckSum;
	uint32_t FEARBits;
	uint32_t FeatureRow;
};

typedef struct lattice_update_config {
	lattice_dev_type_t type;
	uint8_t interface; //i2c or jtag
	uint8_t bus; //i2c/jtag
	uint8_t addr; //i2c
	uint8_t *data; //received data
	uint32_t data_ofs; //received data's ofset
	uint32_t data_len; //received data's length
	uint32_t next_ofs; //next request data's ofset
	uint32_t next_len; //next request data's length
} lattice_update_config_t;

typedef bool (*cpld_i2C_update_func)(lattice_update_config_t *config);
typedef bool (*cpld_jtag_update_func)(lattice_update_config_t *config);

struct lattice_dev_config {
	char name[20];
	uint32_t id;
	cpld_i2C_update_func cpld_i2C_update;
	cpld_jtag_update_func cpld_jtag_update;
};

bool lattice_fwupdate(lattice_update_config_t *config);
bool cpld_i2c_get_id(uint8_t bus, uint8_t addr, uint32_t *dev_id);
bool cpld_i2c_get_usercode(uint8_t bus, uint8_t addr, uint32_t *usercode);
lattice_dev_type_t find_type_by_str(char *str);

#endif
