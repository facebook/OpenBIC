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

#ifndef PLAT_I2C_h
#define PLAT_I2C_h

#include "hal_i2c.h"

// map i2c bus to peripherial bus
// i2c peripheral 1 based, as used i2c index 0 in firmware.
enum _i2c_bus_num {
	I2C_BUS1,
	I2C_BUS2,
	I2C_BUS3,
	I2C_BUS4,
	I2C_BUS5,
	I2C_BUS6,
	I2C_BUS7,
	I2C_BUS8,
	I2C_BUS9,
	I2C_BUS10,
	I2C_BUS11,
	I2C_BUS12,
	I2C_BUS13,
	I2C_BUS14,
	I2C_BUS_MAX_NUM,
};

#define SSIF_I2C_BUS I2C_BUS6
#define SSIF_I2C_ADDR 0x20 //8bit

#define CPLD_I2C_ADDR 0x42 //8bit

#define MCTP_I2C_SATMC_BUS I2C_BUS4
#define MCTP_I2C_SATMC_ADDR 0x80 //8bit

#define CPUDVDD_I2C_BUS I2C_BUS3
#define CPUDVDD_I2C_ADDR 0x32 //8bit

#define CPUVDD_I2C_BUS I2C_BUS3
#define CPUVDD_I2C_ADDR 0x4A //8bit

#define SOCVDD_I2C_BUS I2C_BUS3
#define SOCVDD_I2C_ADDR 0x4C //8bit

#endif
