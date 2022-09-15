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

enum {
	I2C_BUS0,
	I2C_BUS1,
	I2C_BUS2,
	I2C_BUS3,
	I2C_BUS4,
	I2C_BUS5,
	I2C_BUS6,
	I2C_BUS7,
	I2C_BUS8,
	I2C_BUS9,
};

#define IPMB_I2C_BMC I2C_BUS1
#define I2C_BUS_MAX_NUM 10

#endif
