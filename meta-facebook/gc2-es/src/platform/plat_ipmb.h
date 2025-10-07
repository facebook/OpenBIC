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

#ifndef PLAT_IPMB_H
#define PLAT_IPMB_H

#include "plat_i2c.h"
#include "ipmb.h"

#define IPMB_BMC_BUS I2C_BUS7
#define IPMB_ME_BUS I2C_BUS3
#define IPMB_EXP1_BUS I2C_BUS8
#define IPMB_EXP2_BUS I2C_BUS9
#define IPMB_BB_BIC_BUS I2C_BUS8

#define BMC_I2C_ADDRESS 0x10
#define ME_I2C_ADDRESS 0x16
#define SELF_I2C_ADDRESS 0x20
#define BIC1_I2C_ADDRESS 0x20
#define BIC2_I2C_ADDRESS 0x20
#define BB_BIC_I2C_ADDRESS 0x20
#define MAX_IPMB_IDX 5

enum {
	BMC_IPMB_IDX,
	ME_IPMB_IDX,
	EXP1_IPMB_IDX,
	BB_IPMB_IDX = EXP1_IPMB_IDX,
	EXP2_IPMB_IDX,
};

extern IPMB_config pal_IPMB_config_table[];
#endif
