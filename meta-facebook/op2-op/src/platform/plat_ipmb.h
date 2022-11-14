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

#define IPMB_HD_BIC_BUS I2C_BUS7
#define IPMB_OPA_BIC_BUS I2C_BUS7
#define IPMB_OPB_BIC_BUS I2C_BUS8

#define SELF_I2C_ADDRESS 0x20
#define HD_BIC_I2C_ADDRESS 0x20
#define OPA_BIC_I2C_ADDRESS 0x20
#define OPB_BIC_I2C_ADDRESS 0x20
#define MAX_IPMB_IDX 4

enum {
	HD_BIC_IPMB_IDX,
	OPA_BIC_IPMB_IDX,
	OPB_BIC_IPMB_IDX,
};

extern IPMB_config pal_IPMB_config_table[];

#endif
