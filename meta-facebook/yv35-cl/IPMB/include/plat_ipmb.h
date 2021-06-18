/*
 * Copyright (c) Facebook, Inc. and its affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PLAT_IPMB_H
#define PLAT_IPMB_H

#include "plat_i2c.h"

#define IPMB_BMC_BUS  i2c_bus7
#define IPMB_ME_BUS   i2c_bus3
#define IPMB_EXP1_BUS i2c_bus8
#define IPMB_EXP2_BUS i2c_bus9
#define Reserve_BUS 0xff

#define BMC_I2C_ADDRESS  0x10
#define ME_I2C_ADDRESS   0x16
#define Self_I2C_ADDRESS 0x20
#define BIC0_I2C_ADDRESS 0x2A
#define BIC1_I2C_ADDRESS 0x2B
#define Reserve_ADDRESS  0xff 

enum { 
  BMC_IPMB_IDX,
  ME_IPMB_IDX,
  EXP1_IPMB_IDX,
  EXP2_IPMB_IDX,
  MAX_IPMB_IDX,
};

#endif
