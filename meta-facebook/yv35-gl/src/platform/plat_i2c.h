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
#define I2C_BUS1 0
#define I2C_BUS2 1
#define I2C_BUS3 2
#define I2C_BUS4 3
#define I2C_BUS5 4
#define I2C_BUS6 5
#define I2C_BUS7 6
#define I2C_BUS8 7
#define I2C_BUS9 8

#define IPMB_I2C_BMC I2C_BUS7
#define I2C_BUS_MAX_NUM 9

// Server board CPLD difinition
#define SB_CPLD_BUS I2C_BUS1
#define SB_CPLD_ADDR 0x21
#define SB_CPLD_SPI_OOB_FROM_CPU 0x02
#define SB_CPLD_REG_BOARD_CLASS_TYPE 0x05
#define SB_CPLD_REG_BOARD_REVISION_ID 0x08
#define SB_CPLD_REG_1OU_CARD_DETECTION 0x09
#define SB_CPLD_SPI_OOB_FROM_BIC 0x0B
#define SB_CPLD_REG_SPI_OOB_CONTROL 0x0C
#define SB_CPLD_REG_ADR_MODE0_GPIO_CTRL 0x16
#define SB_CPLD_REG_POWER_SEQ 0x3

#endif
