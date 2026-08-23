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
 *
 * IPMB configuration for the MCX-N9XX-EVK: one real IPMB-over-I2C
 * channel, target/slave mode on flexcomm2_lpi2c2 (I2C_BUS_LCD_HEADER -
 * see plat_i2c.h). This board has no real BMC/host to bridge to, so
 * this channel is only exercisable by wiring an external controller
 * (or another MCX-N9XX-EVK acting as controller) to that bus - see
 * README.md's "I2C target mode" section for the verified J2 pinout.
 */

#ifndef PLAT_IPMB_H
#define PLAT_IPMB_H

#include "plat_i2c.h"

/* +1 for the RESERVED_IDX terminator entry channel_index_mapping()
 * (common/service/ipmb/ipmb.c) scans for - see plat_ipmb.c. */
#define MAX_IPMB_IDX 2

#define BIC_IPMB_IDX 0
#define BIC_IPMB_ADDRESS 0x20 /* 7-bit IPMB self address */
#define SELF_I2C_ADDRESS BIC_IPMB_ADDRESS

#define IPMB_0_BUS I2C_BUS_LCD_HEADER
#define IPMB_0_SELF_ADDR BIC_IPMB_ADDRESS

#endif
