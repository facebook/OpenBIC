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
 * Board-specific I2C bus map for the MCX-N9XX-EVK. hal_i2c's bus
 * indices are firmware-internal (0-based). Both buses are enabled and
 * used here - flexcomm3_lpi2c3 as the controller-mode bus, and
 * flexcomm2_lpi2c2 as the target/slave-mode bus carrying the real
 * IPMB-over-I2C channel (see plat_ipmb.h, common/hal/hal_i2c.c's
 * ipmb_target_register()/ipmb_target_read(), and README.md's "IPMI
 * transport" section). Both buses' SDA/SCL lines are physically on
 * the same Arduino-compatible header, J2 - see README.md for the
 * verified pin table. flexcomm2_lpi2c2 used to crash at boot with a
 * real divide-by-zero in NXP's own MCUX LPI2C driver - see the
 * overlay for the root cause (a FlexComm2 clock conflict with the
 * unused flexcomm2_lpuart2 node) and README.md for the full writeup.
 */

#ifndef PLAT_I2C_H
#define PLAT_I2C_H

#include "hal_i2c.h"

#define I2C_BUS_MAX_NUM 4

#define I2C_BUS_ARDUINO_HEADER 3 /* flexcomm3_lpi2c3, controller mode */
#define I2C_BUS_LCD_HEADER 2 /* flexcomm2_lpi2c2, target mode */

#endif
