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
 * indices are firmware-internal (0-based). Both buses are physically
 * on the same Arduino-compatible header, J2 - see README.md for the
 * verified pin table.
 *
 * flexcomm2_lpi2c2 is the single sideband bus: it carries *both* the
 * IPMB-over-I2C channel (target mode, 7-bit addr 0x20 - see
 * plat_ipmb.h) *and* the MCTP-over-SMBus endpoint (target mode, 7-bit
 * addr 0x10 - see plat_mctp.c), as two independent slave-address
 * matches on the one LPI2C instance. This is the "one physical bus
 * carries IPMI + MCTP/PLDM (+ SPDM)" arrangement. It relies on the
 * LPI2C driver patch that programs address0/address1 and demuxes the
 * slave IRQ by matched address (drivers/i2c/i2c_mcux_lpi2c.c in the
 * wrouwet/zephyr fork) - mainline only matched one address per
 * instance. Used to crash at boot with a real divide-by-zero in NXP's
 * own MCUX LPI2C driver - see the overlay for that root cause (a
 * FlexComm2 clock conflict with the unused flexcomm2_lpuart2 node)
 * and README.md for the full writeup.
 *
 * flexcomm3_lpi2c3 is now controller-mode only (i2c scan/read/write);
 * it no longer hosts an MCTP target. Kept enabled as a spare
 * controller bus on J2 pins 15/17.
 */

#ifndef PLAT_I2C_H
#define PLAT_I2C_H

#include "hal_i2c.h"

#define I2C_BUS_MAX_NUM 4

#define I2C_BUS_ARDUINO_HEADER 3 /* flexcomm3_lpi2c3, controller mode only */
#define I2C_BUS_LCD_HEADER 2 /* flexcomm2_lpi2c2 */

/* Canonical name for the shared sideband bus (IPMB 0x20 + MCTP 0x10). */
#define I2C_BUS_SIDEBAND I2C_BUS_LCD_HEADER

#endif
