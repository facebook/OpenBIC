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
 * indices are firmware-internal (0-based). Only flexcomm3_lpi2c3
 * (Arduino header D14/D15) is actually enabled and used here -
 * flexcomm2_lpi2c2 (LCD-shield connector) is disabled in this board's
 * overlay because it triggers a real divide-by-zero in NXP's own
 * MCUX LPI2C driver at boot on this SoC/Zephyr combination, and
 * nothing in this port needs it anyway - see the overlay and
 * README.md for the full writeup. Nothing is physically attached to
 * the Arduino header on this EVK, so transactions will genuinely
 * NACK/timeout - see README.md for why that's still a meaningful
 * real-hardware milestone.
 */

#ifndef PLAT_I2C_H
#define PLAT_I2C_H

#include "hal_i2c.h"

#define I2C_BUS_MAX_NUM 4

#define I2C_BUS_ARDUINO_HEADER 3 /* flexcomm3_lpi2c3 */

#endif
