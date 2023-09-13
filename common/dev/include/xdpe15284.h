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

#ifndef XDPE15284_H
#define XDPE15284_H

enum XDPE15284_WRITE_PROTECT_OPTIONAL {
	XDPE15284_ENABLE_WRITE_PROTECT,
	XDPE15284_DISABLE_WRITE_PROTECT,
};

enum XDPE15284_WRITE_PROTECT_REG_VAL {
	XDPE15284_DISABLE_WRITE_PROTECT_VAL = 0x00,
	// XDPE15284 Disable all writes except the WRITE_PROTECT, OPERATION, PAGE, ON_OFF_CONFIG, and VOUT_COMMAND commands
	XDPE15284_DISABLE_ALL_WRITE_EXCEPT_FIVE_COMMANDS_VAL = 0x20,
	// XDPE15284 Disable all writes except the WRITE_PROTECT, OPERATION, and PAGE commands
	XDPE15284_DISABLE_ALL_WRITE_EXCEPT_THREE_COMMANDS_VAL = 0x40,
	// XDPE15284 Disable all writes except the WRITE_PROTECT command
	XDPE15284_DISABLE_ALL_WRITE_EXCEPT_WRITE_PROTECT_VAL = 0x80,
};

bool xdpe15284_get_checksum(uint8_t bus, uint8_t addr, uint8_t *checksum);
bool xdpe15284_lock_reg(uint8_t bus, uint8_t addr);
bool xdpe15284_unlock_reg(uint8_t bus, uint8_t addr);
bool xdpe15284_get_remaining_wr(uint8_t bus, uint8_t addr, uint8_t *data);
bool xdpe15284_enable_write_protect(uint8_t bus, uint8_t addr);
bool xdpe15284_disable_write_protect(uint8_t bus, uint8_t addr);
void xdpe15284_set_write_protect_default_val(uint8_t val);

#endif
