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

#ifndef HAL_JTAG_H
#define HAL_JTAG_H

#include <drivers/jtag.h>

enum bmc_jtag_endstate {
	JTAG_STATE_TLRESET,
	JTAG_STATE_IDLE,
	JTAG_STATE_SELECTDR,
	JTAG_STATE_CAPTUREDR,
	JTAG_STATE_SHIFTDR,
	JTAG_STATE_EXIT1DR,
	JTAG_STATE_PAUSEDR,
	JTAG_STATE_EXIT2DR,
	JTAG_STATE_UPDATEDR,
	JTAG_STATE_SELECTIR,
	JTAG_STATE_CAPTUREIR,
	JTAG_STATE_SHIFTIR,
	JTAG_STATE_EXIT1IR,
	JTAG_STATE_PAUSEIR,
	JTAG_STATE_EXIT2IR,
	JTAG_STATE_UPDATEIR
};

enum JTAG_OP { JTAG_OP_IR, JTAG_OP_DR };

struct jtag_xfer {
	uint8_t op; // Operation type, e.g. ir, dr
	int length;
	uint8_t tdi[512];
	uint16_t tdi_bits;
	uint8_t tdo[512];
	uint16_t tdo_bits;
	uint8_t end_tap_state;
} __attribute__((__packed__));

void jtag_set_tap(uint8_t data, uint8_t bitlength);
void jtag_shift_data(struct jtag_xfer *xfer);
void jtag_tck_cycle(uint8_t cycle);

#endif
