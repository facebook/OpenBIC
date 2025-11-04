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

#include <stdio.h>
#include <string.h>
#include "hal_jtag.h"
#include "libutil.h"
#include <logging/log.h>
#include "plat_def.h"

LOG_MODULE_REGISTER(hal_jtag);

#ifdef CONFIG_JTAG_HW_MODE
static const int bmc_jtag_tap_mapping[] = {
	[JTAG_STATE_TLRESET] = TAP_RESET,	[JTAG_STATE_IDLE] = TAP_IDLE,
	[JTAG_STATE_SELECTDR] = TAP_DRSELECT,	[JTAG_STATE_CAPTUREDR] = TAP_DRCAPTURE,
	[JTAG_STATE_SHIFTDR] = TAP_DRSHIFT,	[JTAG_STATE_EXIT1DR] = TAP_DREXIT1,
	[JTAG_STATE_PAUSEDR] = TAP_DRPAUSE,	[JTAG_STATE_EXIT2DR] = TAP_DREXIT2,
	[JTAG_STATE_UPDATEDR] = TAP_DRUPDATE,	[JTAG_STATE_SELECTIR] = TAP_IRSELECT,
	[JTAG_STATE_CAPTUREIR] = TAP_IRCAPTURE, [JTAG_STATE_SHIFTIR] = TAP_IRSHIFT,
	[JTAG_STATE_EXIT1IR] = TAP_IREXIT1,	[JTAG_STATE_PAUSEIR] = TAP_IRPAUSE,
	[JTAG_STATE_EXIT2IR] = TAP_IREXIT2,	[JTAG_STATE_UPDATEIR] = TAP_IRUPDATE,
};
#endif

static char *jtag_device = "JTAG1";

void jtag_tck_cycle(uint8_t cycle)
{
	const struct device *dev;
	dev = device_get_binding(jtag_device);
	if (!dev) {
		LOG_ERR("JTAG device not found");
		return;
	}
	jtag_tck_run(dev, cycle);
}

void jtag_set_tap(uint8_t data, uint8_t bitlength)
{
	const struct device *dev;
	dev = device_get_binding(jtag_device);
	if (!dev) {
		LOG_ERR("JTAG device not found");
		return;
	}

#ifndef CONFIG_JTAG_HW_MODE
	uint8_t value, index;

	for (index = 0; index < bitlength; index++) {
		value = data & 0x01;

		jtag_sw_xfer(dev, JTAG_TCK, 0);
		jtag_sw_xfer(dev, JTAG_TDI, 0);
		jtag_sw_xfer(dev, JTAG_TMS, value);
		jtag_sw_xfer(dev, JTAG_TCK, 1);
		jtag_sw_xfer(dev, JTAG_TDI, 0);
		jtag_sw_xfer(dev, JTAG_TMS, value);

		data = data >> 1;
	}
#else
	/* For hardware mode, we just set the tap state
	 * If the target state is TLRESET, we need to
	 * cycle TCK for at least 5 cycles to ensure
	 * the TAP goes to reset state.
	 * using bitlength parameter for that purpose.
	 */
	jtag_tap_set(dev, bmc_jtag_tap_mapping[data]);
	if (data == JTAG_STATE_TLRESET) {
		jtag_tck_cycle(bitlength);
	}
#endif
}

void jtag_shift_data(struct jtag_xfer *xfer)
{
	CHECK_NULL_ARG(xfer);
	const struct device *dev;
	dev = device_get_binding(jtag_device);
	if (!dev) {
		LOG_ERR("JTAG device not found");
		return;
	}

#ifndef CONFIG_JTAG_HW_MODE
	uint16_t Wbit = xfer->tdi_bits;
	uint16_t Rbit = xfer->tdo_bits;
	uint16_t RnWbit = (Wbit > Rbit) ? Wbit : Rbit;
	uint8_t value = 0, tdo_val = 0, TMS_val = 0;

	for (uint16_t index = 0; index < RnWbit; index++) {
		if ((Wbit != 0) && (Rbit != 0)) {
			value = (xfer->tdi[index / 8] >> (index % 8)) & 0x01;
			if ((Wbit == 1) && (Rbit == 1)) {
				TMS_val = xfer->end_tap_state;
			}

			jtag_sw_xfer(dev, JTAG_TCK, 0);
			jtag_sw_xfer(dev, JTAG_TDI, value);
			jtag_sw_xfer(dev, JTAG_TMS, TMS_val);
			jtag_sw_xfer(dev, JTAG_TCK, 1);
			jtag_sw_xfer(dev, JTAG_TDI, value);
			jtag_sw_xfer(dev, JTAG_TMS, TMS_val);
			jtag_tdo_get(dev, &tdo_val);

			xfer->tdo[index / 8] |= tdo_val << index % 8;
			Wbit--;
			Rbit--;
		} else if (Wbit != 0) {
			value = (xfer->tdi[index / 8] >> (index % 8)) & 0x01;
			if (Wbit == 1) {
				TMS_val = xfer->end_tap_state;
			}

			jtag_sw_xfer(dev, JTAG_TCK, 0);
			jtag_sw_xfer(dev, JTAG_TDI, value);
			jtag_sw_xfer(dev, JTAG_TMS, TMS_val);
			jtag_sw_xfer(dev, JTAG_TCK, 1);
			jtag_sw_xfer(dev, JTAG_TDI, value);
			jtag_sw_xfer(dev, JTAG_TMS, TMS_val);

			Wbit--;
		} else if (Rbit != 0) {
			if (Rbit == 1) {
				TMS_val = xfer->end_tap_state;
			}

			jtag_sw_xfer(dev, JTAG_TCK, 0);
			jtag_sw_xfer(dev, JTAG_TDI, 0);
			jtag_sw_xfer(dev, JTAG_TMS, TMS_val);
			jtag_sw_xfer(dev, JTAG_TCK, 1);
			jtag_sw_xfer(dev, JTAG_TDI, 0);
			jtag_sw_xfer(dev, JTAG_TMS, TMS_val);
			jtag_tdo_get(dev, &tdo_val);

			xfer->tdo[index / 8] |= tdo_val << index % 8;
			Rbit--;
		}
	}
#else
	if (xfer->op == JTAG_OP_IR) {
		jtag_ir_scan(dev, xfer->length, xfer->tdi, xfer->tdo,
			     bmc_jtag_tap_mapping[xfer->end_tap_state]);
	} else if (xfer->op == JTAG_OP_DR) {
		jtag_dr_scan(dev, xfer->length, xfer->tdi, xfer->tdo,
			     bmc_jtag_tap_mapping[xfer->end_tap_state]);
	} else {
		LOG_ERR("Unsupported JTAG operation type: %d", xfer->op);
		return;
	}
#endif
}
