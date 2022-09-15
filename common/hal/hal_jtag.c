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

static char *jtag_device = "JTAG1";

void jtag_set_tap(uint8_t data, uint8_t bitlength)
{
	const struct device *dev;
	dev = device_get_binding(jtag_device);
	if (!dev) {
		printf("JTAG device not found\n");
		return;
	}
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
}

void jtag_shift_data(uint16_t Wbit, uint8_t *Wdate, uint16_t Rbit, uint8_t *Rdate, uint8_t lastidx)
{
	const struct device *dev;
	dev = device_get_binding(jtag_device);
	if (!dev) {
		printf("JTAG device not found\n");
		return;
	}

	uint8_t value, tdo_val, TMS_val = 0;
	uint16_t RnWbit, RnWbyte, index;

	RnWbit = (Wbit > Rbit) ? Wbit : Rbit;
	RnWbyte = (RnWbit + 7) >> 3;

	for (index = 0; index < RnWbit; index++) {
		if ((Wbit != 0) && (Rbit != 0)) {
			value = Wdate[index / 8] & 0x01;
			if ((Wbit == 1) && (Rbit == 1)) {
				TMS_val = lastidx;
			}

			jtag_sw_xfer(dev, JTAG_TCK, 0);
			jtag_sw_xfer(dev, JTAG_TDI, value);
			jtag_sw_xfer(dev, JTAG_TMS, TMS_val);
			jtag_sw_xfer(dev, JTAG_TCK, 1);
			jtag_sw_xfer(dev, JTAG_TDI, value);
			jtag_sw_xfer(dev, JTAG_TMS, TMS_val);
			jtag_tdo_get(dev, &tdo_val);

			Wdate[index / 8] = Wdate[index / 8] >> 1;
			Rdate[index / 8] |= tdo_val << index % 8;
			Wbit--;
			Rbit--;
		} else if (Wbit != 0) {
			value = Wdate[index / 8] & 0x01;
			if ((Wbit == 1)) {
				TMS_val = lastidx;
			}

			jtag_sw_xfer(dev, JTAG_TCK, 0);
			jtag_sw_xfer(dev, JTAG_TDI, value);
			jtag_sw_xfer(dev, JTAG_TMS, TMS_val);
			jtag_sw_xfer(dev, JTAG_TCK, 1);
			jtag_sw_xfer(dev, JTAG_TDI, value);
			jtag_sw_xfer(dev, JTAG_TMS, TMS_val);

			Wdate[index / 8] = Wdate[index / 8] >> 1;
			Wbit--;
		} else if (Rbit != 0) {
			if ((Rbit == 1)) {
				TMS_val = lastidx;
			}

			jtag_sw_xfer(dev, JTAG_TCK, 0);
			jtag_sw_xfer(dev, JTAG_TDI, 0);
			jtag_sw_xfer(dev, JTAG_TMS, TMS_val);
			jtag_sw_xfer(dev, JTAG_TCK, 1);
			jtag_sw_xfer(dev, JTAG_TDI, 0);
			jtag_sw_xfer(dev, JTAG_TMS, TMS_val);
			jtag_tdo_get(dev, &tdo_val);

			Rdate[index / 8] |= tdo_val << index % 8;
			Rbit--;
		}
	}
}
