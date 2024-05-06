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

#ifndef __INA238__
#define __INA238__

#include <stdint.h>
#include "sensor.h"

typedef struct _ina238_init_arg {
	bool is_init;
	// user defined
	double r_shunt; /* Shunt resistor value. Unit: Ohm. */
	uint8_t adc_range; /* IN+ and IN–, 0:±163.84 mV, 1:±40.96 mV */
	uint8_t alert_latch; /*alert_latch, 0:Disable, 1:Enable */
	double i_max; /* Expected maximum current */
	// calculated data don't set 
	double cur_lsb;
} ina238_init_arg;

enum INA238_OFFSET {
	INA238_CFG_OFFSET = 0x00,
	INA238_ADC_CFG_OFFSET = 0x01,
	INA238_SHUNT_CAL_OFFSET = 0x02,
	INA238_VSHUNT_OFFSET = 0x04,
	INA238_VBUS_OFFSET = 0x05,
	INA238_DIETEMP_OFFSET = 0x06,
	INA238_CUR_OFFSET = 0x07,
	INA238_PWR_OFFSET = 0x08,
	INA238_DIAG_ALRT_OFFSET = 0x0B,
	INA238_SOVL_OFFSET = 0x0C,
	INA238_SUVL_OFFSET = 0x0D,
	INA238_BOVL_OFFSET = 0x0E,
	INA238_BUVL_OFFSET = 0x0F,
	INA238_TEMP_LIMIT_OFFSET = 0x10,
	INA238_PWR_LIMIT_OFFSET = 0x11,
	INA238_MANUFACTURER_ID_OFFSET = 0x3E,
	INA238_DEVICE_ID_OFFSET = 0x3F,
};

enum INA238_ADC_RANGE {
	/* IN+ and IN–, 0:±163.84 mV, 1:±40.96 mV */
	INA238_ADC_RANGE_PN_163 = 0x00,
	INA238_ADC_RANGE_PN_40 = 0x10,
};

enum INA238_ALERT_LATCH {
	INA238_ALERT_LATCH_DISABLE,
	INA238_ALERT_LATCH_ENABLE,
};
#endif
