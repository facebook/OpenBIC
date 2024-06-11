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

#include <logging/log.h>
#include <drivers/peci.h>

#include "sensor.h"
#include "intel_peci.h"
#include "intel_dimm.h"
#include "power_status.h"
#include "i2c-mux-tca9548.h"
#include "libutil.h"
#include "hal_peci.h"

#include "plat_hook.h"
#include "plat_sensor_table.h"
#include "plat_gpio.h"
#include "plat_isr.h"

LOG_MODULE_REGISTER(plat_hook);

#define ADJUST_P0V85_VOLTAGE(x) (x * 1.01)

adc_asd_init_arg ast_adc_init_args[] = {
	[0] = { .is_init = false,
		.deglitch[0] = { .deglitch_en = false,},
		.deglitch[1] = { .deglitch_en = false,},
		.deglitch[2] = { .deglitch_en = false,},
		.deglitch[3] = { .deglitch_en = false,},
		.deglitch[4] = { .deglitch_en = false,},
		.deglitch[5] = { .deglitch_en = false,},
		.deglitch[6] = { .deglitch_en = false,},
		.deglitch[7] = { .deglitch_en = false,},
	},
	[1] = {
		.is_init = false,
		.deglitch[0] = { .deglitch_en = false,},
		.deglitch[1] = { .deglitch_en = false,},
		.deglitch[2] = { .deglitch_en = false,},
		.deglitch[3] = { .deglitch_en = false,},
	}
};

ina233_init_arg ina233_init_args[] = {
        [0] = {
                .is_init = false,
                .current_lsb = 0.001,
                .r_shunt = 0.005,
                .mfr_config_init = false,
                .is_need_mfr_device_config_init = false,
                .is_need_set_alert_threshold = false,
        },
        [1] = {
                .is_init = false,
                .current_lsb = 0.001,
                .r_shunt = 0.005,
                .mfr_config_init = false,
                .is_need_mfr_device_config_init = false,
                .is_need_set_alert_threshold = false,
        },
};

vr_pre_read_arg vr_pre_read_args[] = {
	[0] = { 0x0 },
	[1] = { 0x1 },
};

/* Setup Byte:
	bit[7](register bit) = 1
	bit[6-4] follow Table of Reference Voltage on max11617 datasheet
	Configuration Byte:
	bit[7](register bit) = 0
	bit[4-1] is for channel selection
*/
max11617_init_arg max11617_init_args[] = {
	[0] = {
		.is_init = false,
		.setup_byte = 0xd2,
		.scalefactor[0] = 2,
		.scalefactor[1] = 2,
		.scalefactor[2] = 1,
		.scalefactor[3] = 1,
		.scalefactor[4] = 1,
		.scalefactor[5] = 1,
		.scalefactor[6] = 1,
	},
};

adc128d818_init_arg adc128d818_init_args[] = {
	[0] = {
		.is_init = false,
		.external_vref = false,
		.scalefactor[0] = 2,
		.scalefactor[1] = 2,
		.scalefactor[2] = 1,
		.scalefactor[3] = 1,
		.scalefactor[4] = 1,
		.scalefactor[5] = 1,
		.scalefactor[6] = 1,
	},
};

bool pre_vr_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	vr_pre_read_arg *pre_read_args = (vr_pre_read_arg *)args;
	uint8_t retry = 5;
	I2C_MSG msg;
	/* set page */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = pre_read_args->vr_page;
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Failed to set page");
		return true;
	}
	return true;
}

bool post_p085v_voltage_read(sensor_cfg *cfg, void *args, int *reading)
{
	ARG_UNUSED(cfg);
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(reading, false);

	sensor_val *sval = (sensor_val *)reading;
	float val = (float)sval->integer + (sval->fraction / 1000.0);

	val = ADJUST_P0V85_VOLTAGE(val);
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;
	return true;
}

bool post_adc128d818_read(sensor_cfg *cfg, void *args, int *reading)
{
	ARG_UNUSED(cfg);
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(reading, false);

	// TODO: return false if ADC128D818 is abnormal
	// Read INT_ADC_2_TI_R_N from IOE4 P17
	// 1: sensor normal
	// 0: sensor abnormal

	// The external pull-up for INT_ADC_2_TI_R_N is not on the board currently.
	// Will check the INT_ADC_2_TI_R_N if the external pull-up is ready.

	return true;
}
