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
#include <logging/log.h>
#include "plat_hook.h"
#include "plat_class.h"
#include "nct7363.h"
LOG_MODULE_REGISTER(plat_hook);

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
adm1272_init_arg adm1272_init_args[] = {
	[0] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 0.3,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	[1] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 0.3,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
};

nct7363_init_arg nct7363_init_args[] = {
    // GPIO setting: Reserved=11, FANINx=10, PWMx=01, GPIOXX=00
    // gpio_dir == 1 : input(default), gpio_dir == 0 : output
    // Fan BD
    [0] = { 
        .is_init = false, 
        .pin_type[13] = NCT7363_PIN_TPYE_FANIN,
        .pin_type[14] = NCT7363_PIN_TPYE_PWM,
        .gpio_dir[2] = 1,
        .gpio_dir[10] = 1,
        .gpio_dir[11] = 1,
        .gpio_dir[12] = 1,
        .gpio_dir[13] = 1,
        .gpio_dir[14] = 1,
        .gpio_dir[15] = 1,
        .fan_poles = 0,
    },	
    // Management BD(no used)
    [1] = { 
        .is_init = false,
        .fan_poles = 0,
    },
    //Backplane BD
    [2] = { 
        .is_init = false,
        .pin_type[0] = NCT7363_PIN_TPYE_PWM,
        .pin_type[1] = NCT7363_PIN_TPYE_PWM,
        .pin_type[2] = NCT7363_PIN_TPYE_PWM,
        .pin_type[3] = NCT7363_PIN_TPYE_FANIN,
        .gpio_dir[0] = 1,
        .gpio_dir[1] = 1,
        .gpio_dir[2] = 1,
        .gpio_dir[3] = 1,
        .gpio_dir[4] = 1,
        .gpio_dir[5] = 1,
        .gpio_dir[6] = 1,
        .gpio_dir[7] = 1,
        .fan_poles = 0,
    },
    //Pump BD
    [3] = { 
        .is_init = false,
        .pin_type[0] = NCT7363_PIN_TPYE_PWM,
        .pin_type[1] = NCT7363_PIN_TPYE_PWM,
        .pin_type[4] = NCT7363_PIN_TPYE_FANIN,
        .pin_type[5] = NCT7363_PIN_TPYE_FANIN,
        .pin_type[6] = NCT7363_PIN_TPYE_FANIN,
        .gpio_dir[0] = 1,
        .gpio_dir[1] = 1,
        .gpio_dir[2] = 1,
        .gpio_dir[3] = 1,
        .gpio_dir[4] = 1,
        .gpio_dir[5] = 1,
        .gpio_dir[6] = 1,
        .gpio_dir[7] = 1,
        .gpio_dir[10] = 1,
        .gpio_dir[11] = 1,
        .gpio_dir[12] = 1,
        .gpio_dir[13] = 1,
        .gpio_dir[14] = 1,
        .gpio_dir[15] = 1,
        .fan_poles = 0,
    },
};
/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
mux_config bus_1_PCA9546A_configs[] = {
	[0] = { .target_addr = 0xE0, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = 0xE0, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = 0xE0, .channel = PCA9546A_CHANNEL_2 },
	[3] = { .target_addr = 0xE0, .channel = PCA9546A_CHANNEL_3 },
};
mux_config bus_2_PCA9546A_configs[] = {
	[0] = { .target_addr = 0xE2, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = 0xE2, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = 0xE2, .channel = PCA9546A_CHANNEL_2 },
	[3] = { .target_addr = 0xE2, .channel = PCA9546A_CHANNEL_3 },
};
mux_config bus_6_PCA9546A_configs[] = {
	[0] = { .target_addr = 0xE4, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = 0xE4, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = 0xE4, .channel = PCA9546A_CHANNEL_2 },
	[3] = { .target_addr = 0xE4, .channel = PCA9546A_CHANNEL_3 },
};
mux_config bus_7_PCA9546A_configs[] = {
	[0] = { .target_addr = 0xE6, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = 0xE6, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = 0xE6, .channel = PCA9546A_CHANNEL_2 },
	[3] = { .target_addr = 0xE6, .channel = PCA9546A_CHANNEL_3 },
};
mux_config bus_8_PCA9546A_configs[] = {
	[0] = { .target_addr = 0xE8, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = 0xE8, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = 0xE8, .channel = PCA9546A_CHANNEL_2 },
};
mux_config bus_9_PCA9546A_configs[] = {
	[0] = { .target_addr = 0xE8, .channel = PCA9546A_CHANNEL_1 }, // sensor box
	[1] = { .target_addr = 0xE8, .channel = PCA9546A_CHANNEL_2 }, // PDB
};
/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/

bool post_adm1272_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);

	if (reading == NULL) {
		return check_reading_pointer_null_is_allowed(cfg);
	}

	sensor_val *sval = (sensor_val *)reading;
	if (cfg->offset == PMBUS_READ_IOUT || cfg->offset == PMBUS_READ_IIN) {
		// Adjust negative current value to zero according to power team suggestion
		if ((int)sval->integer < 0) {
			*reading = 0;
			return true;
		}
	}

	if (cfg->offset == PMBUS_READ_IOUT || cfg->offset == PMBUS_READ_IIN ||
	    cfg->offset == PMBUS_READ_POUT || cfg->offset == PMBUS_READ_PIN) {
		// multiply 98% for accuracy
		float val = ((float)sval->integer + (sval->fraction / 1000.0)) * 0.98;
		sval->integer = (int)val & 0xFFFF;
		sval->fraction = (val - sval->integer) * 1000;
	}

	return true;
}