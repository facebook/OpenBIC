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

LOG_MODULE_REGISTER(plat_hook);

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
