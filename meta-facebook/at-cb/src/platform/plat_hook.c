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

#include "plat_hook.h"
#include <stdio.h>
#include <string.h>
#include <logging/log.h>
#include "sensor.h"
#include "pmbus.h"
#include "libutil.h"
#include "plat_i2c.h"
#include "power_status.h"
#include "common_i2c_mux.h"
#include "i2c-mux-tca9543a.h"
#include "i2c-mux-pi4msd5v9542.h"
#include "plat_sensor_table.h"
#include "i2c-mux-pca954x.h"

LOG_MODULE_REGISTER(plat_hook);

#define PEX_SWITCH_INIT_RETRY_COUNT 3

struct k_mutex xdpe15284_mutex;

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
adc_asd_init_arg adc_asd_init_args[] = {
	[0] = { .is_init = false },
};

adm1272_init_arg adm1272_init_args[] = {
	[0] = { .is_init = false, .is_need_set_pwr_cfg = true, .pwr_monitor_cfg.value = 0x3F3F },
	[1] = { .is_init = false, .is_need_set_pwr_cfg = true, .pwr_monitor_cfg.value = 0x3F3F },
};

ltc4286_init_arg ltc4286_init_args[] = {
	[0] = { .is_init = false, .r_sense_mohm = 0.3, .mfr_config_1 = { 0x5572 } },
	[1] = { .is_init = false, .r_sense_mohm = 0.3, .mfr_config_1 = { 0x5572 } }
};

ina233_init_arg ina233_init_args[] = {
	[0] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[1] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[2] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[3] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[4] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[5] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[6] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[7] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[8] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[9] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[10] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[11] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
};

pex89000_init_arg pex_sensor_init_args[] = {
	[0] = { .idx = 0, .is_init = false },
	[1] = { .idx = 1, .is_init = false },
};

ina233_init_arg accl_ina233_init_args[] = {
	// ACCL 1
	[0] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[1] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[2] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	// ACCL 2
	[3] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[4] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[5] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	// ACCL 3
	[6] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[7] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[8] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	// ACCL 4
	[9] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[10] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[11] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	// ACCL 5
	[12] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[13] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[14] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	// ACCL 6
	[15] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[16] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[17] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	// ACCL 7
	[18] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[19] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[20] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	// ACCL 8
	[21] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[22] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[23] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	// ACCL 9
	[24] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[25] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[26] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	// ACCL 10
	[27] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[28] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[29] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	// ACCL 11
	[30] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[31] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[32] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	// ACCL 12
	[33] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[34] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
	[35] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b0100,
	},
	},
};

sq52205_init_arg sq52205_init_args[] = {
	[0] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	},
	[1] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b111, //set 1024 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	},
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
mux_config tca9543_configs[] = {
	[0] = { .target_addr = 0x70, .channel = TCA9543A_CHANNEL_1 },
	[1] = { .target_addr = 0x71, .channel = TCA9543A_CHANNEL_1 },
};

mux_config pi4msd5v9542_configs[] = {
	[0] = { .target_addr = 0x70, .channel = PI4MSD5V9542_CHANNEL_0 },
	[1] = { .target_addr = 0x70, .channel = PI4MSD5V9542_CHANNEL_1 },
};

mux_config ina233_tca9543_configs[] = {
	[0] = { .target_addr = 0x70, .channel = TCA9543A_CHANNEL_0 },
	[1] = { .target_addr = 0x70, .channel = TCA9543A_CHANNEL_1 },
};

vr_page_cfg xdpe15284_page[] = {
	[0] = { .vr_page = PMBUS_PAGE_0 },
	[1] = { .vr_page = PMBUS_PAGE_1 },
};

mux_config pca9548_configs[] = {
	[0] = { .bus = I2C_BUS7, .target_addr = 0x70, .channel = PCA9548A_CHANNEL_0 },
	[1] = { .bus = I2C_BUS7, .target_addr = 0x70, .channel = PCA9548A_CHANNEL_1 },
	[2] = { .bus = I2C_BUS7, .target_addr = 0x70, .channel = PCA9548A_CHANNEL_2 },
	[3] = { .bus = I2C_BUS7, .target_addr = 0x70, .channel = PCA9548A_CHANNEL_3 },
	[4] = { .bus = I2C_BUS7, .target_addr = 0x70, .channel = PCA9548A_CHANNEL_4 },
	[5] = { .bus = I2C_BUS7, .target_addr = 0x70, .channel = PCA9548A_CHANNEL_5 },
	[6] = { .bus = I2C_BUS8, .target_addr = 0x74, .channel = PCA9548A_CHANNEL_0 },
	[7] = { .bus = I2C_BUS8, .target_addr = 0x74, .channel = PCA9548A_CHANNEL_1 },
	[8] = { .bus = I2C_BUS8, .target_addr = 0x74, .channel = PCA9548A_CHANNEL_2 },
	[9] = { .bus = I2C_BUS8, .target_addr = 0x74, .channel = PCA9548A_CHANNEL_3 },
	[10] = { .bus = I2C_BUS8, .target_addr = 0x74, .channel = PCA9548A_CHANNEL_4 },
	[11] = { .bus = I2C_BUS8, .target_addr = 0x74, .channel = PCA9548A_CHANNEL_5 },
};

mux_config pca9546_configs[] = {
	[0] = { .target_addr = 0x72, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = 0x72, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = 0x72, .channel = PCA9546A_CHANNEL_2 },
	[3] = { .target_addr = 0x72, .channel = PCA9546A_CHANNEL_3 },
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
bool pre_ina233_read(uint8_t sensor_num, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	// Select Channel
	bool ret = true;
	int mutex_status = 0;
	mux_config *pre_args = (mux_config *)args;
	pre_args->bus = sensor_config[sensor_config_index_map[sensor_num]].port;

	struct k_mutex *mutex = get_i2c_mux_mutex(pre_args->bus);
	mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
	if (mutex_status != 0) {
		LOG_ERR("Mutex lock fail, status: %d", mutex_status);
		return false;
	}

	ret = set_mux_channel(*pre_args, MUTEX_LOCK_ENABLE);
	if (ret == false) {
		LOG_ERR("ina233 switch mux fail");
		k_mutex_unlock(mutex);
	}

	return ret;
}

bool post_ina233_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(reading);
	ARG_UNUSED(args);

	int unlock_status = 0;
	uint8_t bus = sensor_config[sensor_config_index_map[sensor_num]].port;

	struct k_mutex *mutex = get_i2c_mux_mutex(bus);
	if (mutex->lock_count != 0) {
		unlock_status = k_mutex_unlock(mutex);
	}

	if (unlock_status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d", unlock_status);
		return false;
	}

	return true;
}

bool pre_xdpe15284_read(uint8_t sensor_num, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	bool ret = false;
	int retry = 3;
	int mutex_status = 0;
	I2C_MSG msg = { 0 };
	vr_page_cfg *xdpe15284_vr_page = (vr_page_cfg *)args;

	mutex_status = k_mutex_lock(&xdpe15284_mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
	if (mutex_status != 0) {
		LOG_ERR("Mutex lock fail, status: %d", mutex_status);
		return false;
	}

	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 2;
	msg.data[0] = PMBUS_PAGE;
	msg.data[1] = xdpe15284_vr_page->vr_page;

	ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Set xdpe15284 page fail, ret: %d", ret);
		return false;
	}

	return true;
}

bool post_xdpe15284_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(reading);
	ARG_UNUSED(args);

	int unlock_status = k_mutex_unlock(&xdpe15284_mutex);
	if (unlock_status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d", unlock_status);
		return false;
	}

	return true;
}

bool pre_pex89000_read(uint8_t sensor_num, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	/* Can not access i2c mux and PEX89000 when DC off */
	if (is_acb_power_good() == false) {
		return false;
	}

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	pex89000_init_arg *pex_init_arg = (pex89000_init_arg *)cfg->init_args;

	bool ret = true;
	int mutex_status = 0;
	static uint8_t check_init_count = 0;
	mux_config *pre_args = (mux_config *)args;
	pre_args->bus = cfg->port;

	struct k_mutex *mutex = get_i2c_mux_mutex(pre_args->bus);
	mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
	if (mutex_status != 0) {
		LOG_ERR("Mutex lock fail, status: %d", mutex_status);
		return false;
	}

	ret = set_mux_channel(*pre_args, MUTEX_LOCK_ENABLE);
	if (ret == false) {
		LOG_ERR("pex switch mux fail");
		k_mutex_unlock(mutex);
		return false;
	}

	if (pex_init_arg->is_init == false) {
		if (check_init_count >= PEX_SWITCH_INIT_RETRY_COUNT) {
			post_pex89000_read(sensor_num, cfg->post_sensor_read_args, NULL);
			return false;
		}

		check_init_count += 1;
		ret = init_drive_type_delayed(cfg);
		if (ret == false) {
			LOG_ERR("pex initial fail");
			post_pex89000_read(sensor_num, cfg->post_sensor_read_args, NULL);
			return ret;
		}
	}

	return ret;
}

bool post_pex89000_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(reading);
	ARG_UNUSED(args);

	int unlock_status = 0;
	uint8_t bus = sensor_config[sensor_config_index_map[sensor_num]].port;

	struct k_mutex *mutex = get_i2c_mux_mutex(bus);
	if (mutex->lock_count != 0) {
		unlock_status = k_mutex_unlock(mutex);
	}

	if (unlock_status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d", unlock_status);
		return false;
	}

	return true;
}

bool pre_accl_mux_switch(uint8_t card_id, uint8_t sensor_num)
{
	bool ret = false;
	mux_config accl_mux = { 0 };
	mux_config channel_mux = { 0 };

	if (get_accl_mux_config(card_id, &accl_mux) != true) {
		return false;
	}

	if (get_mux_channel_config(card_id, sensor_num, &channel_mux) != true) {
		return false;
	}

	int mutex_status = 0;
	struct k_mutex *mutex = get_i2c_mux_mutex(accl_mux.bus);

	mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
	if (mutex_status != 0) {
		LOG_ERR("Mutex lock fail, status: %d", mutex_status);
		return false;
	}

	ret = set_mux_channel(accl_mux, MUTEX_LOCK_ENABLE);
	if (ret == false) {
		LOG_ERR("ACCL switch mux fail");
		k_mutex_unlock(mutex);
		return false;
	}

	ret = set_mux_channel(channel_mux, MUTEX_LOCK_ENABLE);
	if (ret == false) {
		LOG_ERR("ACCL switch mux fail");
		k_mutex_unlock(mutex);
		return false;
	}

	return true;
}

bool post_accl_mux_switch(uint8_t card_id, uint8_t sensor_num)
{
	mux_config accl_mux = { 0 };

	if (get_accl_mux_config(card_id, &accl_mux) != true) {
		return false;
	}

	int unlock_status = 0;
	struct k_mutex *mutex = get_i2c_mux_mutex(accl_mux.bus);
	if (mutex->lock_count != 0) {
		unlock_status = k_mutex_unlock(mutex);
	}

	if (unlock_status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d", unlock_status);
		return false;
	}

	return true;
}
