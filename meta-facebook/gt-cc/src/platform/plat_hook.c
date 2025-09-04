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
#include "libutil.h"
#include "sensor.h"
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "plat_hook.h"
#include "plat_sensor_table.h"

#include "i2c-mux-tca9548.h"
#include "pex89000.h"
#include "pmbus.h"
#include "nvme.h"
#include "plat_pldm_monitor.h"
#include "plat_mctp.h"

#include <logging/log.h>

LOG_MODULE_REGISTER(plat_hook);

#define ADJUST_MP5990_POWER(x) ((x * 1.0004) + 6.5116)
#define ADJUST_MP5990_CURRENT(x) ((x * 0.9993) + 0.6114)
#define ADJUST_LTC4282_POWER(x) ((x * 0.9722) - 16.315)
#define ADJUST_LTC4282_CURRENT(x) ((x * 0.9634) - 1.0236)
#define ADJUST_LTC4286_POWER(x) ((x * 0.95) - 8.6)
#define ADJUST_LTC4286_CURRENT(x) ((x * 0.95) - 0.7)

K_MUTEX_DEFINE(i2c_bus6_mutex);
K_MUTEX_DEFINE(i2c_bus9_mutex);
K_MUTEX_DEFINE(i2c_bus10_mutex);

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
/*
 * MODE: Shunt and bus, continuous
 * SADC/BADC: 128 samples
 * PG: 320 mv
 * BRNG: 16 V
 */
isl28022_init_arg isl28022_nic_sensor_init_args[] = {
	[0] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[1] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[2] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[3] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[4] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[5] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[6] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[7] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
};

ina230_init_arg ina230_nic_sensor_init_args[] = {
	[0] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [1] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [2] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [3] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [4] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [5] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [6] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [7] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  
};

mp5990_init_arg mp5990_hsc_init_args[] = {
	/* MP5990 register value is written by OTP, so set to 0xFFFF to skip setting */
	[0] = { .is_init = false,
		.iout_cal_gain = 0xFFFF,
		.iout_oc_fault_limit = 0xFFFF,
		.ocw_sc_ref = 0xFFFF, 
		},
};

ltc4282_init_arg ltc4282_hsc_init_args[] = {
	[0] = { .is_init = false,
		.r_sense_mohm = 0.142,
		.is_register_setting_needed = 0x00,
		},
};

ltc4286_init_arg ltc4286_hsc_init_args[] = {
	[0] = { .is_init = false, .r_sense_mohm = 0.142, .mfr_config_1 = { 0xFFFF } },
};

adc_asd_init_arg ast_adc_init_args[] = {
	[0] = { .is_init = false,
		.deglitch[0] = { .deglitch_en = true, .upper_bound = 0x248 },
		.deglitch[1] = { .deglitch_en = true, .upper_bound = 0x240 },
		.deglitch[2] = { .deglitch_en = true, .upper_bound = 0x23E },
		.deglitch[3] = { .deglitch_en = true, .upper_bound = 0x208 },
		.deglitch[4] = { .deglitch_en = true, .upper_bound = 0x23E },
		.deglitch[5] = { .deglitch_en = true, .upper_bound = 0x258 },
		.deglitch[6] = { .deglitch_en = true, .upper_bound = 0x258 },
	},
	[1] = {
		.is_init = false,
		.deglitch[6] = { .deglitch_en = true, .upper_bound = 0x258 },
		.deglitch[7] = { .deglitch_en = true, .upper_bound = 0x258 },
	}
};

/*
 * MODE: Shunt and bus, continuous
 * SADC/BADC: 128 samples
 * PG: 320 mv
 * BRNG: 16 V
 */
isl28022_init_arg isl28022_pex_p1v25_sensor_init_args[] = {
	[0] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 1.536,
		.r_shunt = 2,
		.is_init = false },
	[1] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 1.536,
		.r_shunt = 2,
		.is_init = false },
	[2] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 1.536,
		.r_shunt = 2,
		.is_init = false },
	[3] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 1.536,
		.r_shunt = 2,
		.is_init = false },
};

ina230_init_arg ina230_pex_p1v25_sensor_init_args[] = {
	[0] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 1.32,
		.i_max = 16.384
  },
  [1] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 1.32,
		.i_max = 16.384
  },
  [2] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 1.32,
		.i_max = 16.384
  },
  [3] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 1.32,
		.i_max = 16.384
  },
};
/*
 * MODE: Shunt and bus, continuous
 * SADC/BADC: 128 samples
 * PG: 320 mv
 * BRNG: 16 V
 */
isl28022_init_arg isl28022_pex_p1v8_sensor_init_args[] = {
	[0] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b00,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 2.048,
		.r_shunt = 2,
		.is_init = false },
};

ina230_init_arg ina230_pex_p1v8_sensor_init_args[] = {
  [0] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 2.0,
		.i_max = 16.384
  },
};
/*
 * MODE: Shunt and bus, continuous
 * SADC/BADC: 128 samples
 * PG: 320 mv
 * BRNG: 16 V
 */
isl28022_init_arg isl28022_ssd_sensor_init_args[] = {
	[0] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[1] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[2] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[3] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[4] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[5] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[6] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[7] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[8] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[9] = { .config.fields.MODE = 0b111,
		.config.fields.SADC = 0b1111,
		.config.fields.BADC = 0b1111,
		.config.fields.PG = 0b11,
		.config.fields.BRNG = 0b00,
		.aux_control_config.do_config = true,
		.aux_control_config.config.fields.INTREN = 1,
		.bus_volt_threshold_config.do_config = true,
		.bus_volt_threshold_config.max_threshold_limit = 13.312,
		.r_shunt = 2,
		.is_init = false },
	[10] = { .config.fields.MODE = 0b111,
		 .config.fields.SADC = 0b1111,
		 .config.fields.BADC = 0b1111,
		 .config.fields.PG = 0b11,
		 .config.fields.BRNG = 0b00,
		 .aux_control_config.do_config = true,
		 .aux_control_config.config.fields.INTREN = 1,
		 .bus_volt_threshold_config.do_config = true,
		 .bus_volt_threshold_config.max_threshold_limit = 13.312,
		 .r_shunt = 2,
		 .is_init = false },
	[11] = { .config.fields.MODE = 0b111,
		 .config.fields.SADC = 0b1111,
		 .config.fields.BADC = 0b1111,
		 .config.fields.PG = 0b11,
		 .config.fields.BRNG = 0b00,
		 .aux_control_config.do_config = true,
		 .aux_control_config.config.fields.INTREN = 1,
		 .bus_volt_threshold_config.do_config = true,
		 .bus_volt_threshold_config.max_threshold_limit = 13.312,
		 .r_shunt = 2,
		 .is_init = false },
	[12] = { .config.fields.MODE = 0b111,
		 .config.fields.SADC = 0b1111,
		 .config.fields.BADC = 0b1111,
		 .config.fields.PG = 0b11,
		 .config.fields.BRNG = 0b00,
		 .aux_control_config.do_config = true,
		 .aux_control_config.config.fields.INTREN = 1,
		 .bus_volt_threshold_config.do_config = true,
		 .bus_volt_threshold_config.max_threshold_limit = 13.312,
		 .r_shunt = 2,
		 .is_init = false },
	[13] = { .config.fields.MODE = 0b111,
		 .config.fields.SADC = 0b1111,
		 .config.fields.BADC = 0b1111,
		 .config.fields.PG = 0b11,
		 .config.fields.BRNG = 0b00,
		 .aux_control_config.do_config = true,
		 .aux_control_config.config.fields.INTREN = 1,
		 .bus_volt_threshold_config.do_config = true,
		 .bus_volt_threshold_config.max_threshold_limit = 13.312,
		 .r_shunt = 2,
		 .is_init = false },
	[14] = { .config.fields.MODE = 0b111,
		 .config.fields.SADC = 0b1111,
		 .config.fields.BADC = 0b1111,
		 .config.fields.PG = 0b11,
		 .config.fields.BRNG = 0b00,
		 .aux_control_config.do_config = true,
		 .aux_control_config.config.fields.INTREN = 1,
		 .bus_volt_threshold_config.do_config = true,
		 .bus_volt_threshold_config.max_threshold_limit = 13.312,
		 .r_shunt = 2,
		 .is_init = false },
	[15] = { .config.fields.MODE = 0b111,
		 .config.fields.SADC = 0b1111,
		 .config.fields.BADC = 0b1111,
		 .config.fields.PG = 0b11,
		 .config.fields.BRNG = 0b00,
		 .aux_control_config.do_config = true,
		 .aux_control_config.config.fields.INTREN = 1,
		 .bus_volt_threshold_config.do_config = true,
		 .bus_volt_threshold_config.max_threshold_limit = 13.312,
		 .r_shunt = 2,
		 .is_init = false },
};

ina230_init_arg ina230_ssd_sensor_init_args[] = {
  [0] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [1] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [2] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [3] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [4] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [5] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [6] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [7] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [8] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [9] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [10] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [11] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [12] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [13] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [14] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
  [15] = { 
    .is_init = false,
		.config = {
		  .MODE = 0b111, 
			.VSH_CT = 0b100,
			.VBUS_CT = 0b100,
			.AVG = 0b000,
		},
		.alt_cfg = {
			.LEN = 1,
			.BOL = 1,
		},
		.r_shunt = 0.002,
		.alert_value = 13.2,
		.i_max = 16.384
  },
};

pex89000_init_arg pex_sensor_init_args[] = {
	[0] = { .idx = 0, .is_init = false },
	[1] = { .idx = 1, .is_init = false },
	[2] = { .idx = 2, .is_init = false },
	[3] = { .idx = 3, .is_init = false },
};

nct7718w_init_arg nct7718w_init_args[] = {
	[0] = { .is_init = false,
		.rt1_high_alert_temp = 0x46,
		.rt_filter_alert_mode = 0x00,
		.alert_mask = 0x07,
		.configuration = 0x85,
		.rt1_critical_temperature = 0x65,
		.lt_critical_temperature = 0x65 },
};

cx7_init_arg cx7_init_args[] = {
	[0] = { .is_init = false,
		.endpoint = 0x10,
		.sensor_id = 0x0008,
		.re_init_eid_fn = plat_set_dev_endpoint },
	[1] = { .is_init = false,
		.endpoint = 0x11,
		.sensor_id = 0x0008,
		.re_init_eid_fn = plat_set_dev_endpoint },
	[2] = { .is_init = false,
		.endpoint = 0x12,
		.sensor_id = 0x0008,
		.re_init_eid_fn = plat_set_dev_endpoint },
	[3] = { .is_init = false,
		.endpoint = 0x13,
		.sensor_id = 0x0008,
		.re_init_eid_fn = plat_set_dev_endpoint },
	[4] = { .is_init = false,
		.endpoint = 0x14,
		.sensor_id = 0x0008,
		.re_init_eid_fn = plat_set_dev_endpoint },
	[5] = { .is_init = false,
		.endpoint = 0x15,
		.sensor_id = 0x0008,
		.re_init_eid_fn = plat_set_dev_endpoint },
	[6] = { .is_init = false,
		.endpoint = 0x16,
		.sensor_id = 0x0008,
		.re_init_eid_fn = plat_set_dev_endpoint },
	[7] = { .is_init = false,
		.endpoint = 0x17,
		.sensor_id = 0x0008,
		.re_init_eid_fn = plat_set_dev_endpoint },
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
struct tca9548 mux_conf_addr_0xe0[] = {
	[0] = { .addr = 0xe0, .chan = 0 }, [1] = { .addr = 0xe0, .chan = 1 },
	[2] = { .addr = 0xe0, .chan = 2 }, [3] = { .addr = 0xe0, .chan = 3 },
	[4] = { .addr = 0xe0, .chan = 4 }, [5] = { .addr = 0xe0, .chan = 5 },
	[6] = { .addr = 0xe0, .chan = 6 }, [7] = { .addr = 0xe0, .chan = 7 },
};

struct tca9548 mux_conf_addr_0xe2[] = {
	[0] = { .addr = 0xe2, .chan = 0 }, [1] = { .addr = 0xe2, .chan = 1 },
	[2] = { .addr = 0xe2, .chan = 2 }, [3] = { .addr = 0xe2, .chan = 3 },
	[4] = { .addr = 0xe2, .chan = 4 }, [5] = { .addr = 0xe2, .chan = 5 },
	[6] = { .addr = 0xe2, .chan = 6 }, [7] = { .addr = 0xe2, .chan = 7 },
};

vr_pre_proc_arg vr_pre_read_args[] = {
	[0] = { .mux_info_p = &mux_conf_addr_0xe0[6], .vr_page = 0x0 },
	[1] = { .mux_info_p = &mux_conf_addr_0xe0[6], .vr_page = 0x1 },
	[2] = { .mux_info_p = &mux_conf_addr_0xe0[6], .vr_page = 0x0 },
	[3] = { .mux_info_p = &mux_conf_addr_0xe0[6], .vr_page = 0x1 },
};
pex89000_pre_proc_arg pex89000_pre_read_args[] = {
	[0] = { .mux_info_p = &mux_conf_addr_0xe0[0] },
	[1] = { .mux_info_p = &mux_conf_addr_0xe0[1] },
	[2] = { .mux_info_p = &mux_conf_addr_0xe0[2] },
	[3] = { .mux_info_p = &mux_conf_addr_0xe0[3] },
};
/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/

/* VR pre read function
 *
 * set mux and VR page
 *
 * @param sensor_num sensor number
 * @param args pointer to vr_pre_proc_arg
 * @param reading pointer to reading from previous step
 * @retval true if setting mux and page is successful.
 * @retval false if setting mux or page fails.
 */
bool pre_vr_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)args;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	if (!pre_i2c_bus_read((void *)cfg, pre_proc_args->mux_info_p)) {
		LOG_ERR("pre_i2c_bus_read fail");
		return false;
	}

	/* set page */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = pre_proc_args->vr_page;
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Set page fail");
		k_mutex_unlock(&i2c_bus6_mutex);
		return false;
	}
	return true;
}

bool pre_pex89000_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	pex89000_pre_proc_arg *pre_read_args = (pex89000_pre_proc_arg *)args;

	/* Can not access i2c mux and PEX89000 when DC off */
	if (is_mb_dc_on() == false)
		return false;

	if (!pre_i2c_bus_read((void *)cfg, pre_read_args->mux_info_p)) {
		LOG_ERR("Pre_i2c_bus_read fail.");
		return false;
	}
	return true;
}

bool pre_i2c_bus_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	struct k_mutex *mutex = find_bus_mutex(cfg);

	CHECK_NULL_ARG_WITH_RETURN(mutex, false);

	if (k_mutex_lock(mutex, K_MSEC(300))) {
		LOG_ERR("Sensor number 0x%x mutex lock fail", cfg->num);
		return false;
	}

	if (!tca9548_select_chan((void *)cfg, (struct tca9548 *)args)) {
		k_mutex_unlock(mutex);
		return false;
	}

	return true;
}

bool post_i2c_bus_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);
	ARG_UNUSED(reading);

	struct k_mutex *mutex = find_bus_mutex(cfg);

	CHECK_NULL_ARG_WITH_RETURN(mutex, false);

	if (cfg->port == I2C_BUS9) {
		/* Check whether has pre-reading argument, if yes use it to get the i2c mux address. */
		struct tca9548 *p = (struct tca9548 *)cfg->pre_sensor_read_args;

		if (p) {
			/**
       * Because BUS9 has two mux behind 16 E1.S with the same i2c address, 
       * so close all mux channels after the sensor read to avoid conflict with 
       * other devices reading.
       */
			I2C_MSG msg = { 0 };
			uint8_t retry = 5;
			msg.bus = cfg->port;
			msg.target_addr = ((p->addr) >> 1);
			msg.tx_len = 1;
			msg.data[0] = 0x00;

			if (i2c_master_write(&msg, retry)) {
				k_mutex_unlock(mutex);
				LOG_ERR("Close mux address 0x%x channel failed!", p->addr);
				return false;
			}
		}
	}
	if (k_mutex_unlock(mutex)) {
		LOG_ERR("Sensor num 0x%x mutex unlock failed!", cfg->num);
		return false;
	}
	return true;
}

bool post_mp5990_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);

	if (!reading) {
		post_i2c_bus_read((void *)cfg, args, reading);
		return false;
	}

	if (!post_i2c_bus_read((void *)cfg, args, reading))
		return false;

	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);

	/* Adjust the power value for accuracy by power team requirement */
	switch (cfg->offset) {
	case PMBUS_READ_TEMPERATURE_1:
	case PMBUS_READ_VIN:
	case PMBUS_READ_VOUT:
		return true;
	case PMBUS_READ_PIN:
		val = ADJUST_MP5990_POWER(val);
		break;
	case PMBUS_READ_IOUT:
		val = ADJUST_MP5990_CURRENT(val);
		break;
	default:
		return false;
	}

	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

	return true;
}

bool post_ltc4282_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);

	if (!reading) {
		post_i2c_bus_read((void *)cfg, args, reading);
		return false;
	}

	if (!post_i2c_bus_read(cfg, args, reading))
		return false;

	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);

	/* Adjust the power value for accuracy by power team requirement */
	switch (cfg->offset) {
	case LTC4282_VSOURCE_OFFSET:
		return true;
	case LTC4282_VSENSE_OFFSET:
		val = ADJUST_LTC4282_CURRENT(val);
		break;
	case LTC4282_POWER_OFFSET:
		val = ADJUST_LTC4282_POWER(val);
		break;
	default:
		return false;
	}

	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

	return true;
}

bool post_ltc4286_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);

	if (!reading) {
		post_i2c_bus_read((void *)cfg, args, reading);
		return false;
	}

	if (!post_i2c_bus_read((void *)cfg, args, reading))
		return false;

	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);

	/* Adjust the power value for accuracy by power team requirement */
	switch (cfg->offset) {
	case PMBUS_READ_VOUT:
		return true;
	case PMBUS_READ_IOUT:
		val = ADJUST_LTC4286_CURRENT(val);
		break;
	case PMBUS_READ_PIN:
		val = is_mb_dc_on() ? ADJUST_LTC4286_POWER(val) : val;
		break;
	default:
		return false;
	}

	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

	return true;
}

struct k_mutex *find_bus_mutex(sensor_cfg *cfg)
{
	struct k_mutex *mutex = NULL;

	if (cfg->port == I2C_BUS6)
		mutex = &i2c_bus6_mutex;
	else if (cfg->port == I2C_BUS9)
		mutex = &i2c_bus9_mutex;
	else if (cfg->port == I2C_BUS10)
		mutex = &i2c_bus10_mutex;
	else
		return NULL;

	return mutex;
}

void ssd_drive_reinit(void)
{
	disable_sensor_poll();

	for (uint8_t i = 0; i < ARRAY_SIZE(e1s_sensor_table); i++) {
		uint8_t sensor_num = e1s_sensor_table[i];
		sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

		if (!cfg) {
			LOG_ERR("The pointer to sensor number 0x%x cfg is NULL", sensor_num);
			continue;
		}

		if (cfg->read != NULL) {
			continue;
		} else {
			LOG_ERR("sensor number 0x%x cfg->read not init, do reinit", cfg->num);
		}

		if (!nvme_init(cfg)) {
			LOG_ERR("sensor number 0x%x cfg->read reinit fail", cfg->num);
		}
	}
	enable_sensor_poll();
}

bool is_mb_dc_on()
{
	/* SYS_PWR_READY_N is low active,
   * 0 -> power on
   * 1 -> power off
   */
	return !gpio_get(SYS_PWR_READY_N);
}

void set_cx7_init_arg_to_thor2()
{
	cx7_init_args[0].sensor_id = 0x01F4;
	cx7_init_args[1].sensor_id = 0x01F4;
	cx7_init_args[2].sensor_id = 0x01F4;
	cx7_init_args[3].sensor_id = 0x01F4;
	cx7_init_args[4].sensor_id = 0x01F4;
	cx7_init_args[5].sensor_id = 0x01F4;
	cx7_init_args[6].sensor_id = 0x01F4;
	cx7_init_args[7].sensor_id = 0x01F4;
}
