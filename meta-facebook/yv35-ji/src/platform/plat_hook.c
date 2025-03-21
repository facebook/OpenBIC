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
#include "sensor.h"
#include "plat_def.h"
#include "plat_class.h"
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "plat_hook.h"
#include "plat_sensor_table.h"
#include "i2c-mux-tca9548.h"
#include "logging/log.h"
#include "libipmi.h"
#include "ipmi.h"
#include "power_status.h"
#ifdef ENABLE_NVIDIA
#include "nvidia.h"
#include "plat_mctp.h"
#endif

#define ADJUST_MP5990_CURRENT(x) (x * 1) // temporary set
#define ADJUST_MP5990_POWER(x) (x * 1) // temporary set
#define ADJUST_RS31380R_CURRENT(x) (x * 1) // temporary set
#define ADJUST_RS31380R_POWER(x) (x * 1) // temporary set

LOG_MODULE_REGISTER(plat_hook);

#define RETIMER_INIT_RETRY_COUNT 3

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
adc_asd_init_arg ast_adc_init_args[] = {
	[0] = {
		.is_init = false,
		.deglitch[0] = { .deglitch_en = true, .upper_bound = 0x333, .lower_bound = 0x29F },
		.deglitch[1] = { .deglitch_en = true, .upper_bound = 0x32B, .lower_bound = 0x290 },
		.deglitch[2] = { .deglitch_en = true, .upper_bound = 0x2BC, .lower_bound = 0x27F },
		.deglitch[3] = { .deglitch_en = true, .upper_bound = 0x1DB, .lower_bound = 0x138 },
		.deglitch[4] = { .deglitch_en = true, .upper_bound = 0x1E9, .lower_bound = 0x170 },
		.deglitch[5] = { .deglitch_en = true, .upper_bound = 0x204, .lower_bound = 0x127 },
		.deglitch[7] = { .deglitch_en = true, .upper_bound = 0x208, .lower_bound = 0x1CF },
	},
	[1] = {
		.is_init = false,
		.deglitch[1] = { .deglitch_en = true, .upper_bound = 0x2BC, .lower_bound = 0x27F },
		.deglitch[2] = { .deglitch_en = true, .upper_bound = 0x208, .lower_bound = 0x1CF },
		.deglitch[3] = { .deglitch_en = true, .upper_bound = 0x0ED, .lower_bound = 0x0BD },
		.deglitch[4] = { .deglitch_en = true, .upper_bound = 0x1CE, .lower_bound = 0x19A },
		.deglitch[5] = { .deglitch_en = true, .upper_bound = 0x322, .lower_bound = 0x2B5 },
		.deglitch[6] = { .deglitch_en = true, .upper_bound = 0x25C, .lower_bound = 0x224 },
		.deglitch[7] = { .deglitch_en = true, .upper_bound = 0x189, .lower_bound = 0x159 },
	}
};

mp5990_init_arg mp5990_init_args[] = {
	[0] = { .is_init = false,
		.iout_cal_gain = 0xFFFF,
		.iout_oc_fault_limit = 0xFFFF,
		.ocw_sc_ref = 0xFFFF },
};

rs31380r_init_arg rs31380r_init_args[] = {
	[0] = { .is_init = false,
		.iout_cal_gain = 0xFFFF,
		.iout_oc_fault_limit = 0xFFFF,
		.ocw_sc_ref = 0xFFFF },
};

ina230_init_arg ina230_init_args[] = {
	[0] = { .is_init = false,
		.config =
			{
				.MODE = 0b111, // Measure voltage of shunt resistor and bus(default).
				.VSH_CT = 0b100, // The Vshunt conversion time is 1.1ms(default).
				.VBUS_CT = 0b100, // The Vbus conversion time is 1.1ms(default).
				.AVG = 0b000, // Average number is 1(default).
			},
		.alt_cfg =
			{
				.LEN = 1, // Alert Latch enabled.
				.BOL = 1,
			},
		.r_shunt = 0.01,
		.alert_value = 13.2, // Unit: Watt, // Unit: Watt
		.i_max = 16.384 },
};

pt5161l_init_arg pt5161l_init_args[] = { [0] = { .is_init = false,
						 .temp_cal_code_pma_a = { 0, 0, 0, 0 },
						 .temp_cal_code_pma_b = { 0, 0, 0, 0 },
						 .temp_cal_code_avg = 0 } };

#ifdef ENABLE_NVIDIA
nv_satmc_init_arg satmc_init_args[] = {
	/* numeric sensor */
	[0] = { .is_init = false,
		.endpoint = MCTP_EID_SATMC,
		.sensor_id = NV_SATMC_SENSOR_NUM_TMP_GRACE },
	[1] = { .is_init = false,
		.endpoint = MCTP_EID_SATMC,
		.sensor_id = NV_SATMC_SENSOR_NUM_PWR_VDD_CPU },
	[2] = { .is_init = false,
		.endpoint = MCTP_EID_SATMC,
		.sensor_id = NV_SATMC_SENSOR_NUM_VOL_VDD_CPU },
	[3] = { .is_init = false,
		.endpoint = MCTP_EID_SATMC,
		.sensor_id = NV_SATMC_SENSOR_NUM_PWR_VDD_SOC },
	[4] = { .is_init = false,
		.endpoint = MCTP_EID_SATMC,
		.sensor_id = NV_SATMC_SENSOR_NUM_VOL_VDD_SOC },
	[5] = { .is_init = false,
		.endpoint = MCTP_EID_SATMC,
		.sensor_id = NV_SATMC_SENSOR_NUM_PWR_GRACE },
	[6] = { .is_init = false,
		.endpoint = MCTP_EID_SATMC,
		.sensor_id = NV_SATMC_SENSOR_NUM_CNT_PAGE_RETIRE },
	/* state sensor */
	[7] = { .endpoint = MCTP_EID_SATMC,
		.sensor_id = NV_SATMC_SENSOR_NUM_CPU_THROT_STATE,
		.state_sensor_idx = 0 },
	[8] = { .endpoint = MCTP_EID_SATMC,
		.sensor_id = NV_SATMC_SENSOR_NUM_POWER_BREAK,
		.state_sensor_idx = 0 },
	[9] = { .endpoint = MCTP_EID_SATMC,
		.sensor_id = NV_SATMC_SENSOR_NUM_SPARE_CH_PRESENCE,
		.state_sensor_idx = 0 },
};
#endif

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
struct tca9548 mux_conf_addr_0xe0[4] = {
	[0] = { .addr = 0xe0, .chan = 0 },
	[1] = { .addr = 0xe0, .chan = 1 },
	[2] = { .addr = 0xe0, .chan = 2 },
	[3] = { .addr = 0xe0, .chan = 3 },
};

struct tca9548 mux_conf_addr_0xe2[2] = {
	[0] = { .addr = 0xe2, .chan = 0 },
	[1] = { .addr = 0xe2, .chan = 1 },
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
bool pre_vol_bat3v_read(sensor_cfg *cfg, void *args)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	if (cfg->num == SENSOR_NUM_VOL_ADC4_P3V_BAT) {
		gpio_set(P3V_BAT_SCALED_EN_R, GPIO_HIGH);
		k_msleep(1);
	}

	return true;
}

bool post_vol_bat3v_read(sensor_cfg *cfg, void *args, int *reading)
{
	ARG_UNUSED(args);
	ARG_UNUSED(reading);
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	if (cfg->num == SENSOR_NUM_VOL_ADC4_P3V_BAT) {
		gpio_set(P3V_BAT_SCALED_EN_R, GPIO_LOW);
		k_msleep(1);
	}

	return true;
}

bool post_mp5990_cur_read(sensor_cfg *cfg, void *args, int *reading)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	if (!reading) {
		return check_reading_pointer_null_is_allowed(cfg);
	}

	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);
	val = ADJUST_MP5990_CURRENT(val);
	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

	return true;
}

bool post_mp5990_pwr_read(sensor_cfg *cfg, void *args, int *reading)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	if (!reading) {
		return check_reading_pointer_null_is_allowed(cfg);
	}

	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);
	val = ADJUST_MP5990_POWER(val);
	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

	return true;
}

bool post_rs31380r_cur_read(sensor_cfg *cfg, void *args, int *reading)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	if (!reading) {
		return check_reading_pointer_null_is_allowed(cfg);
	}

	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);
	val = ADJUST_RS31380R_CURRENT(val);
	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

	return true;
}

bool post_rs31380r_pwr_read(sensor_cfg *cfg, void *args, int *reading)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	if (!reading) {
		return check_reading_pointer_null_is_allowed(cfg);
	}

	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);
	val = ADJUST_RS31380R_POWER(val);
	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

	return true;
}

bool pre_tmp451_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	gpio_set(BIC_TMP_LVSFT_EN, GPIO_HIGH);
	k_msleep(100);

	int retry = 200; // workaround for poc board
	for (int i = 0; i < retry; i++) {
		if (tca9548_select_chan(cfg, (struct tca9548 *)args))
			return true;
	}

	return false;
}

bool pre_tmp75_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	int retry = 200; // workaround for poc board
	for (int i = 0; i < retry; i++) {
		if (tca9548_select_chan(cfg, (struct tca9548 *)args))
			return true;
	}

	return false;
}

bool pre_pt4080l_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	/* Need to switch channel after EVT2 */
	if (get_board_revision() >= SYS_BOARD_EVT2) {
		int retry = 200; // workaround for poc board
		int i = 0;
		for (i = 0; i < retry; i++) {
			if (tca9548_select_chan(cfg, (struct tca9548 *)args))
				break;
		}

		if (i == 200) {
			LOG_ERR("Channel switch failed!");
			return false;
		}
	}

	pt5161l_init_arg *init_arg = (pt5161l_init_arg *)cfg->init_args;
	static uint8_t check_init_count = 0;
	bool ret = true;

	if (init_arg->is_init == false) {
		if (check_init_count >= RETIMER_INIT_RETRY_COUNT) {
			LOG_ERR("retimer initial fail reach max retry");
			return false;
		}

		check_init_count += 1;
		ret = init_drive_type_delayed(cfg);
		if (ret == false) {
			LOG_ERR("retimer initial fail");
			return ret;
		}
	}

	return ret;
}

bool pre_ds160pt801_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	/* Need to switch channel after EVT2 */
	if (get_board_revision() >= SYS_BOARD_EVT2) {
		int retry = 200; // workaround for poc board
		int i = 0;
		for (i = 0; i < retry; i++) {
			if (tca9548_select_chan(cfg, (struct tca9548 *)args))
				break;
		}

		if (i == 200) {
			LOG_ERR("Channel switch failed!");
			return false;
		}
	}

	return true;
}