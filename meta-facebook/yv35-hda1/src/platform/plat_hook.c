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
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "plat_hook.h"
#include "plat_sensor_table.h"
#include "i2c-mux-tca9548.h"
#include "logging/log.h"
#include "libipmi.h"
#include "ipmi.h"
#include "power_status.h"

#define ADJUST_ADM1278_CURRENT(x) (x)
#define ADJUST_ADM1278_POWER(x) (x)
#define ADJUST_LTC4282_CURRENT(x) (x)
#define ADJUST_LTC4282_POWER(x) (x)
#define ADJUST_MP5990_CURRENT(x) (x)
#define ADJUST_MP5990_POWER(x) (x)

LOG_MODULE_REGISTER(plat_hook);

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
adc_asd_init_arg ast_adc_init_args[] = { [0] = { .is_init = false } };

adm1278_init_arg adm1278_init_args[] = {
	[0] = { .is_init = false, .config = { 0x3F1C }, .r_sense = 0.3 }
};

ltc4282_init_arg ltc4282_init_args[] = { [0] = { .r_sense_mohm = 0.5 } };

mp5990_init_arg mp5990_init_args[] = {
	[0] = { .is_init = false,
		.iout_cal_gain = 0xFFFF,
		.iout_oc_fault_limit = 0xFFFF,
		.ocw_sc_ref = 0xFFFF },
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
struct tca9548 mux_conf_addr_0xe2[] = {
	[0] = { .addr = 0xe2, .chan = 0 }, [1] = { .addr = 0xe2, .chan = 1 },
	[2] = { .addr = 0xe2, .chan = 2 }, [3] = { .addr = 0xe2, .chan = 3 },
	[4] = { .addr = 0xe2, .chan = 4 }, [5] = { .addr = 0xe2, .chan = 5 },
	[6] = { .addr = 0xe2, .chan = 6 }, [7] = { .addr = 0xe2, .chan = 7 },
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/

bool pre_nvme_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);
	return tca9548_select_chan(cfg, (struct tca9548 *)args);
}

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

bool post_adm1278_cur_read(sensor_cfg *cfg, void *args, int *reading)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	if (!reading) {
		return check_reading_pointer_null_is_allowed(cfg);
	}

	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);
	val = ADJUST_ADM1278_CURRENT(val);
	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

	return true;
}

bool post_adm1278_pwr_read(sensor_cfg *cfg, void *args, int *reading)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	if (!reading) {
		return check_reading_pointer_null_is_allowed(cfg);
	}

	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);
	val = ADJUST_ADM1278_POWER(val);
	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

	return true;
}

bool post_ltc4282_cur_read(sensor_cfg *cfg, void *args, int *reading)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	if (!reading) {
		return check_reading_pointer_null_is_allowed(cfg);
	}

	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);
	val = ADJUST_LTC4282_CURRENT(val);
	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

	return true;
}

bool post_ltc4282_pwr_read(sensor_cfg *cfg, void *args, int *reading)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	if (!reading) {
		return check_reading_pointer_null_is_allowed(cfg);
	}

	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);
	val = ADJUST_LTC4282_POWER(val);
	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

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
