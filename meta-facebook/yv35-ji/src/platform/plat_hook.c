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

LOG_MODULE_REGISTER(plat_hook);

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
adc_asd_init_arg ast_adc_init_args[] = { [0] = { .is_init = false } };

mp5990_init_arg mp5990_init_args[] = {
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

#ifdef ENABLE_NVIDIA
nv_satmc_init_arg satmc_init_args[] = {
	/* numeric sensor */
	[0] = { .is_init = false, .endpoint = MCTP_EID_SATMC, .sensor_id = NV_SATMC_SENSOR_NUM_TMP_GRACE },
	[1] = { .is_init = false, .endpoint = MCTP_EID_SATMC, .sensor_id = NV_SATMC_SENSOR_NUM_PWR_VDD_CPU },
	[2] = { .is_init = false, .endpoint = MCTP_EID_SATMC, .sensor_id = NV_SATMC_SENSOR_NUM_VOL_VDD_CPU },
	[3] = { .is_init = false, .endpoint = MCTP_EID_SATMC, .sensor_id = NV_SATMC_SENSOR_NUM_PWR_VDD_SOC },
	[4] = { .is_init = false, .endpoint = MCTP_EID_SATMC, .sensor_id = NV_SATMC_SENSOR_NUM_VOL_VDD_SOC },
	[5] = { .is_init = false, .endpoint = MCTP_EID_SATMC, .sensor_id = NV_SATMC_SENSOR_NUM_PWR_GRACE },
	/* state sensor */
	[6] = { .endpoint = MCTP_EID_SATMC, .sensor_id = NV_SATMC_SENSOR_NUM_CPU_THROT_STATE, .state_sensor_idx = 0 },
	[7] = { .endpoint = MCTP_EID_SATMC, .sensor_id = NV_SATMC_SENSOR_NUM_POWER_BREAK, .state_sensor_idx = 0 },
	[8] = { .endpoint = MCTP_EID_SATMC, .sensor_id = NV_SATMC_SENSOR_NUM_SPARE_CH_PRESENCE, .state_sensor_idx = 0 },
};
#endif

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
struct tca9548 mux_conf_addr_0xe0[4] = {
	[0] = { .addr = 0xe0, .chan = 0 }, [1] = { .addr = 0xe0, .chan = 1 },
	[2] = { .addr = 0xe0, .chan = 2 }, [3] = { .addr = 0xe0, .chan = 3 },
};

struct tca9548 mux_conf_addr_0xe2[2] = {
	[0] = { .addr = 0xe2, .chan = 0 }, [1] = { .addr = 0xe2, .chan = 1 },
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

bool pre_tmp451_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	gpio_set(BIC_TMP_LVSFT_EN, GPIO_HIGH);

	struct tca9548 *p = (struct tca9548 *)args;

	uint8_t retry = 200; //workaround for poc board
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	/* change address to 7-bit */
	msg.target_addr = ((p->addr) >> 1);
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = (1 << (p->chan));

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("I2C master write failed");
		return false;
	}

	return true;
}

bool pre_tmp75_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	struct tca9548 *p = (struct tca9548 *)args;

	uint8_t retry = 200; //workaround for poc board
	I2C_MSG msg = { 0 };

	msg.bus = cfg->port;
	/* change address to 7-bit */
	msg.target_addr = ((p->addr) >> 1);
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = (1 << (p->chan));

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("I2C master write failed");
		return false;
	}

	return true;
}
