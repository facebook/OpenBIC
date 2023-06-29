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
#include "plat_dimm.h"

#define RDPKG_IDX_DIMM_TEMP 0x0E

LOG_MODULE_REGISTER(plat_hook);

xdpe15284_pre_read_arg xdpe15284_pre_read_args[] = {
	[0] = { 0x0 },
	[1] = { 0x1 },
};

adc_asd_init_arg adc_asd_init_args[] = { [0] = { .is_init = false } };
adm1278_init_arg adm1278_init_args[] = {
	[0] = { .is_init = false, .config = { 0x3F1C }, .r_sense = 0.25 }
};
mp5990_init_arg mp5990_init_args[] = { [0] = { .is_init = false,
					       .iout_cal_gain = 0x01B3,
					       .iout_oc_fault_limit = 0x0044,
					       .ocw_sc_ref = 0x0AE0 },
				       [1] = { .is_init = false,
					       .iout_cal_gain = 0x021A,
					       .iout_oc_fault_limit = 0x0054,
					       .ocw_sc_ref = 0x0ADF } };
mp2985_init_arg mp2985_init_args[] = { [0] = { .is_init = false } };

struct tca9548 mux_conf_addr_0xe2[8] = {
	[0] = { .addr = 0xe2, .chan = 0 }, [1] = { .addr = 0xe2, .chan = 1 },
	[2] = { .addr = 0xe2, .chan = 2 }, [3] = { .addr = 0xe2, .chan = 3 },
	[4] = { .addr = 0xe2, .chan = 4 }, [5] = { .addr = 0xe2, .chan = 5 },
	[6] = { .addr = 0xe2, .chan = 6 }, [7] = { .addr = 0xe2, .chan = 7 },
};

bool pre_xdpe15284_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	xdpe15284_pre_read_arg *pre_read_args = (xdpe15284_pre_read_arg *)args;
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
		return false;
	}
	return true;
}

bool pre_vol_bat3v_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);

	if (cfg->num == SENSOR_NUM_MB_ADC_P3V_BAT_VOLT_V) {
		gpio_set(A_P3V_BAT_SCALED_EN_R, GPIO_HIGH);
		k_msleep(1);
	}

	return true;
}

bool post_vol_bat3v_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);
	ARG_UNUSED(reading);

	if (cfg->num == SENSOR_NUM_MB_ADC_P3V_BAT_VOLT_V)
		gpio_set(A_P3V_BAT_SCALED_EN_R, GPIO_LOW);

	return true;
}

bool pre_nvme_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	if (!tca9548_select_chan((void *)cfg, (struct tca9548 *)args))
		return false;

	return true;
}
bool post_cpu_margin_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);

	if (!reading)
		return check_reading_pointer_null_is_allowed(cfg);

	sensor_val *sval = (sensor_val *)reading;
	// The margin sensor should be shown as negative value in BMC.
	sval->integer = -sval->integer;
	return true;
}

bool pre_intel_peci_dimm_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);

	if (get_post_status() == false) {
		return true;
	}

	if (!is_dimm_inited()) {
		return true;
	}

	//Check DIMM is present.
	uint8_t dimm_id = DIMM_ID_UNKNOWN;

	dimm_id = sensor_num_map_dimm_id(cfg->num);
	if (dimm_id == DIMM_ID_UNKNOWN) {
		LOG_ERR("DIMM id is unknown");
		return false;
	}

	if (!get_dimm_presence_status(dimm_id)) {
		return false;
	}

	return true;
}
