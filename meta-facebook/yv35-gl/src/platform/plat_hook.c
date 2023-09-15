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

adc_asd_init_arg adc_asd_init_args[] = {
	[0] = { .is_init = false,
		.deglitch[0] = { .deglitch_en = true, .upper_bound = 0x33C },
		.deglitch[2] = { .deglitch_en = true, .upper_bound = 0x2DA },
		.deglitch[3] = { .deglitch_en = true, .upper_bound = 0x1D1 },
		.deglitch[4] = { .deglitch_en = true, .upper_bound = 0x1DE },
	},
	[1] = {
		.is_init = false,
		.deglitch[0] = { .deglitch_en = false,					   },
		.deglitch[1] = { .deglitch_en = true, .upper_bound = 0x28C },
		.deglitch[3] = { .deglitch_en = true, .upper_bound = 0x372 },
		.deglitch[5] = { .deglitch_en = true, .upper_bound = 0x213 },
		.deglitch[6] = { .deglitch_en = true, .upper_bound = 0x2F6 },
		.deglitch[7] = { .deglitch_en = true, .upper_bound = 0x31D },
	}
};
adm1278_init_arg adm1278_init_args[] = {
	[0] = { .is_init = false, .config = { 0x3F1C }, .r_sense = 0.25 }
};
mp5990_init_arg mp5990_init_args[] = { [0] = { .is_init = false,
					       .iout_cal_gain = 0x01B3,
					       .iout_oc_fault_limit = 0x0050,
					       .ocw_sc_ref = 0x0AE0 },
				       [1] = { .is_init = false,
					       .iout_cal_gain = 0x021A,
					       .iout_oc_fault_limit = 0x0058,
					       .ocw_sc_ref = 0x0AE0 } };
mp2985_init_arg mp2985_init_args[] = { [0] = { .is_init = false } };

struct tca9548 mux_conf_addr_0xe2[8] = {
	[0] = { .addr = 0xe2, .chan = 0 }, [1] = { .addr = 0xe2, .chan = 1 },
	[2] = { .addr = 0xe2, .chan = 2 }, [3] = { .addr = 0xe2, .chan = 3 },
	[4] = { .addr = 0xe2, .chan = 4 }, [5] = { .addr = 0xe2, .chan = 5 },
	[6] = { .addr = 0xe2, .chan = 6 }, [7] = { .addr = 0xe2, .chan = 7 },
};

dimm_post_proc_arg dimm_post_proc_args[] = {
	[1] = { .dimm_channel = DIMM_CHANNEL_NUM_1, .dimm_number = DIMM_NUMBER_0 },
	[2] = { .dimm_channel = DIMM_CHANNEL_NUM_2, .dimm_number = DIMM_NUMBER_0 },
	[3] = { .dimm_channel = DIMM_CHANNEL_NUM_3, .dimm_number = DIMM_NUMBER_0 },
	[4] = { .dimm_channel = DIMM_CHANNEL_NUM_4, .dimm_number = DIMM_NUMBER_0 },
	[5] = { .dimm_channel = DIMM_CHANNEL_NUM_5, .dimm_number = DIMM_NUMBER_0 },
	[6] = { .dimm_channel = DIMM_CHANNEL_NUM_6, .dimm_number = DIMM_NUMBER_0 },
	[7] = { .dimm_channel = DIMM_CHANNEL_NUM_7, .dimm_number = DIMM_NUMBER_0 },
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
	if (sval->integer > 0) {
		LOG_ERR("CPU margin value should not be positive, return NA");
		return false;
	}

	return true;
}

bool pre_intel_dimm_i3c_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);

	if (get_post_status() == false) {
		return true;
	}

	if (!is_dimm_prsnt_inited()) {
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

bool post_intel_dimm_i3c_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	if (!get_post_status()) {
		return true;
	}

	dimm_post_proc_arg *post_proc_args = (dimm_post_proc_arg *)args;
	sensor_val *sval = (sensor_val *)reading;

	uint8_t addr = 0, write_len = 0, read_len = 0, cmd = 0, read_buf = 0;
	uint8_t write_buf[PECI_WR_PKG_LEN_DWORD] = { 0 };
	int ret = 0;

	// Using PECI WrPkgConfig command to feedback DIMM temperature to CPU
	addr = CPU_PECI_ADDR;
	write_len = PECI_WR_PKG_LEN_DWORD;
	read_len = PECI_WR_PKG_RD_LEN;
	cmd = PECI_CMD_WR_PKG_CFG0;
	write_buf[0] = 0x00;
	write_buf[1] = WRPKG_IDX_DIMM_TEMP;
	write_buf[2] = post_proc_args->dimm_channel;
	write_buf[3] = post_proc_args->dimm_number;
	write_buf[4] = sval->integer;

	ret = peci_write(cmd, addr, read_len, &read_buf, write_len, write_buf);
	if (ret != 0) {
		LOG_ERR("Failed to write DIMM temperature to CPU, ret= %d", ret);
		return false;
	}

	return true;
}
