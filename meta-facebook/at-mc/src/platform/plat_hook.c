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
#include "libutil.h"
#include "sensor.h"
#include "pmbus.h"
#include "plat_hook.h"
#include "common_i2c_mux.h"
#include "i2c-mux-pca954x.h"
#include "i2c-mux-pca984x.h"
#include "plat_sensor_table.h"
#include "ltc2991.h"
#include "plat_i2c.h"
#include "plat_ipmi.h"
#include "plat_class.h"
#include "xdpe12284c.h"
#include "plat_dev.h"
#include "plat_mctp.h"
#include "util_sys.h"

LOG_MODULE_REGISTER(plat_hook);

#define VR_A0V8_INIT_OFFSET 0
#define VR_D0V8_INIT_OFFSET 1
#define VR_VDDQCD_INIT_OFFSET 2

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
mp5990_init_arg mp5990_init_args[] = {
	[0] = { .is_init = false,
		.iout_cal_gain = 0x0280,
		.iout_oc_fault_limit = 0x005F,
		.ocw_sc_ref = 0x05A0 },
};

adc_asd_init_arg adc_asd_init_args[] = { [0] = {
						 .is_init = false,
						 .deglitch[0] = { .deglitch_en = true,
								  .upper_bound = 0x2D3 },
						 .deglitch[1] = { .deglitch_en = true,
								  .upper_bound = 0x20D },
						 .deglitch[2] = { .deglitch_en = true,
								  .upper_bound = 0x2AE },
					 } };

sq52205_init_arg sq52205_init_args[] = {
	[0] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	.is_need_accum_config_init = true,
	.accum_config.value = 0x4C04,
	.is_need_set_alert_threshold = false,
	},
	[1] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	.is_need_accum_config_init = true,
	.accum_config.value = 0x4C04,
	.is_need_set_alert_threshold = false,
	},
	[2] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	.is_need_accum_config_init = true,
	.accum_config.value = 0x4C04,
	.is_need_set_alert_threshold = false,
	},
	[3] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	.is_need_accum_config_init = true,
	.accum_config.value = 0x4C04,
	.is_need_set_alert_threshold = false,
	},
	[4] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	.is_need_accum_config_init = true,
	.accum_config.value = 0x4C04,
	.is_need_set_alert_threshold = false,
	},
	[5] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	.is_need_accum_config_init = true,
	.accum_config.value = 0x4C04,
	.is_need_set_alert_threshold = false,
	},
	[6] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	.is_need_accum_config_init = true,
	.accum_config.value = 0x4C04,
	.is_need_set_alert_threshold = false,
	},
	[7] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	.is_need_accum_config_init = true,
	.accum_config.value = 0x4C04,
	.is_need_set_alert_threshold = false,
	},
	[8] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	.is_need_accum_config_init = true,
	.accum_config.value = 0x4C04,
	.is_need_set_alert_threshold = false,
	},
	[9] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	.is_need_accum_config_init = true,
	.accum_config.value = 0x4C04,
	.is_need_set_alert_threshold = false,
	},
	[10] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	.is_need_accum_config_init = true,
	.accum_config.value = 0x4C04,
	.is_need_set_alert_threshold = false,
	},
	[11] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	.is_need_accum_config_init = true,
	.accum_config.value = 0x4C04,
	.is_need_set_alert_threshold = false,
	},
	[12] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	.is_need_accum_config_init = true,
	.accum_config.value = 0x4C04,
	.is_need_set_alert_threshold = false,
	},
	[13] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001,
	.config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b000,
		.reset_bit = 0b0,
	},
	.is_need_accum_config_init = true,
	.accum_config.value = 0x4C04,
	.is_need_set_alert_threshold = false,
	},
};

ina233_init_arg mc_ina233_init_args[] = {
	[0] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[1] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[2] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[3] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[4] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[5] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[6] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[7] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[8] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[9] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[10] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[11] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[12] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[13] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.001, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
};

ina233_init_arg ina233_init_args[] = {
	// JCN11
	[0] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[1] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	// JCN12
	[2] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[3] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	// JCN9
	[4] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[5] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	// JCN10
	[6] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[7] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	// JCN4
	[8] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[9] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	// JCN3
	[10] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[11] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	// JCN2
	[12] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[13] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	// JCN1
	[14] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
	},
	.is_need_set_alert_threshold = false,
	},
	[15] = { .is_init = false, .current_lsb = 0.001, .r_shunt = 0.005, .mfr_config_init = true,
	.mfr_config = {
		.operating_mode =0b111,
		.shunt_volt_time = 0b100,
		.bus_volt_time = 0b100,
		.aver_mode = 0b011, //set 64 average times
		.rsvd = 0b0100,
	},
	.is_need_mfr_device_config_init = true,
	.mfr_device_config = {
		.apol = 0b0,
		.alert_behavior = 0b1,
		.ein_autoclear = 0b1,
		.i2c_filt = 0b0,
		.ein_accum = 0b00,
		.rsvd = 0b0,
		.ein_status = 0b0,
  },
	.is_need_set_alert_threshold = false,
	},
};

ltc2991_init_arg ltc2991_init_args[] = {
	// JCN11
	[0] = { .is_init = false,
		.v1_v4_control_operation.value = 0,
		.v5_v8_control_operation.value = 0 },
	// JCN12
	[1] = { .is_init = false,
		.v1_v4_control_operation.value = 0,
		.v5_v8_control_operation.value = 0 },
	// JCN9
	[2] = { .is_init = false,
		.v1_v4_control_operation.value = 0,
		.v5_v8_control_operation.value = 0 },
	// JCN10
	[3] = { .is_init = false,
		.v1_v4_control_operation.value = 0,
		.v5_v8_control_operation.value = 0 },
	// JCN4
	[4] = { .is_init = false,
		.v1_v4_control_operation.value = 0,
		.v5_v8_control_operation.value = 0 },
	// JCN3
	[5] = { .is_init = false,
		.v1_v4_control_operation.value = 0,
		.v5_v8_control_operation.value = 0 },
	// JCN2
	[6] = { .is_init = false,
		.v1_v4_control_operation.value = 0,
		.v5_v8_control_operation.value = 0 },
	// JCN1
	[7] = { .is_init = false,
		.v1_v4_control_operation.value = 0,
		.v5_v8_control_operation.value = 0 },
};

uint8_t plat_monitor_table_arg[] = { CXL_CARD_1, CXL_CARD_2, CXL_CARD_3, CXL_CARD_4,
				     CXL_CARD_5, CXL_CARD_6, CXL_CARD_7, CXL_CARD_8 };

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
/** CXL ID mux config **/
mux_config bus_2_pca9548_configs[] = {
	[0] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_6 },
	[1] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_7 },
	[2] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_4 },
	[3] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_5 },
	[4] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_3 },
	[5] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_2 },
	[6] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_1 },
	[7] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_0 },
};

mux_config bus_4_pca9548_configs[] = {
	[0] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_0 },
	[1] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_1 },
	[2] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_2 },
	[3] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_3 },
	/** JCN 13~14 E1.S mux config **/
	[4] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_5 },
	[5] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_4 },
	[6] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_7 },
	[7] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_6 },
};

/** JCN 1~4, 9~12 E1.S 0 mux config **/
mux_config bus_8_pca9548_configs[] = {
	[0] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_0 },
	[1] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_1 },
	[2] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_2 },
	[3] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_3 },
	[4] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_4 },
	[5] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_5 },
	[6] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_6 },
	[7] = { .target_addr = 0x70, .channel = PCA9548A_CHANNEL_7 },
};

mux_config cxl_mux_configs[] = {
	[0] = { .target_addr = 0x71, .channel = PCA9848_CHANNEL_0 },
	[1] = { .target_addr = 0x71, .channel = PCA9848_CHANNEL_1 },
	[2] = { .target_addr = 0x71, .channel = PCA9848_CHANNEL_2 },
	[3] = { .target_addr = 0x71, .channel = PCA9848_CHANNEL_3 },
	[4] = { .target_addr = 0x71, .channel = PCA9848_CHANNEL_4 },
	[5] = { .target_addr = 0x71, .channel = PCA9848_CHANNEL_5 },
};

pwr_monitor_pre_proc_arg pwr_monitor_pre_proc_args[] = {
	[0] = { { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_0 }, .jcn_number = 0 },
	[1] = { { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_0 }, .jcn_number = 1 },
	[2] = { { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_0 }, .jcn_number = 2 },
	[3] = { { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_0 }, .jcn_number = 3 },
	[4] = { { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_1 }, .jcn_number = 4 },
	[5] = { { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_1 }, .jcn_number = 5 },
	[6] = { { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_1 }, .jcn_number = 6 },
	[7] = { { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_1 }, .jcn_number = 7 },
	[8] = { { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_2 }, .jcn_number = 8 },
	[9] = { { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_2 }, .jcn_number = 9 },
	[10] = { { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_2 }, .jcn_number = 10 },
	[11] = { { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_2 }, .jcn_number = 11 },
	[12] = { { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_3 }, .jcn_number = 12 },
	[13] = { { .target_addr = 0x70, .channel = PCA9546A_CHANNEL_3 }, .jcn_number = 13 },
};

uint8_t pm8702_pre_arg[] = { CXL_CARD_1, CXL_CARD_2, CXL_CARD_3, CXL_CARD_4,
			     CXL_CARD_5, CXL_CARD_6, CXL_CARD_7, CXL_CARD_8 };

vr_pre_arg vr_pre_args[] = {
	[0] = { { .vr_page = PMBUS_PAGE_0 }, .cxl_id = CXL_CARD_1 },
	[1] = { { .vr_page = PMBUS_PAGE_1 }, .cxl_id = CXL_CARD_1 },
	[2] = { { .vr_page = PMBUS_PAGE_0 }, .cxl_id = CXL_CARD_2 },
	[3] = { { .vr_page = PMBUS_PAGE_1 }, .cxl_id = CXL_CARD_2 },
	[4] = { { .vr_page = PMBUS_PAGE_0 }, .cxl_id = CXL_CARD_3 },
	[5] = { { .vr_page = PMBUS_PAGE_1 }, .cxl_id = CXL_CARD_3 },
	[6] = { { .vr_page = PMBUS_PAGE_0 }, .cxl_id = CXL_CARD_4 },
	[7] = { { .vr_page = PMBUS_PAGE_1 }, .cxl_id = CXL_CARD_4 },
	[8] = { { .vr_page = PMBUS_PAGE_0 }, .cxl_id = CXL_CARD_5 },
	[9] = { { .vr_page = PMBUS_PAGE_1 }, .cxl_id = CXL_CARD_5 },
	[10] = { { .vr_page = PMBUS_PAGE_0 }, .cxl_id = CXL_CARD_6 },
	[11] = { { .vr_page = PMBUS_PAGE_1 }, .cxl_id = CXL_CARD_6 },
	[12] = { { .vr_page = PMBUS_PAGE_0 }, .cxl_id = CXL_CARD_7 },
	[13] = { { .vr_page = PMBUS_PAGE_1 }, .cxl_id = CXL_CARD_7 },
	[14] = { { .vr_page = PMBUS_PAGE_0 }, .cxl_id = CXL_CARD_8 },
	[15] = { { .vr_page = PMBUS_PAGE_1 }, .cxl_id = CXL_CARD_8 },
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
bool pre_nvme_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	// Select Channel
	bool ret = true;
	int mutex_status = 0;
	mux_config *pre_args = (mux_config *)args;
	pre_args->bus = cfg->port;

	struct k_mutex *mutex = get_i2c_mux_mutex(pre_args->bus);
	mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
	if (mutex_status != 0) {
		LOG_ERR("Mutex lock fail, status: %d", mutex_status);
		return false;
	}

	ret = set_mux_channel(*pre_args, MUTEX_LOCK_ENABLE);
	if (ret != true) {
		k_mutex_unlock(mutex);
	}

	return ret;
}

bool post_nvme_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(reading);
	ARG_UNUSED(args);

	int unlock_status = 0;
	uint8_t bus = cfg->port;

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

bool pre_sq52205_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	// Select Channel
	bool ret = true;
	bool is_sensor_init_done = false;
	bool is_time_to_polling = false;
	int mutex_status = 0;
	uint8_t card_type;

	pwr_monitor_pre_proc_arg *pre_args = (pwr_monitor_pre_proc_arg *)args;

	if (get_pcie_card_type(pre_args->jcn_number, &card_type)) {
		LOG_ERR("Fail to get card present status");
		cfg->is_enable_polling = false;
		return false;
	}

	if (card_type == CARD_NOT_PRESENT) {
		cfg->is_enable_polling = false;
		return false;
	}

	is_sensor_init_done = get_sensor_init_done_flag();
	is_time_to_polling = is_time_to_poll_card_sensor(pre_args->jcn_number);

	if (is_sensor_init_done) {
		if (is_time_to_polling != true) {
			cfg->cache_status = SENSOR_POLLING_DISABLE;
			return true;
		} else {
			if (cfg->cache_status == SENSOR_POLLING_DISABLE) {
				cfg->cache_status = SENSOR_INIT_STATUS;
			}
		}
	}

	mux_config *mux_args = &(pre_args->bus_3_mux_configs);
	mux_args->bus = cfg->port;

	struct k_mutex *mutex = get_i2c_mux_mutex(mux_args->bus);
	mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
	if (mutex_status != 0) {
		LOG_ERR("Mutex lock fail, status: %d", mutex_status);
		return false;
	}

	ret = set_mux_channel(*mux_args, MUTEX_LOCK_ENABLE);
	if (ret != true) {
		k_mutex_unlock(mutex);
	}

	return ret;
}

bool post_sq52205_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(reading);
	ARG_UNUSED(args);

	int unlock_status = 0;
	uint8_t bus = cfg->port;

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

bool pre_cxl_switch_mux(uint8_t sensor_num, void *arg)
{
	CHECK_NULL_ARG_WITH_RETURN(arg, false);

	mux_config card_mux = { 0 };
	mux_config cxl_mux = { 0 };
	uint8_t *cxl_id = (uint8_t *)arg;

	bool ret = get_pcie_card_mux_config(*cxl_id, sensor_num, &card_mux, &cxl_mux);
	if (ret != true) {
		return ret;
	}

	int mutex_status = 0;
	struct k_mutex *mutex = get_i2c_mux_mutex(MEB_CXL_BUS);
	mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
	if (mutex_status != 0) {
		LOG_ERR("Mutex lock fail, status: %d", mutex_status);
		return false;
	}

	// Switch card mux
	ret = set_mux_channel(card_mux, MUTEX_LOCK_ENABLE);
	if (ret != true) {
		LOG_ERR("Switch card mux fail");
		k_mutex_unlock(mutex);
		return false;
	}

	// Switch cxl mux
	ret = set_mux_channel(cxl_mux, MUTEX_LOCK_ENABLE);
	if (ret != true) {
		LOG_ERR("Switch cxl mux fail");
		k_mutex_unlock(mutex);
		return false;
	}

	return true;
}

bool post_cxl_switch_mux(uint8_t sensor_num, void *arg)
{
	ARG_UNUSED(arg);

	int unlock_status = 0;
	struct k_mutex *mutex = get_i2c_mux_mutex(MEB_CXL_BUS);
	unlock_status = k_mutex_unlock(mutex);
	if (unlock_status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d", unlock_status);
		return false;
	}

	return true;
}

bool pre_cxl_vr_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	vr_pre_arg *pre_arg = (vr_pre_arg *)args;

	uint8_t offset = 0;
	uint8_t retry = 5;
	uint8_t cxl_id = pre_arg->cxl_id;
	I2C_MSG msg = { 0 };

	switch (cfg->target_addr) {
	case CXL_VR_A0V8_ADDR:
		offset = cxl_id * 3 + VR_A0V8_INIT_OFFSET;
		break;
	case CXL_VR_D0V8_ADDR:
		offset = cxl_id * 3 + VR_D0V8_INIT_OFFSET;
		break;
	case CXL_VR_VDDQCD_ADDR:
		offset = cxl_id * 3 + VR_VDDQCD_INIT_OFFSET;
		break;
	default:
		LOG_ERR("Invalid xdpe12284 address: 0x%x, cxl id: 0x%x, sensor num: 0x%x",
			cfg->target_addr, cxl_id, cfg->num);
		return false;
	}

	/* Get CXL VR version */
	if (!cxl_vr_info_table[offset].is_init) {
		if (!xdpe12284c_get_checksum(cfg->port, cfg->target_addr,
					     (cxl_vr_info_table[offset].checksum))) {
			LOG_ERR("cxl id %d %s get checksum failed", cxl_id,
				cfg->target_addr == CXL_VR_A0V8_ADDR   ? "VR_P0V89A" :
				cfg->target_addr == CXL_VR_D0V8_ADDR   ? "VR_P0V8D_PVDDQ_AB" :
				cfg->target_addr == CXL_VR_VDDQCD_ADDR ? "VR_PVDDQ_CD" :
									 "unknown vr");
			return false;
		}

		if (!xdpe12284c_get_remaining_write(
			    cfg->port, cfg->target_addr,
			    (uint16_t *)&(cxl_vr_info_table[offset].remaining_write))) {
			LOG_ERR("cxl id %d %s get remaining write failed", cxl_id,
				cfg->target_addr == CXL_VR_A0V8_ADDR   ? "VR_P0V89A" :
				cfg->target_addr == CXL_VR_D0V8_ADDR   ? "VR_P0V8D_PVDDQ_AB" :
				cfg->target_addr == CXL_VR_VDDQCD_ADDR ? "VR_PVDDQ_CD" :
									 "unknown vr");
			return false;
		}

		cxl_vr_info_table[offset].vendor = VENDOR_INFINEON;
		cxl_vr_info_table[offset].is_init = true;
	}

	/* set page */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = pre_arg->page.vr_page;

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Set page fail");
		return false;
	}
	return true;
}

bool post_cxl_xdpe12284c_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(args);

	if (reading == NULL) {
		return check_reading_pointer_null_is_allowed(cfg);
	}

	sensor_val *sval = (sensor_val *)reading;
	float val = (float)sval->integer + (sval->fraction / 1000.0);
	switch (cfg->offset) {
	case PMBUS_READ_IOUT:
		if (val < (-2)) {
			LOG_ERR("Sensor %x unexpected current reading", cfg->num);
			return false;
		}

		//the tolerance of current is -2 amps.
		if (val < 0) {
			sval->integer = 0;
			sval->fraction = 0;
		}
		break;
	case PMBUS_READ_POUT:
		if (val < (-4)) {
			LOG_ERR("Sensor %x unexpected power reading", cfg->num);
			return false;
		}

		//the tolerance of power is -4 watts.
		if (val < 0) {
			sval->integer = 0;
			sval->fraction = 0;
		}
		break;
	default:
		break;
	}

	return true;
}

bool pre_pm8702_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	uint8_t *cxl_id = (uint8_t *)args;

	if (get_cxl_eid_flag(*cxl_id) != true) {
		/* Not ready to polling PM8702 sensor, so skip */
		cfg->cache_status = SENSOR_POLLING_DISABLE;
	} else {
		if (cfg->cache_status == SENSOR_POLLING_DISABLE) {
			cfg->cache_status = SENSOR_INIT_STATUS;
		}
	}

	switch (cfg->cache_status) {
	case SENSOR_READ_SUCCESS:
	case SENSOR_READ_ACUR_SUCCESS:
	case SENSOR_READ_4BYTE_ACUR_SUCCESS:
		/* Sensor reading success */
	case SENSOR_INIT_STATUS:
	case SENSOR_NOT_ACCESSIBLE:
	case SENSOR_POLLING_DISABLE:
		/* Not ready to polling PM8702 sensor */
		break;
	default:
		return false;
	}

	return true;
}
