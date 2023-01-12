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
#include "plat_apml.h"
#include "power_status.h"

#define ADJUST_ADM1278_CURRENT(x) (x * 0.94)
#define ADJUST_ADM1278_POWER(x) (x * 0.95)
#define ADJUST_LTC4282_CURRENT(x) ((x * 0.96) - 0.04)
#define ADJUST_LTC4282_POWER(x) ((x * 0.96) - 0.6)

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
		.iout_cal_gain = 0x0140,
		.iout_oc_fault_limit = 0x0032,
		.ocw_sc_ref = 0x0FD8 },
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

vr_pre_proc_arg vr_pre_read_args[] = {
	[0] = { 0x0 },
	[1] = { 0x1 },
};

apml_mailbox_init_arg apml_mailbox_init_args[] = { [0] = { .data = 0x00000000, .retry = 0 } };

ddr5_init_temp_arg ddr5_init_temp_args[] = {
	[0] = { .HID_code = 0x00, .port_number = 0 }, [1] = { .HID_code = 0x02, .port_number = 0 },
	[2] = { .HID_code = 0x01, .port_number = 0 }, [3] = { .HID_code = 0x03, .port_number = 0 },
	[4] = { .HID_code = 0x00, .port_number = 1 }, [5] = { .HID_code = 0x02, .port_number = 1 },
	[6] = { .HID_code = 0x01, .port_number = 1 }, [7] = { .HID_code = 0x03, .port_number = 1 },
};

ddr5_init_power_arg ddr5_init_power_args[] = {
	[0] = { .HID_code = 0x00, .port_number = 0 }, [1] = { .HID_code = 0x02, .port_number = 0 },
	[2] = { .HID_code = 0x01, .port_number = 0 }, [3] = { .HID_code = 0x03, .port_number = 0 },
	[4] = { .HID_code = 0x00, .port_number = 1 }, [5] = { .HID_code = 0x02, .port_number = 1 },
	[6] = { .HID_code = 0x01, .port_number = 1 }, [7] = { .HID_code = 0x03, .port_number = 1 },
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/

bool pre_nvme_read(uint8_t sensor_num, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(args, false);
	return tca9548_select_chan(sensor_num, (struct tca9548 *)args);
}

bool pre_vr_read(uint8_t sensor_num, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	vr_pre_proc_arg *pre_proc_args = (vr_pre_proc_arg *)args;
	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	uint8_t retry = 5;
	I2C_MSG msg;

	/* set page */
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = pre_proc_args->vr_page;
	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("Failed to set VR page, sensor_num 0x%x \n", sensor_num);
		return false;
	}
	return true;
}

bool pre_vol_bat3v_read(uint8_t sensor_num, void *args)
{
	ARG_UNUSED(args);

	if (sensor_num == SENSOR_NUM_VOL_P3V_BAT) {
		gpio_set(P3V_BAT_SCALED_EN_R, GPIO_HIGH);
		k_msleep(1);
	}

	return true;
}

bool post_vol_bat3v_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(args);
	ARG_UNUSED(reading);

	if (sensor_num == SENSOR_NUM_VOL_P3V_BAT) {
		gpio_set(P3V_BAT_SCALED_EN_R, GPIO_LOW);
		k_msleep(1);
	}

	return true;
}

bool post_adm1278_cur_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(reading, false);

	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);
	val = ADJUST_ADM1278_CURRENT(val);
	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

	return true;
}

bool post_adm1278_pwr_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(reading, false);

	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);
	val = ADJUST_ADM1278_POWER(val);
	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

	return true;
}

bool post_ltc4282_cur_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(reading, false);

	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);
	val = ADJUST_LTC4282_CURRENT(val);
	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

	return true;
}

bool post_ltc4282_pwr_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(reading, false);

	sensor_val *sval = (sensor_val *)reading;
	float val = sval->integer + (sval->fraction * 0.001);
	val = ADJUST_LTC4282_POWER(val);
	sval->integer = (int16_t)val;
	sval->fraction = (val - sval->integer) * 1000;

	return true;
}

void apml_report_fail_cb(apml_msg *msg)
{
	CHECK_NULL_ARG(msg);
	CHECK_NULL_ARG(msg->ptr_arg);

	sensor_cfg *cfg = (sensor_cfg *)msg->ptr_arg;
	LOG_ERR("Failed to report DIMM power/temperature to CPU, sensor number 0x%x.", cfg->num);
}

void apml_report_result_check(apml_msg *msg)
{
	CHECK_NULL_ARG(msg);
	CHECK_NULL_ARG(msg->ptr_arg);

	sensor_cfg *cfg = (sensor_cfg *)msg->ptr_arg;
	mailbox_RdData *rddata = (mailbox_RdData *)msg->RdData;

	if (rddata->error_code != SBRMI_MAILBOX_NO_ERR) {
		LOG_ERR("Error when report DIMM power/temperature, error code %d, sensor number 0x%x.",
			rddata->error_code, cfg->num);
	}
}

bool post_ddr5_pwr_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(reading, false);

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	ddr5_init_power_arg *init_arg = cfg->init_args;
	CHECK_NULL_ARG_WITH_RETURN(init_arg, false);

	if (cfg->cache_status != SENSOR_READ_4BYTE_ACUR_SUCCESS) {
		return true;
	}

	/* report DIMM power consumption
	 * Command: 40h
	 * Data in:
	 * 		[7:0]  : DIMM address
	 * 		[16:8] : Update rate
	 * 		[31:17]: DIMM power(mWatt)
	 */
	apml_msg mailbox_msg;
	memset(&mailbox_msg, 0, sizeof(apml_msg));
	mailbox_msg.msg_type = APML_MSG_TYPE_MAILBOX;
	mailbox_msg.bus = I2C_BUS14;
	mailbox_msg.target_addr = APML_ADDR;
	mailbox_msg.error_cb_fn = apml_report_fail_cb;
	mailbox_msg.cb_fn = apml_report_result_check;
	mailbox_msg.ptr_arg = cfg;

	mailbox_WrData *wrdata = (mailbox_WrData *)mailbox_msg.WrData;
	report_dimm_power_data_in *data_in = (report_dimm_power_data_in *)wrdata->data_in;
	sensor_val *sval = (sensor_val *)&cfg->cache;
	uint16_t pwr_mw = (sval->integer * 1000) + sval->fraction;

	wrdata->command = SBRMI_MAILBOX_REPORT_DIMM_POWER;
	data_in->dimm_addr = (init_arg->HID_code & 0x07) | ((init_arg->port_number << 4) & 0x30);
	data_in->update_rate = 0x1FF;
	data_in->dimm_power_mw = pwr_mw & 0x7FFF;

	apml_read(&mailbox_msg);
	return true;
}

bool post_ddr5_temp_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(reading, false);

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];
	ddr5_init_temp_arg *init_arg = cfg->init_args;
	CHECK_NULL_ARG_WITH_RETURN(init_arg, false);

	if (cfg->cache_status != SENSOR_READ_4BYTE_ACUR_SUCCESS) {
		return true;
	}

	/* report DIMM thermal sensor(TS0/TS1)
	 * Command: 41h
	 * Data in:
	 * 		[7:0]  : DIMM address
	 * 		[16:8] : Update rate
	 * 		[31:21]: Temperature(0.25 degree C/LSB)
	 */
	apml_msg mailbox_msg;
	memset(&mailbox_msg, 0, sizeof(apml_msg));

	mailbox_msg.msg_type = APML_MSG_TYPE_MAILBOX;
	mailbox_msg.bus = I2C_BUS14;
	mailbox_msg.target_addr = APML_ADDR;
	mailbox_msg.error_cb_fn = apml_report_fail_cb;
	mailbox_msg.cb_fn = apml_report_result_check;
	mailbox_msg.ptr_arg = cfg;
	mailbox_WrData *wrdata = (mailbox_WrData *)mailbox_msg.WrData;
	report_dimm_temp_data_in *data_in = (report_dimm_temp_data_in *)wrdata->data_in;

	/* TS0 */
	uint16_t temp = (uint16_t)(init_arg->ts0_temp * 4);

	wrdata->command = SBRMI_MAILBOX_REPORT_DIMM_TEMP;
	data_in->dimm_addr = (init_arg->HID_code & 0x07) | ((init_arg->port_number << 4) & 0x30);
	data_in->update_rate = 0x1FF;
	data_in->dimm_temp = temp & 0x7FF;
	apml_read(&mailbox_msg);

	/* TS1 */
	memset(wrdata, 0, sizeof(mailbox_WrData));
	temp = (uint16_t)(init_arg->ts1_temp * 4);

	wrdata->command = SBRMI_MAILBOX_REPORT_DIMM_TEMP;
	data_in->dimm_addr =
		(init_arg->HID_code & 0x07) | ((init_arg->port_number << 4) & 0x30) | 0x40;
	data_in->update_rate = 0x1FF;
	data_in->dimm_temp = temp & 0x7FF;
	apml_read(&mailbox_msg);

	return true;
}

bool post_amd_tsi_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(args);
	CHECK_NULL_ARG_WITH_RETURN(reading, false);

	static bool is_cpu_throttle_assert = false;
	static uint8_t deassert_count = 0;

	if (!get_tsi_status()) {
		LOG_DBG("TSI not initialized.");
		return true;
	}
	if (!get_post_status()) {
		LOG_DBG("Post code not complete.");
		return true;
	}

	uint8_t tsi_status = 0;
	if (apml_read_byte(I2C_BUS14, TSI_ADDR, SBTSI_STATUS, &tsi_status)) {
		LOG_ERR("Failed to read TSI status");
		return true;
	}

	common_addsel_msg_t sel_msg;
	if ((tsi_status & BIT(4)) && !is_cpu_throttle_assert) {
		deassert_count = 0;
		is_cpu_throttle_assert = true;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
	} else if (!(tsi_status & BIT(4)) && is_cpu_throttle_assert) {
		deassert_count++;
		if (deassert_count < 6) {
			return true;
		}
		is_cpu_throttle_assert = false;
		sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
	} else {
		return true;
	}
	sel_msg.InF_target = BMC_IPMB;
	sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
	sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
	sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_FMTHROTTLE;
	sel_msg.event_data2 = 0xFF;
	sel_msg.event_data3 = 0xFF;
	if (!common_add_sel_evt_record(&sel_msg)) {
		LOG_ERR("[%s] Failed to add FM Throttle sel.\n", __func__);
	}
	return true;
}
