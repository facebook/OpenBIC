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
#include "pmic.h"
#include "ipmi.h"
#include "sensor.h"
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "plat_hook.h"
#include "plat_sensor_table.h"
#include "pmbus.h"
#include "intel_peci.h"
#include "intel_dimm.h"
#include "power_status.h"

#include "i2c-mux-tca9548.h"

#define ADJUST_ADM1278_POWER(x) (x * 0.98)
#define ADJUST_ADM1278_CURRENT(x) ((x * 0.98) + 0.1)
#define ADJUST_LTC4286_POWER(x) ((x * 0.97) - 8)
#define ADJUST_LTC4286_CURRENT(x) ((x * 0.97) - 0.4)
#define ADJUST_LTC4282_POWER(x) ((x * 0.98) - 0.2)
#define ADJUST_LTC4282_CURRENT(x) ((x * 0.99) - 0.2)

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
adc_asd_init_arg adc_asd_init_args[] = { [0] = { .is_init = false } };

adm1278_init_arg adm1278_init_args[] = {
	[0] = { .is_init = false, .config = { 0x3F1C }, .r_sense = 0.25 }
};
mp5990_init_arg mp5990_init_args[] = { [0] = { .is_init = false,
					       .iout_cal_gain = 0x0104,
					       .iout_oc_fault_limit = 0x0028,
					       .ocw_sc_ref = 0xFFFF },
				       [1] = { .is_init = false,
					       .iout_cal_gain = 0x01BF,
					       .iout_oc_fault_limit = 0x0046,
					       .ocw_sc_ref = 0xFFFF } };
ltc4286_init_arg ltc4286_init_args[] = {
	[0] = { .is_init = false, .r_sense_mohm = 0.25, .mfr_config_1 = { 0x1570 } },
	[1] = { .is_init = false, .r_sense_mohm = 0.25, .mfr_config_1 = { 0x3570 } }
};
ltc4282_init_arg ltc4282_init_args[] = { [0] = { .is_init = false,
						 .r_sense_mohm = 0.25,
						 .is_register_setting_needed = 0x01,
						 .ilim_adjust = { 0x13 } },
					 [1] = { .is_init = false,
						 .r_sense_mohm = 0.25,
						 .is_register_setting_needed = 0x01,
						 .ilim_adjust = { 0x53 } } };

pmic_init_arg pmic_init_args[] = {
	[0] = { .is_init = false, .smbus_bus_identifier = 0x00, .smbus_addr = 0x90 },
	[1] = { .is_init = false, .smbus_bus_identifier = 0x00, .smbus_addr = 0x98 },
	[2] = { .is_init = false, .smbus_bus_identifier = 0x00, .smbus_addr = 0x9C },
	[3] = { .is_init = false, .smbus_bus_identifier = 0x01, .smbus_addr = 0x90 },
	[4] = { .is_init = false, .smbus_bus_identifier = 0x01, .smbus_addr = 0x98 },
	[5] = { .is_init = false, .smbus_bus_identifier = 0x01, .smbus_addr = 0x9C }
};

// R_load is the value of resistance connected to EFUSE , and EFUSE would adjust the reading accuracy according to r_load
max16550a_init_arg max16550a_init_args[] = { [0] = { .r_load = 14000 } };

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
struct tca9548 mux_conf_addr_0xe2[8] = {
	[0] = { .addr = 0xe2, .chan = 0 }, [1] = { .addr = 0xe2, .chan = 1 },
	[2] = { .addr = 0xe2, .chan = 2 }, [3] = { .addr = 0xe2, .chan = 3 },
	[4] = { .addr = 0xe2, .chan = 4 }, [5] = { .addr = 0xe2, .chan = 5 },
	[6] = { .addr = 0xe2, .chan = 6 }, [7] = { .addr = 0xe2, .chan = 7 },
};

isl69259_pre_proc_arg isl69259_pre_read_args[] = {
	[0] = { 0x0 },
	[1] = { 0x1 },
};

dimm_pre_proc_arg dimm_pre_proc_args[] = {
	[0] = { .is_present_checked = false }, [1] = { .is_present_checked = false },
	[2] = { .is_present_checked = false }, [3] = { .is_present_checked = false },
	[4] = { .is_present_checked = false }, [5] = { .is_present_checked = false }
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
/* ISL6925 pre read function
 *
 * set mux and VR page
 *
 * @param sensor_num sensor number
 * @param args pointer to isl69259_pre_proc_arg
 * @param reading pointer to reading from previous step
 * @retval true if setting mux and page is successful.
 * @retval false if setting mux or page fails.
 */
bool pre_isl69259_read(uint8_t sensor_num, void *args)
{
	if (args == NULL) {
		return false;
	}

	isl69259_pre_proc_arg *pre_proc_args = (isl69259_pre_proc_arg *)args;
	uint8_t retry = 5;
	I2C_MSG msg;

	/* set page */
	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = pre_proc_args->vr_page;
	if (i2c_master_write(&msg, retry)) {
		printf("pre_isl69259_read, set page fail\n");
		return false;
	}
	return true;
}

/* NVME pre read function
 *
 * set mux
 *
 * @param sensor_num sensor number
 * @param args pointer to struct tca9548
 * @param reading pointer to reading from previous step
 * @retval true if setting mux is successful.
 * @retval false if setting mux fails.
 */
bool pre_nvme_read(uint8_t sensor_num, void *args)
{
	if (!args)
		return false;
	if (!tca9548_select_chan(sensor_num, (struct tca9548 *)args))
		return false;

	return true;
}

/* AST ADC pre read function
 *
 * set gpio high if sensor is "SENSOR_NUM_VOL_BAT3V"
 *
 * @param sensor_num sensor number
 * @param args pointer to NULL
 * @param reading pointer to reading from previous step
 * @retval true always.
 * @retval false NULL
 */
bool pre_vol_bat3v_read(uint8_t sensor_num, void *args)
{
	ARG_UNUSED(args);

	if (sensor_num == SENSOR_NUM_VOL_BAT3V) {
		gpio_set(A_P3V_BAT_SCALED_EN_R, GPIO_HIGH);
		k_msleep(1);
	}

	return true;
}

bool pre_intel_peci_dimm_read(uint8_t sensor_num, void *args)
{
	if (get_post_status() == false) {
		// BIC can't check DIMM temperature by ME, return true to keep do sensor initial
		return true;
	}

	dimm_pre_proc_arg *pre_proc_args = (dimm_pre_proc_arg *)args;
	if (pre_proc_args->is_present_checked == true) {
		return true;
	}

	bool ret = false;
	uint8_t dimm_present_result = 0;
	sensor_cfg cfg = sensor_config[sensor_config_index_map[sensor_num]];
	switch (cfg.offset) {
	case PECI_TEMP_CHANNEL0_DIMM0:
		ret = check_dimm_present(DIMM_CHANNEL_NUM_0, DIMM_NUMBER_0, &dimm_present_result);
		break;
	case PECI_TEMP_CHANNEL2_DIMM0:
		ret = check_dimm_present(DIMM_CHANNEL_NUM_2, DIMM_NUMBER_0, &dimm_present_result);
		break;
	case PECI_TEMP_CHANNEL3_DIMM0:
		ret = check_dimm_present(DIMM_CHANNEL_NUM_3, DIMM_NUMBER_0, &dimm_present_result);
		break;
	case PECI_TEMP_CHANNEL4_DIMM0:
		ret = check_dimm_present(DIMM_CHANNEL_NUM_4, DIMM_NUMBER_0, &dimm_present_result);
		break;
	case PECI_TEMP_CHANNEL6_DIMM0:
		ret = check_dimm_present(DIMM_CHANNEL_NUM_6, DIMM_NUMBER_0, &dimm_present_result);
		break;
	case PECI_TEMP_CHANNEL7_DIMM0:
		ret = check_dimm_present(DIMM_CHANNEL_NUM_7, DIMM_NUMBER_0, &dimm_present_result);
		break;
	default:
		printf("[%s] input sensor 0x%x offset is invalid, offset: 0x%x\n", __func__,
		       sensor_num, cfg.offset);
		return ret;
	}

	if (ret == false) {
		return ret;
	}

	// Check dimm temperature result, report 0xFF if dimm not present
	if (dimm_present_result == DIMM_NOT_PRESENT) {
		ret = disable_dimm_pmic_sensor(sensor_num);
	}

	pre_proc_args->is_present_checked = true;
	return ret;
}

/* AST ADC post read function
 *
 * set gpio low if sensor is "SENSOR_NUM_VOL_BAT3V"
 *
 * @param sensor_num sensor number
 * @param args pointer to NULL
 * @param reading pointer to reading from previous step
 * @retval true always.
 * @retval false NULL
 */
bool post_vol_bat3v_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(args);
	ARG_UNUSED(reading);

	if (sensor_num == SENSOR_NUM_VOL_BAT3V)
		gpio_set(A_P3V_BAT_SCALED_EN_R, GPIO_LOW);

	return true;
}

/* INTEL PECI post read function
 *
 * modify certain sensor value after reading
 *
 * @param sensor_num sensor number
 * @param args pointer to NULL
 * @param reading pointer to reading from previous step
 * @retval true if no error
 * @retval false if reading get NULL
 */

bool post_cpu_margin_read(uint8_t sensor_num, void *args, int *reading)
{
	if (!reading)
		return check_reading_pointer_null_is_allowed(sensor_num);
	ARG_UNUSED(args);

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = -sval->integer; /* for BMC minus */
	return true;
}

/* ADM1278 post read function
 *
 * modify ADM1278 power value after reading
 *
 * @param sensor_num sensor number
 * @param args pointer to NULL
 * @param reading pointer to reading from previous step
 * @retval true if no error
 * @retval false if reading get NULL
 */
bool post_adm1278_power_read(uint8_t sensor_num, void *args, int *reading)
{
	if (!reading)
		return check_reading_pointer_null_is_allowed(sensor_num);
	ARG_UNUSED(args);

	sensor_val *sval = (sensor_val *)reading;
	float val = (float)sval->integer + (sval->fraction / 1000.0);

	val = ADJUST_ADM1278_POWER(val);
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;
	return true;
}

/* ADM1278 post read function
 *
 * modify ADM1278 current value after reading
 *
 * @param sensor_num sensor number
 * @param args pointer to NULL
 * @param reading pointer to reading from previous step
 * @retval true if no error
 * @retval false if reading get NULL
 */
bool post_adm1278_current_read(uint8_t sensor_num, void *args, int *reading)
{
	if (!reading)
		return check_reading_pointer_null_is_allowed(sensor_num);
	ARG_UNUSED(args);

	sensor_val *sval = (sensor_val *)reading;
	float val = (float)sval->integer + (sval->fraction / 1000.0);

	val = ADJUST_ADM1278_CURRENT(val);
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;
	return true;
}

/* LTC4286 post read function
 *
 * modify LTC4286 current and power value after reading
 *
 * @param sensor_num sensor number
 * @param args pointer to NULL
 * @param reading pointer to reading from previous step
 * @retval true if no error
 * @retval false if reading get NULL or the offset is unknown
 */
bool post_ltc4286_read(uint8_t sensor_num, void *args, int *reading)
{
	if (!reading) {
		return check_reading_pointer_null_is_allowed(sensor_num);
	}
	ARG_UNUSED(args);

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	sensor_val *sval = (sensor_val *)reading;
	float val = (float)sval->integer + (sval->fraction / 1000.0);

	switch (cfg->offset) {
	case PMBUS_READ_VIN:
	case PMBUS_READ_VOUT:
	case PMBUS_READ_TEMPERATURE_1:
		return true;
	case PMBUS_READ_IOUT:
		val = ADJUST_LTC4286_CURRENT(val);
		break;
	case PMBUS_READ_PIN:
		val = ADJUST_LTC4286_POWER(val);
		break;
	default:
		printf("[%s] Unknown register(0x%x)\n", __func__, cfg->offset);
		return false;
	}

	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;
	return true;
}

/* LTC4282 post read function
 *
 * modify LTC4282 current and power value after reading
 *
 * @param sensor_num sensor number
 * @param args pointer to NULL
 * @param reading pointer to reading from previous step
 * @retval true if no error
 * @retval false if reading get NULL
 */
bool post_ltc4282_read(uint8_t sensor_num, void *args, int *reading)
{
	if (!reading) {
		return check_reading_pointer_null_is_allowed(sensor_num);
	}
	ARG_UNUSED(args);

	sensor_cfg *cfg = &sensor_config[sensor_config_index_map[sensor_num]];

	sensor_val *sval = (sensor_val *)reading;
	float val = (float)sval->integer + (sval->fraction / 1000.0);

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

	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;
	return true;
}
