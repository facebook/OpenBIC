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
#include <stdlib.h>
#include <logging/log.h>
#include "ipmi.h"
#include "sensor.h"
#include "plat_i2c.h"
#include "plat_gpio.h"
#include "plat_hook.h"
#include "plat_sensor_table.h"
#include "i2c-mux-tca9548.h"
#include "pmbus.h"
#include "libipmi.h"
#include "plat_sys.h"

LOG_MODULE_DECLARE(sensor);

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
adc_asd_init_arg adc_asd_init_args[] = { [0] = { .is_init = false } };

ltc4282_init_arg ltc4282_init_args[] = { [0] = { .is_init = false, .r_sense_mohm = 0.25 } };

mp5990_init_arg mp5990_init_args[] = {
	[0] = { .is_init = false, .iout_cal_gain = 0x0202, .iout_oc_fault_limit = 0x0028 }
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
struct tca9548 mux_conf_addr_0xe2[8] = {
	[0] = { .addr = 0xe2, .chan = 0 }, [1] = { .addr = 0xe2, .chan = 1 },
	[2] = { .addr = 0xe2, .chan = 2 }, [3] = { .addr = 0xe2, .chan = 3 },
	[4] = { .addr = 0xe2, .chan = 4 }, [5] = { .addr = 0xe2, .chan = 5 },
	[6] = { .addr = 0xe2, .chan = 6 }, [7] = { .addr = 0xe2, .chan = 7 },
};

vr_pre_proc_arg vr_page_select[] = {
	[0] = { 0x0 },
	[1] = { 0x1 },
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/

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
	if (args == NULL) {
		return false;
	}

	if (!tca9548_select_chan(sensor_num, (struct tca9548 *)args)) {
		return false;
	}

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
	if (reading == NULL) {
		return check_reading_pointer_null_is_allowed(sensor_num);
	}
	ARG_UNUSED(args);

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = -sval->integer; /* for BMC minus */
	return true;
}

/* AST ADC pre read function
 *
 * set gpio high if sensor is "SENSOR_NUM_V_BAT"
 *
 * @param sensor_num sensor number
 * @param args pointer to NULL
 * @param reading pointer to reading from previous step
 * @retval true always.
 */
bool pre_vol_bat3v_read(uint8_t sensor_num, void *args)
{
	ARG_UNUSED(args);

	if (sensor_num == SENSOR_NUM_V_BAT) {
		gpio_set(A_P3V_BAT_SCALED_EN, GPIO_HIGH);
		k_msleep(1);
	}

	return true;
}

/* AST ADC post read function
 *
 * set gpio low if sensor is "SENSOR_NUM_V_BAT"
 *
 * @param sensor_num sensor number
 * @param args pointer to NULL
 * @param reading pointer to reading from previous step
 * @retval true always.
 */
bool post_vol_bat3v_read(uint8_t sensor_num, void *args, int *reading)
{
	ARG_UNUSED(args);
	ARG_UNUSED(reading);

	if (sensor_num == SENSOR_NUM_V_BAT)
		gpio_set(A_P3V_BAT_SCALED_EN, GPIO_LOW);

	return true;
}

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
bool pre_vr_read(uint8_t sensor_num, void *args)
{
	if (args == NULL) {
		return false;
	}

	vr_pre_proc_arg *vr_page_sel = (vr_pre_proc_arg *)args;
	uint8_t retry = 5;
	I2C_MSG msg;

	if (k_mutex_lock(&vr_page_mutex, K_MSEC(VR_PAGE_MUTEX_TIMEOUT_MS))) {
		LOG_ERR("[%s] Failed to lock vr page\n", __func__);
		return false;
	}

	/* set page */
	msg.bus = sensor_config[sensor_config_index_map[sensor_num]].port;
	msg.target_addr = sensor_config[sensor_config_index_map[sensor_num]].target_addr;
	msg.tx_len = 2;
	msg.data[0] = 0x00;
	msg.data[1] = vr_page_sel->vr_page;

	if (i2c_master_write(&msg, retry)) {
		LOG_ERR("%s, set page fail\n", __func__);
		if (k_mutex_unlock(&vr_page_mutex)) {
			LOG_ERR("[%s] Failed to unlock vr page\n", __func__);
		}
		return false;
	}
	return true;
}

/* xdpe12284c post read function
 *
 * modify certain sensor value after reading
 *
 * @param sensor_num sensor number
 * @param args pointer to NULL
 * @param reading pointer to reading from previous step
 * @retval true if success.
 */
bool post_xdpe12284c_read(uint8_t sensor_num, void *args, int *reading)
{
	bool ret = true;

	if (reading == NULL) {
		ret = check_reading_pointer_null_is_allowed(sensor_num);
		goto error_exit;
	}
	ARG_UNUSED(args);

	sensor_val *sval = (sensor_val *)reading;
	float val = (float)sval->integer + (sval->fraction / 1000.0);

	switch (sensor_num) {
	case SENSOR_NUM_CUR_PVCCIN_VR:
	case SENSOR_NUM_CUR_PVCCSA_VR:
	case SENSOR_NUM_CUR_PVCCIO_VR:
	case SENSOR_NUM_CUR_P3V3_STBY_VR:
	case SENSOR_NUM_CURR_DIMM_ABC_VR:
	case SENSOR_NUM_CURR_DIMM_DEF_VR:
		if (val < (-2)) {
			LOG_ERR("Sensor %x unexpected current reading\n", sensor_num);
			ret = false;
			goto error_exit;
		}

		//the tolerance of current is -2 amps.
		if ((val >= (-2)) && (val < 0)) {
			sval->integer = 0;
			sval->fraction = 0;
		}
		break;
	case SENSOR_NUM_PWR_PVCCIN_VR:
	case SENSOR_NUM_PWR_PVCCSA_VR:
	case SENSOR_NUM_PWR_PVCCIO_VR:
	case SENSOR_NUM_PWR_P3V3_STBY_VR:
	case SENSOR_NUM_PWR_DIMM_ABC_VR:
	case SENSOR_NUM_PWR_DIMM_DEF_VR:
		if (val < (-4)) {
			LOG_ERR("Sensor %x unexpected power reading\n", sensor_num);
			ret = false;
			goto error_exit;
		}

		//the tolerance of power is -4 watts.
		if ((val >= (-4)) && (val < 0)) {
			sval->integer = 0;
			sval->fraction = 0;
		}
		break;
	case SENSOR_NUM_VOL_P3V3_STBY_VR:
		val *= 2;
		sval->integer = (int)val & 0xFFFF;
		sval->fraction = (val - sval->integer) * 1000;
		break;
	case SENSOR_NUM_VOL_PVCCIO_VR:
		// Check VCCIO UV fault
		check_Infineon_VR_VCCIO_UV_fault(sensor_num);
		break;
	default:
		break;
	}

error_exit:
	if (k_mutex_unlock(&vr_page_mutex)) {
		LOG_ERR("[%s] Failed to unlock vr page\n", __func__);
	}

	return ret;
}

/* isl69254 post read function
 *
 * modify certain sensor value after reading
 *
 * @param sensor_num sensor number
 * @param args pointer to NULL
 * @param reading pointer to reading from previous step
 * @retval true if success.
 */
bool post_isl69254_read(uint8_t sensor_num, void *args, int *reading)
{
	if (k_mutex_unlock(&vr_page_mutex)) {
		LOG_ERR("[%s] Failed to unlock vr page\n", __func__);
	}

	if (reading == NULL) {
		return check_reading_pointer_null_is_allowed(sensor_num);
	}
	ARG_UNUSED(args);

	sensor_val *sval = (sensor_val *)reading;
	float val = (float)sval->integer + (sval->fraction / 1000.0);

	switch (sensor_num) {
	case SENSOR_NUM_CUR_PVCCIN_VR:
	case SENSOR_NUM_CUR_PVCCSA_VR:
	case SENSOR_NUM_CUR_PVCCIO_VR:
	case SENSOR_NUM_CUR_P3V3_STBY_VR:
	case SENSOR_NUM_CURR_DIMM_ABC_VR:
	case SENSOR_NUM_CURR_DIMM_DEF_VR:
		if (val < (-2)) {
			LOG_ERR("Sensor %x unexpected current reading\n", sensor_num);
			return false;
		}

		//the tolerance of current is -2 amps.
		if ((val >= (-2)) && (val < 0)) {
			sval->integer = 0;
			sval->fraction = 0;
		}
		break;
	case SENSOR_NUM_PWR_PVCCIN_VR:
	case SENSOR_NUM_PWR_PVCCSA_VR:
	case SENSOR_NUM_PWR_PVCCIO_VR:
	case SENSOR_NUM_PWR_P3V3_STBY_VR:
	case SENSOR_NUM_PWR_DIMM_ABC_VR:
	case SENSOR_NUM_PWR_DIMM_DEF_VR:
		if (val < (-4)) {
			LOG_ERR("Sensor %x unexpected power reading\n", sensor_num);
			return false;
		}

		//the tolerance of power is -4 watts.
		if ((val >= (-4)) && (val < 0)) {
			sval->integer = 0;
			sval->fraction = 0;
		}
		break;
	case SENSOR_NUM_VOL_P3V3_STBY_VR:
		val = (val * 996 / 499);
		sval->integer = (int)val & 0xFFFF;
		sval->fraction = (val - sval->integer) * 1000;
		break;
	case SENSOR_NUM_VOL_PVCCIO_VR:
		// Check VCCIO UV fault
		check_Renesas_VR_VCCIO_UV_fault(sensor_num);
		break;
	default:
		break;
	}

	return true;
}
