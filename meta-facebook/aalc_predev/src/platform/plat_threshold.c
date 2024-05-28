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

#include "sensor.h"
#include "plat_sensor_table.h"
#include <logging/log.h>

#define THRESHOLD_POLL_STACK_SIZE 1024

struct k_thread threshold_poll;
K_KERNEL_STACK_MEMBER(threshold_poll_stack, THRESHOLD_POLL_STACK_SIZE);

LOG_MODULE_REGISTER(plat_threshold);

static bool threshold_poll_enable_flag = true;

enum THRESHOLD_TYPE {
	THRESHOLD_DISABLE,
	THRESHOLD_ENABLE_LCR,
	THRESHOLD_ENABLE_UCR,
	THRESHOLD_ENABLE_BOTH,
};

enum THRESHOLD_STATUS {
	THRESHOLD_STATUS_NORMAL,
	THRESHOLD_STATUS_LCR,
	THRESHOLD_STATUS_UCR,
};

typedef struct {
	uint8_t sensor_num;
	uint8_t type;
	float lcr;
	float ucr;
	void (*fn)(uint8_t, uint8_t); // para: arg, status
	uint8_t arg;

	// priv data
	uint8_t last_status; // record the last status
} sensor_threshold;

sensor_threshold threshold_tbl[] = {
	{ SENSOR_NUM_BB_TMP75_TEMP_C, THRESHOLD_ENABLE_LCR, 40, 0, NULL, 0 },
	{ SENSOR_NUM_BPB_RPU_OUTLET_TEMP_C, THRESHOLD_ENABLE_LCR, 60, 0, NULL, 0 },
	{ SENSOR_NUM_BPB_RPU_COOLANT_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 65, 0, NULL, 0 },
	{ SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C, THRESHOLD_ENABLE_LCR, 65, 0, NULL, 0 },
	{ SENSOR_NUM_BPB_HEX_WATER_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 65, 0, NULL, 0 },
	{ SENSOR_NUM_MB_RPU_AIR_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 40, 0, NULL, 0 },
	//	{ SENSOR_NUM_PDB_HDC1080DMBR_TEMP_C, None, None, None, NULL, 0 },
	{ SENSOR_NUM_SB_HEX_AIR_OUTLET_1_TEMP_C, THRESHOLD_ENABLE_LCR, 60, 0, NULL, 0 },
	{ SENSOR_NUM_SB_HEX_AIR_OUTLET_2_TEMP_C, THRESHOLD_ENABLE_LCR, 60, 0, NULL, 0 },
	{ SENSOR_NUM_SB_HEX_AIR_OUTLET_3_TEMP_C, THRESHOLD_ENABLE_LCR, 60, 0, NULL, 0 },
	{ SENSOR_NUM_SB_HEX_AIR_OUTLET_4_TEMP_C, THRESHOLD_ENABLE_LCR, 60, 0, NULL, 0 },
	//	{ SENSOR_NUM_BB_HSC_P48V_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_BPB_HSC_P48V_TEMP_C, None, None, None, NULL, 0 },
	{ SENSOR_NUM_FB_1_HEX_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 40, 0, NULL, 0 },
	{ SENSOR_NUM_FB_2_HEX_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 40, 0, NULL, 0 },
	{ SENSOR_NUM_FB_3_HEX_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 40, 0, NULL, 0 },
	{ SENSOR_NUM_FB_4_HEX_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 40, 0, NULL, 0 },
	{ SENSOR_NUM_FB_5_HEX_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 40, 0, NULL, 0 },
	{ SENSOR_NUM_FB_6_HEX_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 40, 0, NULL, 0 },
	{ SENSOR_NUM_FB_7_HEX_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 40, 0, NULL, 0 },
	{ SENSOR_NUM_FB_8_HEX_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 40, 0, NULL, 0 },
	{ SENSOR_NUM_FB_9_HEX_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 40, 0, NULL, 0 },
	{ SENSOR_NUM_FB_10_HEX_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 40, 0, NULL, 0 },
	{ SENSOR_NUM_FB_11_HEX_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 40, 0, NULL, 0 },
	{ SENSOR_NUM_FB_12_HEX_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 40, 0, NULL, 0 },
	{ SENSOR_NUM_FB_13_HEX_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 40, 0, NULL, 0 },
	{ SENSOR_NUM_FB_14_HEX_INLET_TEMP_C, THRESHOLD_ENABLE_LCR, 40, 0, NULL, 0 },
	//	{ SENSOR_NUM_PB_1_HDC1080DMBR_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_2_HDC1080DMBR_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_3_HDC1080DMBR_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_1_HSC_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_2_HSC_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_3_HSC_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_4_HSC_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_5_HSC_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_6_HSC_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_7_HSC_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_8_HSC_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_9_HSC_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_10_HSC_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_11_HSC_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_12_HSC_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_13_HSC_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_14_HSC_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_1_HSC_P48V_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_2_HSC_P48V_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_3_HSC_P48V_TEMP_C, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_1_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_2_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_3_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_4_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_5_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_6_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_7_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_8_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_9_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_10_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_11_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_12_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_13_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_14_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_1_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_2_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_3_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_BB_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_BPB_HSC_P48V_VIN_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_BPB_CDU_COOLANT_LEAKAGE_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_BPB_RACK_COOLANT_LEAKAGE_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_SB_TTV_COOLANT_LEAKAGE_1_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_SB_TTV_COOLANT_LEAKAGE_2_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_SB_TTV_COOLANT_LEAKAGE_3_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PDB_48V_SENSE_DIFF_POS_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PDB_48V_SENSE_DIFF_NEG_VOLT_V, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_1_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_2_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_3_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_4_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_5_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_6_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_7_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_8_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_9_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_10_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_11_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_12_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_13_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_14_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_BB_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_BPB_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_1_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_2_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_3_HSC_P48V_IOUT_CURR_A, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_1_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_2_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_3_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_4_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_5_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_6_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_7_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_8_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_9_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_10_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_11_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_12_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_13_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_14_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_BB_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_BPB_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_1_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_2_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_3_HSC_P48V_PIN_PWR_W, None, None, None, NULL, 0 },
	{ SENSOR_NUM_FB_1_FAN_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_FB_2_FAN_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_FB_3_FAN_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_FB_4_FAN_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_FB_5_FAN_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_FB_6_FAN_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_FB_7_FAN_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_FB_8_FAN_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_FB_9_FAN_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_FB_10_FAN_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_FB_11_FAN_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_FB_12_FAN_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_FB_13_FAN_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_FB_14_FAN_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	//	{ SENSOR_NUM_PB_1_PUMP_TACH_RPM, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_2_PUMP_TACH_RPM, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_3_PUMP_TACH_RPM, None, None, None, NULL, 0 },
	{ SENSOR_NUM_PB_1_FAN_1_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_PB_1_FAN_2_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_PB_2_FAN_1_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_PB_2_FAN_2_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_PB_3_FAN_1_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_PB_3_FAN_2_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_MB_FAN1_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	{ SENSOR_NUM_MB_FAN2_TACH_RPM, THRESHOLD_ENABLE_UCR, 0, 500, NULL, 0 },
	//	{ SENSOR_NUM_BPB_RPU_COOLANT_INLET_P_KPA, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA, None, None, None, NULL, 0 },
	{ SENSOR_NUM_BPB_RACK_PRESSURE_3_P_KPA, THRESHOLD_ENABLE_LCR, 200, 0, NULL, 0 },
	{ SENSOR_NUM_BPB_RACK_PRESSURE_4_P_KPA, THRESHOLD_ENABLE_LCR, 200, 0, NULL, 0 },
	{ SENSOR_NUM_SB_HEX_PRESSURE_1_P_KPA, THRESHOLD_ENABLE_LCR, 200, 0, NULL, 0 },
	{ SENSOR_NUM_SB_HEX_PRESSURE_2_P_KPA, THRESHOLD_ENABLE_LCR, 200, 0, NULL, 0 },
	{ SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM, THRESHOLD_ENABLE_UCR, 0, 10, NULL, 0 },
	{ SENSOR_NUM_BPB_RACK_LEVEL_1, THRESHOLD_ENABLE_LCR, 18000, 0, NULL, 0 },
	{ SENSOR_NUM_BPB_RACK_LEVEL_2, THRESHOLD_ENABLE_LCR, 18000, 0, NULL, 0 },
	//	{ SENSOR_NUM_MB_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PDB_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_1_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_2_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_PB_3_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_1_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_2_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_3_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_4_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_5_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_6_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_7_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_8_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_9_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_10_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_11_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_12_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_13_HUM_PCT_RH, None, None, None, NULL, 0 },
	//	{ SENSOR_NUM_FB_14_HUM_PCT_RH, None, None, None, NULL, 0 },
};

void set_threshold_poll_enable_flag(bool flag)
{
	threshold_poll_enable_flag = flag;
}

bool get_threshold_poll_enable_flag()
{
	return threshold_poll_enable_flag;
}

/*
	check whether the status has changed
	If type is LCR, and UCR occurs, the status will still be normal.
 */
static bool set_threshold_status(sensor_threshold *threshold_tbl, float val)
{
	uint8_t status = THRESHOLD_STATUS_NORMAL;
	switch (threshold_tbl->type) {
	case THRESHOLD_ENABLE_LCR:
		if (val < threshold_tbl->lcr)
			status = THRESHOLD_STATUS_LCR;
		break;
	case THRESHOLD_ENABLE_UCR:
		if (val > threshold_tbl->ucr)
			status = THRESHOLD_STATUS_UCR;
		break;
	case THRESHOLD_ENABLE_BOTH:
		if (val < threshold_tbl->lcr)
			status = THRESHOLD_STATUS_LCR;
		else if (val > threshold_tbl->ucr)
			status = THRESHOLD_STATUS_UCR;
		break;
	default:
		LOG_ERR("set status error type %d", threshold_tbl->type);
		return false;
	}

	if (threshold_tbl->last_status == status)
		return false;

	threshold_tbl->last_status = status;
	return true;
}

void threshold_poll_handler(void *arug0, void *arug1, void *arug2)
{
	ARG_UNUSED(arug0);
	ARG_UNUSED(arug1);
	ARG_UNUSED(arug2);

	int threshold_poll_interval_ms = 1000; // interval 1s

	while (1) {
		if (!get_sensor_init_done_flag()) {
			k_msleep(threshold_poll_interval_ms);
			continue;
		}

		if (!get_sensor_poll_enable_flag()) {
			k_msleep(threshold_poll_interval_ms);
			continue;
		}

		if (!get_threshold_poll_enable_flag()) {
			k_msleep(threshold_poll_interval_ms);
			continue;
		}

		for (uint8_t i = 0; i < ARRAY_SIZE(threshold_tbl); i++) {
			float val = 0;
			if (get_sensor_reading_to_real_val(threshold_tbl[i].sensor_num, &val) !=
			    SENSOR_READ_SUCCESS)
				continue;

			/* check whether the status has changed */
			if (!set_threshold_status(&threshold_tbl[i], val))
				continue;

			if (threshold_tbl[i].fn)
				threshold_tbl[i].fn(threshold_tbl[i].arg,
						    threshold_tbl[i].last_status);
		}

		k_msleep(threshold_poll_interval_ms);
	}

	return;
}

void threshold_poll_init()
{
	k_thread_create(&threshold_poll, threshold_poll_stack,
			K_THREAD_STACK_SIZEOF(threshold_poll_stack), threshold_poll_handler, NULL,
			NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&threshold_poll, "threshold_poll");
	return;
}