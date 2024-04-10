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
#include <stdlib.h>
#include <sys/util.h>
#include <drivers/gpio.h>
#include <modbus/modbus.h>
#include <time.h>
#include <logging/log.h>
#include "sensor.h"
#include "modbus_server.h"
#include "fru.h"
#include "eeprom.h"
#include "libutil.h"

#include "plat_modbus.h"
#include "plat_sensor_table.h"
#include "plat_fru.h"

LOG_MODULE_REGISTER(plat_modbus);

static char server_iface_name[] = "MODBUS0";

struct k_thread modbus_server_thread;
K_KERNEL_STACK_MEMBER(modbus_server_thread_stack, MODBUS_SERVER_THREAD_SIZE);

static float pow_of_10(int8_t exp)
{
	float ret = 1.0;
	int i;

	if (exp < 0) {
		for (i = 0; i > exp; i--) {
			ret /= 10.0;
		}
	} else if (exp > 0) {
		for (i = 0; i < exp; i++) {
			ret *= 10.0;
		}
	}

	return ret;
}

/*
	arg0: sensor number
	arg1: m
	arg2: r

	actual_val =  raw_val * m * (10 ^ r)
*/
static uint8_t modbus_get_senser_reading(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_ARG_NULL);

	int reading = 0;
	uint8_t status = get_sensor_reading(sensor_config, sensor_config_count, cmd->arg0, &reading,
					    GET_FROM_CACHE);

	if (status == SENSOR_READ_SUCCESS) {
		sensor_val *sval = (sensor_val *)&reading;
		float val = (sval->integer * 1000 + sval->fraction) / 1000;
		float r = pow_of_10(cmd->arg2);
		uint16_t byte_val = val / cmd->arg1 / r; // scale
		memcpy(&cmd->data, &byte_val, sizeof(uint16_t) * cmd->size);
		return MODBUS_READ_WRITE_REGISTER_SUCCESS;
	}

	return MODBUS_READ_WRITE_REGISTER_FAIL;
}

modbus_command_mapping modbus_command_table[] = {
	// addr, write_fn, read_fn, arg0, arg1, arg2, size
	{ MODBUS_BPB_RPU_COOLANT_FLOW_RATE_LPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RPU_COOLANT_FLOW_RATE_LPM, 1, 0, 1 },
	{ MODBUS_BPB_RPU_COOLANT_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_BPB_RPU_COOLANT_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RPU_COOLANT_INLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_BPB_RPU_COOLANT_OUTLET_P_KPA_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RPU_COOLANT_OUTLET_P_KPA, 1, 0, 1 },
	{ MODBUS_BPB_RPU_COOLANT_INLET_P_KPA_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RPU_COOLANT_INLET_P_KPA, 1, 0, 1 },
	{ MODBUS_RPU_PWR_W_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_AALC_TOTAL_PWR_W_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_RPU_INPUT_VOLT_V_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_MB_RPU_AIR_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_MB_RPU_AIR_INLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_RPU_PUMP_PWM_TACH_PCT_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_PB_1_PUMP_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_1_PUMP_TACH_RPM, 1, 0, 1 },
	{ MODBUS_PB_2_PUMP_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_2_PUMP_TACH_RPM, 1, 0, 1 },
	{ MODBUS_PB_3_PUMP_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_3_PUMP_TACH_RPM, 1, 0, 1 },
	{ MODBUS_RPU_FAN1_STATUS_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_RPU_FAN2_STATUS_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_MB_FAN1_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_MB_FAN1_TACH_RPM, 1, 0, 1 },
	{ MODBUS_MB_FAN2_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_MB_FAN2_TACH_RPM, 1, 0, 1 },
	{ MODBUS_AALC_COOLING_CAPACITY_W_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_RPU_PUMP1_STATUS_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_RPU_PUMP2_STATUS_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_RPU_PUMP3_STATUS_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_RPU_RESERVOIR_STATUS_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_RPU_LED_RESERVOIR_STATUS_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_RPU_LED_LEAKAGE_STATUS_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_RPU_LED_FAULT_STATUS_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_RPU_LED_POWER_STATUS_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_BB_TMP75_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_BB_TMP75_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_BPB_RPU_OUTLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RPU_OUTLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_PDB_HDC1080DMBR_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PDB_HDC1080DMBR_TEMP_C, 1, 0, 1 },
	{ MODBUS_BB_HSC_P48V_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BB_HSC_P48V_TEMP_C, 1, 0, 1 },
	{ MODBUS_BPB_HSC_P48V_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_HSC_P48V_TEMP_C, 1, 0, 1 },
	{ MODBUS_PB_1_HDC1080DMBR_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_1_HDC1080DMBR_TEMP_C, 1, 0, 1 },
	{ MODBUS_PB_2_HDC1080DMBR_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_2_HDC1080DMBR_TEMP_C, 1, 0, 1 },
	{ MODBUS_PB_3_HDC1080DMBR_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_3_HDC1080DMBR_TEMP_C, 1, 0, 1 },
	{ MODBUS_PB_1_HSC_P48V_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_1_HSC_P48V_TEMP_C, 1, 0, 1 },
	{ MODBUS_PB_2_HSC_P48V_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_2_HSC_P48V_TEMP_C, 1, 0, 1 },
	{ MODBUS_PB_3_HSC_P48V_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_3_HSC_P48V_TEMP_C, 1, 0, 1 },
	{ MODBUS_PB_1_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_1_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_PB_2_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_2_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_PB_3_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_3_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_BB_HSC_P51V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BB_HSC_P51V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_BPB_HSC_P51V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_HSC_P51V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_BB_HSC_P51V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BB_HSC_P51V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_BPB_HSC_P51V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_HSC_P51V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_PB_1_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_1_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_PB_2_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_2_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_PB_3_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_3_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_BB_HSC_P51V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BB_HSC_P51V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_BPB_HSC_P51V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_HSC_P51V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_PB_1_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_1_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_PUMP_1_RUNNING_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_PUMP_2_RUNNING_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_PUMP_3_RUNNING_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_PB_2_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_2_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_PB_3_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_3_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_PB_1_FAN_1_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_1_FAN_1_TACH_RPM, 1, 0, 1 },
	{ MODBUS_PB_1_FAN_2_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_1_FAN_2_TACH_RPM, 1, 0, 1 },
	{ MODBUS_PB_2_FAN_1_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_2_FAN_1_TACH_RPM, 1, 0, 1 },
	{ MODBUS_PB_2_FAN_2_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_2_FAN_2_TACH_RPM, 1, 0, 1 },
	{ MODBUS_PB_3_FAN_1_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_3_FAN_1_TACH_RPM, 1, 0, 1 },
	{ MODBUS_PB_3_FAN_2_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_PB_3_FAN_2_TACH_RPM, 1, 0, 1 },
	{ MODBUS_BPB_RACK_PRESSURE_3_P_KPA_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RACK_PRESSURE_3_P_KPA, 1, 0, 1 },
	{ MODBUS_BPB_RACK_PRESSURE_4_P_KPA_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RACK_PRESSURE_4_P_KPA, 1, 0, 1 },
	{ MODBUS_BPB_RACK_LEVEL_1_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RACK_LEVEL_1, 1, 0, 1 },
	{ MODBUS_BPB_RACK_LEVEL_2_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RACK_LEVEL_2, 1, 0, 1 },
	{ MODBUS_BPB_CDU_LEVEL_3_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_BPB_CDU_LEVEL_3,
	  1, 0, 1 },
	{ MODBUS_MB_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_MB_HUM_PCT_RH, 1,
	  0, 1 },
	{ MODBUS_PDB_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_PDB_HUM_PCT_RH, 1,
	  0, 1 },
	{ MODBUS_PB_1_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_PB_1_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_PB_2_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_PB_2_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_PB_3_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_PB_3_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_HEX_FAN_PWM_TACH_PCT_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_HEX_PWR_W_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_HEX_INPUT_VOLT_V_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_HEX_INPUT_CURRENT_V_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_FB_1_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_1_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_2_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_2_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_3_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_3_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_4_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_4_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_5_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_5_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_6_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_6_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_7_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_7_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_8_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_8_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_9_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_9_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_10_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_10_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_SB_HEX_AIR_OUTLET_1_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_SB_HEX_AIR_OUTLET_1_TEMP_C, 1, 0, 1 },
	{ MODBUS_SB_HEX_AIR_OUTLET_2_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_SB_HEX_AIR_OUTLET_2_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_1_HEX_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_1_HEX_INLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_2_HEX_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_2_HEX_INLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_HEX_WATER_INLET_TEMP_C_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_HEX_BLADDER_LEVEL_STATUS_ADDR, NULL, modbus_get_senser_reading, 0, 1, 0, 1 },
	{ MODBUS_SB_HEX_AIR_OUTLET_3_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_SB_HEX_AIR_OUTLET_3_TEMP_C, 1, 0, 1 },
	{ MODBUS_SB_HEX_AIR_OUTLET_4_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_SB_HEX_AIR_OUTLET_4_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_3_HEX_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_3_HEX_INLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_4_HEX_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_4_HEX_INLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_5_HEX_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_5_HEX_INLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_6_HEX_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_6_HEX_INLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_7_HEX_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_7_HEX_INLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_8_HEX_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_8_HEX_INLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_9_HEX_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_9_HEX_INLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_10_HEX_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_10_HEX_INLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_11_HEX_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_11_HEX_INLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_12_HEX_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_12_HEX_INLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_13_HEX_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_13_HEX_INLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_14_HEX_INLET_TEMP_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_14_HEX_INLET_TEMP_C, 1, 0, 1 },
	{ MODBUS_FB_1_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_1_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_2_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_2_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_3_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_3_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_4_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_4_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_5_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_5_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_6_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_6_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_7_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_7_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_8_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_8_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_9_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_9_HSC_TEMP_C, 1,
	  0, 1 },
	{ MODBUS_FB_10_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_10_HSC_TEMP_C,
	  1, 0, 1 },
	{ MODBUS_FB_11_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_11_HSC_TEMP_C,
	  1, 0, 1 },
	{ MODBUS_FB_12_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_12_HSC_TEMP_C,
	  1, 0, 1 },
	{ MODBUS_FB_13_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_13_HSC_TEMP_C,
	  1, 0, 1 },
	{ MODBUS_FB_14_HSC_TEMP_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_14_HSC_TEMP_C,
	  1, 0, 1 },
	{ MODBUS_FB_1_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_1_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_FB_2_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_2_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_FB_3_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_3_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_FB_4_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_4_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_FB_5_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_5_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_FB_6_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_6_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_FB_7_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_7_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_FB_8_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_8_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_FB_9_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_9_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_FB_10_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_10_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_FB_11_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_11_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_FB_12_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_12_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_FB_13_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_13_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_FB_14_HSC_P48V_VIN_VOLT_V_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_14_HSC_P48V_VIN_VOLT_V, 1, 0, 1 },
	{ MODBUS_FB_1_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_1_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_FB_2_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_2_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_FB_3_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_3_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_FB_4_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_4_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_FB_5_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_5_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_FB_6_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_6_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_FB_7_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_7_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_FB_8_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_8_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_FB_9_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_9_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_FB_10_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_10_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_FB_11_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_11_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_FB_12_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_12_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_FB_13_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_13_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_FB_14_HSC_P48V_IOUT_CURR_A_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_14_HSC_P48V_IOUT_CURR_A, 1, 0, 1 },
	{ MODBUS_FB_1_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_1_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_FB_2_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_2_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_FB_3_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_3_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_FB_4_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_4_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_FB_5_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_5_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_FB_6_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_6_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_FB_7_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_7_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_FB_8_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_8_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_FB_9_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_9_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_FB_10_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_10_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_FB_11_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_11_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_FB_12_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_12_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_FB_13_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_13_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_FB_14_HSC_P48V_PIN_PWR_W_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_14_HSC_P48V_PIN_PWR_W, 1, 0, 1 },
	{ MODBUS_FB_11_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_11_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_12_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_12_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_13_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_13_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_FB_14_FAN_TACH_RPM_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_14_FAN_TACH_RPM, 1, 0, 1 },
	{ MODBUS_SB_HEX_PRESSURE_1_P_KPA_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_SB_HEX_PRESSURE_1_P_KPA, 1, 0, 1 },
	{ MODBUS_SB_HEX_PRESSURE_2_P_KPA_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_SB_HEX_PRESSURE_2_P_KPA, 1, 0, 1 },
	{ MODBUS_FB_1_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_1_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_2_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_2_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_3_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_3_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_4_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_4_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_5_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_5_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_6_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_6_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_7_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_7_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_8_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_8_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_9_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_FB_9_HUM_PCT_RH,
	  1, 0, 1 },
	{ MODBUS_FB_10_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_10_HUM_PCT_RH, 1, 0, 1 },
	{ MODBUS_FB_11_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_11_HUM_PCT_RH, 1, 0, 1 },
	{ MODBUS_FB_12_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_12_HUM_PCT_RH, 1, 0, 1 },
	{ MODBUS_FB_13_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_13_HUM_PCT_RH, 1, 0, 1 },
	{ MODBUS_FB_14_HUM_PCT_RH_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_FB_14_HUM_PCT_RH, 1, 0, 1 },
	{ MODBUS_LEAK_RPU_INT_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_CDU_COOLANT_LEAKAGE_1, 1, 0, 1 },
	{ MODBUS_LEAK_RACK_FLOOR_GPO_AND_RELAY_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RACK_COOLANT_LEAKAGE_2, 1, 0, 1 },
};

static modbus_command_mapping *ptr_to_modbus_table(uint16_t addr)
{
	for (uint16_t i = 0; i < ARRAY_SIZE(modbus_command_table); i++) {
		if (addr >= modbus_command_table[i].addr &&
		    addr < (modbus_command_table[i].addr + modbus_command_table[i].size))
			return &modbus_command_table[i];
	}

	return NULL;
}

static void free_modbus_command_table_memory(void)
{
	for (uint16_t i = 0; i < ARRAY_SIZE(modbus_command_table); i++)
		if (modbus_command_table[i].data)
			SAFE_FREE(modbus_command_table[i].data);
}

void init_modbus_command_table(void)
{
	for (uint16_t i = 0; i < ARRAY_SIZE(modbus_command_table); i++) {
		if (modbus_command_table[i].data)
			SAFE_FREE(modbus_command_table[i].data);

		modbus_command_table[i].data =
			(uint16_t *)malloc(modbus_command_table[i].size * sizeof(uint16_t));
		if (modbus_command_table[i].data == NULL) {
			LOG_ERR("modbus_command_mapping[%i] malloc fail", i);
			goto init_fail;
		}
	}

	return;

init_fail:
	free_modbus_command_table_memory();
}

static int holding_reg_wr(uint16_t addr, uint16_t reg)
{
	modbus_command_mapping *ptr = ptr_to_modbus_table(addr);
	if (!ptr) {
		LOG_ERR("modbus write command 0x%x not find!\n", addr);
		return MODBUS_ADDR_NOT_DEFINITION;
	}

	if (!ptr->wr_fn) {
		LOG_ERR("modbus write function 0x%x not set!\n", addr);
		return MODBUS_ARG_NULL;
	}

	int ret = MODBUS_READ_WRITE_REGISTER_SUCCESS;
	uint8_t offset = addr - ptr->addr;

	ptr->data[offset] = reg;

	if (offset == (ptr->size - 1))
		ret = ptr->wr_fn(ptr);

	return ret;
}

static int holding_reg_rd(uint16_t addr, uint16_t *reg)
{
	CHECK_NULL_ARG_WITH_RETURN(reg, MODBUS_ARG_NULL);

	modbus_command_mapping *ptr = ptr_to_modbus_table(addr);
	if (!ptr) {
		LOG_ERR("modbus read command 0x%x not find!\n", addr);
		return MODBUS_ADDR_NOT_DEFINITION;
	}

	if (!ptr->rd_fn) {
		LOG_ERR("modbus read function 0x%x not set!\n", addr);
		return MODBUS_ARG_NULL;
	}

	int ret = MODBUS_READ_WRITE_REGISTER_SUCCESS;
	uint8_t offset = addr - ptr->addr;

	if (offset == 0)
		ret = ptr->rd_fn(ptr);

	*reg = ptr->data[offset];
	return ret;
}

static struct modbus_user_callbacks mbs_cbs = {
	.holding_reg_rd = holding_reg_rd,
	.holding_reg_wr = holding_reg_wr,
};

const static struct modbus_iface_param server_param = {
	.mode = MODBUS_MODE_RTU,
	.server = {
		.user_cb = &mbs_cbs,
		.unit_id = MODBUS_UART_NODE_ADDR,
	},
	.serial = {
		.baud = MODBUS_UART_BAUDRATE_LOW,
		.parity = MODBUS_UART_PARITY,
	},
};

static void modbus_server_handler(void *arug0, void *arug1, void *arug2)
{
	int val;

	val = init_modbus_server(*server_iface_name, server_param);
	if (val != 0) {
		LOG_ERR("Failed to initialize server");
		return;
	}
}

//void sensor_poll_init()
void modbus_server_handler_init(void)
{
	k_thread_create(&modbus_server_thread, modbus_server_thread_stack,
			K_THREAD_STACK_SIZEOF(modbus_server_thread_stack), modbus_server_handler,
			NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&modbus_server_thread, "modbus_server_handler");
	return;
}
