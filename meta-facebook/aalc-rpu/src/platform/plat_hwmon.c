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
#include <stdint.h>
#include <string.h>
#include "plat_i2c.h"
#include "sensor.h"
#include <logging/log.h>
#include <sys/util.h>
#include "plat_sensor_table.h"
#include "plat_modbus.h"
#include "modbus_server.h"
#include "plat_hwmon.h"
#include "adm1272.h"
#include "plat_log.h"
#include "plat_hook.h"
#include "plat_pwm.h"
#include "plat_status.h"
#include "plat_isr.h"
#include "plat_fru.h"
#include "plat_fsc.h"

LOG_MODULE_REGISTER(plat_hwmon);

static uint32_t pump1_last_switch_time = 0;
static uint32_t pump2_last_switch_time = 0;
static uint32_t pump3_last_switch_time = 0;
static uint32_t pump1_current_boot_unrunning_time = 0;
static uint32_t pump2_current_boot_unrunning_time = 0;
static uint32_t pump3_current_boot_unrunning_time = 0;
static uint32_t last_auto_tune_flag = 0;

bool modbus_pump_setting_unsupport_function(pump_reset_struct *data, uint8_t bit_val)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);
	return true;
}

K_WORK_DEFINE(clear_log_work, modbus_clear_log);
bool clear_log_for_modbus_pump_setting(pump_reset_struct *data, uint8_t bit_val)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	if (bit_val == 0) // do nothing
		return true;

	k_work_submit(&clear_log_work);

	return true;
}

static bool hsc_reset(uint8_t sensor_num)
{
	// Check sensor information in sensor config table
	sensor_cfg *cfg = get_common_sensor_cfg_info(sensor_num);
	if (cfg == NULL) {
		LOG_ERR("Fail when getting pump sensor config, 0x%x", sensor_num);
		return false;
	}

	//uint8_t bus,uint8_t addr, bool enable_flag
	uint8_t bus = cfg->port;
	uint8_t addr = cfg->target_addr;
	// 1 enable, 0 disable, stop pump first
	if (enable_adm1272_hsc(bus, addr, false)) {
		// check pump is already enable
		k_msleep(500);
		// enable pump
		if (enable_adm1272_hsc(bus, addr, true)) {
			return true;
		} else {
			LOG_ERR("Fail when start the pump.");
			return false;
		}
	} else {
		LOG_ERR("Fail when stop the pump.");
		return false;
	}
}

#define PUMP_RESET_FUNCTIONS(num)                                                                  \
	static void pump##num##_reset()                                                            \
	{                                                                                          \
		hsc_reset(SENSOR_NUM_PB_##num##_HSC_P48V_PIN_PWR_W);                               \
	}                                                                                          \
	K_WORK_DEFINE(pump##num##_reset_work, pump##num##_reset);                                  \
	bool pump_setting_pump##num##_reset(pump_reset_struct *data, uint8_t bit_val)              \
	{                                                                                          \
		CHECK_NULL_ARG_WITH_RETURN(data, false);                                           \
		if (bit_val == 0)                                                                  \
			return true;                                                               \
		k_work_submit(&pump##num##_reset_work);                                            \
		return true;                                                                       \
	}

PUMP_RESET_FUNCTIONS(1)
PUMP_RESET_FUNCTIONS(2)
PUMP_RESET_FUNCTIONS(3)

bool close_pump(pump_reset_struct *data, uint8_t bit_val)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	set_status_flag(STATUS_FLAG_FAILURE, PUMP_FAIL_CLOSE_PUMP, bit_val);

	return true;
}

bool pump_setting_set_manual_flag(pump_reset_struct *data, uint8_t bit_val)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	/* error bit val, turn on auto tune at the same time */
	if (bit_val == 2)
		return false;

	if (data->function_index == MANUAL_CONTROL_PUMP)
		set_manual_pwm_flag(MANUAL_PWM_E_PUMP, bit_val);
	else if (data->function_index == MANUAL_CONTROL_FAN)
		set_manual_pwm_flag(MANUAL_PWM_E_HEX_FAN, bit_val);
	else
		LOG_ERR("unknow function_index for set manual flag");

	return true;
}

bool pump_setting_set_auto_tune_flag(pump_reset_struct *data, uint8_t bit_val)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	if (!get_status_flag(STATUS_FLAG_SYSTEM_READY))
		return false;

	if (bit_val) {
		for (uint8_t i = MANUAL_PWM_E_HEX_FAN; i <= MANUAL_PWM_E_RPU_FAN; i++) {
			if (get_manual_pwm_flag(i))
				return false;
		}
	}

	set_status_flag(STATUS_FLAG_AUTO_TUNE, 0, bit_val);
	if (bit_val != last_auto_tune_flag) {
		set_pump_uptime_secs(true, true, true);

		last_auto_tune_flag = bit_val;
	}

	return true;
}

bool pump_setting_set_pump_redundant(pump_reset_struct *data, uint8_t bit_val)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	pump_redundant_enable(bit_val);

	return true;
}

bool set_all_pump_power(bool switch_val)
{
	static uint8_t pump_sensor_nums[] = { SENSOR_NUM_PB_1_PUMP_TACH_RPM,
					      SENSOR_NUM_PB_2_PUMP_TACH_RPM,
					      SENSOR_NUM_PB_3_PUMP_TACH_RPM };
	bool val = true;
	for (uint8_t i = 0; i < ARRAY_SIZE(pump_sensor_nums); i++) {
		sensor_cfg *cfg = get_common_sensor_cfg_info(pump_sensor_nums[i]);
		if (!enable_adm1272_hsc(cfg->port, cfg->target_addr, switch_val)) {
			val = false;
			LOG_ERR("set pump power failed for sensor number %d", pump_sensor_nums[i]);
		}
	}

	return val;
}

uint8_t hsc_pwe_cycle_tbl[] = {
	//fan board (cycle)
	SENSOR_NUM_FB_1_HSC_P48V_VIN_VOLT_V, SENSOR_NUM_FB_2_HSC_P48V_VIN_VOLT_V,
	SENSOR_NUM_FB_3_HSC_P48V_VIN_VOLT_V, SENSOR_NUM_FB_4_HSC_P48V_VIN_VOLT_V,
	SENSOR_NUM_FB_5_HSC_P48V_VIN_VOLT_V, SENSOR_NUM_FB_6_HSC_P48V_VIN_VOLT_V,
	SENSOR_NUM_FB_7_HSC_P48V_VIN_VOLT_V, SENSOR_NUM_FB_8_HSC_P48V_VIN_VOLT_V,
	SENSOR_NUM_FB_9_HSC_P48V_VIN_VOLT_V, SENSOR_NUM_FB_10_HSC_P48V_VIN_VOLT_V,
	SENSOR_NUM_FB_11_HSC_P48V_VIN_VOLT_V, SENSOR_NUM_FB_12_HSC_P48V_VIN_VOLT_V,
	SENSOR_NUM_FB_13_HSC_P48V_VIN_VOLT_V, SENSOR_NUM_FB_14_HSC_P48V_VIN_VOLT_V,
	// pump board (cycle)
	SENSOR_NUM_PB_1_HSC_P48V_VIN_VOLT_V, SENSOR_NUM_PB_2_HSC_P48V_VIN_VOLT_V,
	SENSOR_NUM_PB_3_HSC_P48V_VIN_VOLT_V,
	//back plane board (cycle)
	SENSOR_NUM_BPB_HSC_P48V_VIN_VOLT_V,
	//bridge board (cycle)
	SENSOR_NUM_BB_HSC_P48V_VIN_VOLT_V

};

#define TURN_OFF_HSC_OUTPUT 0
#define TURN_ON_HSC_OUTPUT 1
#define FAN_BORAD_COUNT_NUM 14
#define PUMP_BORAD_COUNT_NUM 17
#define BACKPLANE_BORAD_COUNT_NUM 18
#define BRIDGE_BORAD_COUNT_NUM 19
void hsc_adm1272_pwr_ctrl(sensor_cfg *cfg, uint8_t adm1272_reg, uint8_t state)
{
	I2C_MSG msg = { 0 };
	uint8_t retry = 1;
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	if (!pre_PCA9546A_read(cfg, cfg->pre_sensor_read_args))
		LOG_ERR("pre lock mutex fail !");

	msg.data[0] = adm1272_reg;
	msg.data[1] = state;

	if (adm1272_reg == ADM1272_POWER_CYCLE_REG)
		msg.tx_len = 1;

	uint8_t ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Fail to access hsc adm1272 pwer ctrl, bus: 0x%x, addr: 0x%x, ret: %d",
			cfg->port, cfg->target_addr, ret);
	}

	if (!post_PCA9546A_read(cfg, cfg->pre_sensor_read_args, 0))
		LOG_ERR("pro unlock mutex fail !");
}
void hsc_xdp710_pwr_ctrl(sensor_cfg *cfg, uint8_t xdp710_reg, uint8_t state)
{
	I2C_MSG msg = { 0 };
	uint8_t retry = 1;
	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 2;
	if (!pre_PCA9546A_read(cfg, cfg->pre_sensor_read_args))
		LOG_ERR("pre lock mutex fail !");

	msg.data[0] = xdp710_reg;
	msg.data[1] = state;

	if (xdp710_reg == XDP710_RESTART_ADDR)
		msg.tx_len = 1;

	uint8_t ret = i2c_master_write(&msg, retry);
	if (ret != 0) {
		LOG_ERR("Fail to access hsc xdp710 pwer ctrl, bus: 0x%x, addr: 0x%x, ret: %d",
			cfg->port, cfg->target_addr, ret);
	}

	if (!post_PCA9546A_read(cfg, cfg->pre_sensor_read_args, 0))
		LOG_ERR("pro unlock mutex fail !");
}
void rpu_remote_power_cycle()
{
	// disable sensor poll
	disable_sensor_poll();

	if (!nct7363_wdt_all_disable()) {
		LOG_ERR("nct7363 wdt all disable fail when power cycle!");
	}

	//turn off and cycle rpu
	for (int i = 0; i < ARRAY_SIZE(hsc_pwe_cycle_tbl); i++) {
		sensor_cfg *cfg = get_common_sensor_cfg_info(hsc_pwe_cycle_tbl[i]);

		if (cfg == NULL) {
			LOG_ERR("Get_common_sensor_cfg_info fail 0x%x", hsc_pwe_cycle_tbl[i]);
			continue;
		}

		// cycle fan board and pump board
		if (i < BACKPLANE_BORAD_COUNT_NUM) {
			LOG_WRN("cycle fan board 0x%x", hsc_pwe_cycle_tbl[i]);
			if (cfg->type == sensor_dev_adm1272)
				hsc_adm1272_pwr_ctrl(cfg, ADM1272_POWER_CYCLE_REG, 0);
			else
				hsc_xdp710_pwr_ctrl(cfg, XDP710_RESTART_ADDR, 0);
		}

		// cycle backplane board
		if (i >= PUMP_BORAD_COUNT_NUM && i < BACKPLANE_BORAD_COUNT_NUM) {
			LOG_WRN("cycle backplane board 0x%x", hsc_pwe_cycle_tbl[i]);
			if (cfg->type == sensor_dev_adm1272)
				hsc_adm1272_pwr_ctrl(cfg, ADM1272_POWER_CYCLE_REG, 0);
			else
				hsc_xdp710_pwr_ctrl(cfg, XDP710_RESTART_ADDR, 0);
		}

		// cycle bridge board
		if (i >= BACKPLANE_BORAD_COUNT_NUM && i < BRIDGE_BORAD_COUNT_NUM) {
			k_msleep(1000);
			LOG_WRN("cycle bridge board 0x%x", hsc_pwe_cycle_tbl[i]);
			if (cfg->type == sensor_dev_adm1272)
				hsc_adm1272_pwr_ctrl(cfg, ADM1272_POWER_CYCLE_REG, 0);
			else
				hsc_xdp710_pwr_ctrl(cfg, XDP710_RESTART_ADDR, 0);
		}
	}
}

K_WORK_DEFINE(rpu_pwr_cycle_work, rpu_remote_power_cycle);
bool rpu_remote_power_cycle_function(pump_reset_struct *data, uint8_t bit_val)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	if (bit_val == 0) // do nothing
		return true;

	k_work_submit(&rpu_pwr_cycle_work);
	return true;
}

// pump redundant
void pump_redundant_handler(struct k_timer *timer)
{
	uint32_t current_state = get_status_flag(STATUS_FLAG_PUMP_REDUNDANT);

	current_state++;
	if (current_state == PUMP_REDUNDANT_MAX)
		current_state = PUMP_REDUNDANT_12;
	set_status_flag(STATUS_FLAG_PUMP_REDUNDANT, 0xFF, current_state);
}
void pump_redundant_handler_disable(struct k_timer *timer)
{
	set_status_flag(STATUS_FLAG_PUMP_REDUNDANT, 0xFF, PUMP_REDUNDANT_DISABLE);
}
K_TIMER_DEFINE(pump_redundant_timer, pump_redundant_handler, pump_redundant_handler_disable);

static uint8_t pump_redundant_switch_time = 7;
static uint8_t pump_redundant_switch_time_type = 0; /* for test, 0: day, 1: hours */
uint8_t get_pump_redundant_switch_time()
{
	return pump_redundant_switch_time;
}
void set_pump_redundant_switch_time(uint8_t time)
{
	pump_redundant_switch_time = time;
}
void set_pump_redundant_switch_time_type(uint8_t type)
{
	pump_redundant_switch_time_type = type;
}
void pump_redundant_enable(uint8_t onoff)
{
	if (onoff) {
		if (get_status_flag(STATUS_FLAG_PUMP_REDUNDANT) == PUMP_REDUNDANT_DISABLE)
			k_timer_start(&pump_redundant_timer, K_NO_WAIT,
				      K_HOURS(pump_redundant_switch_time *
					      (pump_redundant_switch_time_type ? 1 : 24)));
	} else {
		k_timer_stop(&pump_redundant_timer);
	}
}

// If a failure occurs, return true
static bool failure_behavior(uint8_t group)
{
	// all 0
	for (uint8_t i = PUMP_FAIL_EMERGENCY_BUTTON; i <= PUMP_FAIL_CLOSE_PUMP; i++) {
		if ((get_status_flag(STATUS_FLAG_FAILURE) >> i) & 0x01) {
			set_pwm_group(group, 0);
			return true;
		}
	}

	// all 100
	for (uint8_t i = PUMP_FAIL_TWO_HEX_FAN_FAILURE; i <= PUMP_FAIL_ABNORMAL_AIR_INLET_TEMP;
	     i++) {
		if ((get_status_flag(STATUS_FLAG_FAILURE) >> i) & 0x01) {
			set_pwm_group(group, 100);
			return true;
		}
	}

	// pump 100
	if (group == PWM_GROUP_E_PUMP) {
		if ((get_status_flag(STATUS_FLAG_FAILURE) >> PUMP_FAIL_FLOW_RATE_NOT_ACCESS) &
		    0x01) {
			set_pwm_group(group, 100);
			return true;
		}
	}

	// hex fan 100
	if (group == PWM_GROUP_E_HEX_FAN) {
		if ((get_status_flag(STATUS_FLAG_FAILURE) >>
		     HEX_FAN_FAIL_COOLANT_OUTLET_TEMP_NOT_ACCESS) &
		    0x01) {
			set_pwm_group(group, 100);
			return true;
		}
	}

	return false;
}

uint8_t pwm_control(uint8_t group, uint8_t duty)
{
	// stop pump
	if ((get_status_flag(STATUS_FLAG_FAILURE) >> PUMP_FAIL_CLOSE_PUMP) & 0x01) {
		set_pwm_group(group, 0);
		return 0;
	}

	// suppurt redundant device in semi mode
	uint32_t redundant_check = PUMP_REDUNDANT_DISABLE;
	if (get_fsc_mode() == FSC_MODE_SEMI_MODE)
		redundant_check = get_status_flag(STATUS_FLAG_PUMP_REDUNDANT);

	switch (group) {
	case PWM_GROUP_E_PUMP:
		if (get_manual_pwm_flag(MANUAL_PWM_E_PUMP)) {
			plat_pwm_ctrl(PWM_DEVICE_E_PB_PUMB_1,
				      (redundant_check == PUMP_REDUNDANT_23) ?
					      0 :
					      get_manual_pwm_cache(MANUAL_PWM_E_PUMP_1));
			plat_pwm_ctrl(PWM_DEVICE_E_PB_PUMB_2,
				      (redundant_check == PUMP_REDUNDANT_13) ?
					      0 :
					      get_manual_pwm_cache(MANUAL_PWM_E_PUMP_2));
			plat_pwm_ctrl(PWM_DEVICE_E_PB_PUMB_3,
				      (redundant_check == PUMP_REDUNDANT_12) ?
					      0 :
					      get_manual_pwm_cache(MANUAL_PWM_E_PUMP_3));
			return 0;
		}
		break;
	case PWM_GROUP_E_HEX_FAN:
		if (get_manual_pwm_flag(MANUAL_PWM_E_HEX_FAN)) {
			set_pwm_group(group, get_manual_pwm_cache(MANUAL_PWM_E_HEX_FAN));
			return 0;
		}
		break;
	case PWM_GROUP_E_RPU_FAN:
		if (get_manual_pwm_flag(MANUAL_PWM_E_RPU_FAN)) {
			plat_pwm_ctrl(PWM_DEVICE_E_PB_PUMB_FAN_1,
				      get_manual_pwm_cache(MANUAL_PWM_E_PUMP_FAN_1));
			plat_pwm_ctrl(PWM_DEVICE_E_PB_PUMB_FAN_2,
				      get_manual_pwm_cache(MANUAL_PWM_E_PUMP_FAN_2));
			plat_pwm_ctrl(PWM_DEVICE_E_PB_PUMB_FAN_3,
				      get_manual_pwm_cache(MANUAL_PWM_E_PUMP_FAN_3));
			plat_pwm_ctrl(PWM_DEVICE_E_BB_FAN,
				      get_manual_pwm_cache(MANUAL_PWM_E_RPU_PCB_FAN));
			return 0;
		}
		break;
	default:
		LOG_ERR("unknow pwm_control group %d", group);
		break;
	}

	static int64_t auto_tune_time;
	if (!get_status_flag(STATUS_FLAG_AUTO_TUNE)) {
		auto_tune_time = k_uptime_get();
		return 0;
	}

	// keep 0 duty when first enabling auto tune
	static bool is_init = false;
	if (!is_init) {
		is_init = true;
		return 0;
	}

	// 10s flow rate ready + 1s polling error
	if ((k_uptime_get() - auto_tune_time) > 11000) {
		if (failure_behavior(group))
			return 0;
	}

	if (!set_pwm_group(group, duty))
		return 0;

	return 1;
}

bool get_pump_last_switch_time(uint8_t pump_num, uint32_t *return_uptime)
{
	switch (pump_num) {
	case PUMP_1_UPTIME:
		*return_uptime = pump1_last_switch_time;

		break;
	case PUMP_2_UPTIME:
		*return_uptime = pump2_last_switch_time;

		break;
	case PUMP_3_UPTIME:
		*return_uptime = pump3_last_switch_time;

		break;
	default:
		LOG_ERR("unknow pump_num %d when get pump last switch time", pump_num);
		return false;
	}

	return true;
}

bool get_pump_current_boot_unrunning_time(uint8_t pump_num, uint32_t *return_uptime)
{
	switch (pump_num) {
	case PUMP_1_UPTIME:
		*return_uptime = pump1_current_boot_unrunning_time;

		break;
	case PUMP_2_UPTIME:
		*return_uptime = pump2_current_boot_unrunning_time;

		break;
	case PUMP_3_UPTIME:
		*return_uptime = pump3_current_boot_unrunning_time;

		break;
	default:
		LOG_ERR("unknow pump_num %d when get pump current boot unrunning time", pump_num);
		return false;
	}

	return true;
}

pump_running_time pump_running_time_info[] = {
	{ PUMP_1_UPTIME, EEPROM_PUMP1_UPTIME_SIZE, EEPROM_PUMP1_UPTIME_OFFSET },
	{ PUMP_2_UPTIME, EEPROM_PUMP2_UPTIME_SIZE, EEPROM_PUMP2_UPTIME_OFFSET },
	{ PUMP_3_UPTIME, EEPROM_PUMP3_UPTIME_SIZE, EEPROM_PUMP3_UPTIME_OFFSET },
};

bool read_pump_running_time_data_from_eeprom(uint8_t pump_num, uint32_t *return_data)
{
	if (pump_num >= PUMP_MAX_NUM) {
		LOG_ERR("unknow pump_num when read data from eeprom %d", pump_num);
		return false;
	}

	// read from eeprom
	uint8_t tmp_read_data[4] = { 0 }; // 4 bytes
	uint16_t offset = 0;

	offset = pump_running_time_info[pump_num].eeprom_offset;

	if (!plat_eeprom_read(offset, tmp_read_data, pump_running_time_info->size)) {
		LOG_ERR("read %d uptime fail!, ", pump_num);
		return false;
	}

	*return_data = (tmp_read_data[0] << 24) | (tmp_read_data[1] << 16) |
		       (tmp_read_data[2] << 8) | tmp_read_data[3];

	return true;
}

bool get_pump_uptime_secs(uint8_t pump_num, uint32_t *return_uptime)
{
	CHECK_NULL_ARG_WITH_RETURN(return_uptime, false);

	if (pump_num > PUMP_MAX_NUM) {
		LOG_ERR("unknow pump_num when read pump uptime secs %d", pump_num);
		return false;
	}

	// get pump running time from eeprom
	uint32_t tmp_uptime = 0;
	if (!read_pump_running_time_data_from_eeprom(pump_num, &tmp_uptime))
		return false;

	uint32_t tmp_last_switch_time = 0;
	if (!get_pump_last_switch_time(pump_num, &tmp_last_switch_time))
		return false;

	uint32_t tmp_current_boot_unrunning_time = 0;
	if (!get_pump_current_boot_unrunning_time(pump_num, &tmp_current_boot_unrunning_time))
		return false;

	// if auto tune is on
	if (get_status_flag(STATUS_FLAG_AUTO_TUNE))
		*return_uptime =
			tmp_uptime + (k_uptime_get_32() / 1000) - tmp_current_boot_unrunning_time;
	else
		*return_uptime =
			tmp_uptime + (tmp_last_switch_time - tmp_current_boot_unrunning_time);
	return true;
}

bool set_pump_uptime_secs(uint8_t pump_1_set, uint8_t pump_2_set, uint8_t pump_3_set)
{
	bool auto_tune_flag = get_status_flag(STATUS_FLAG_AUTO_TUNE);

	if (auto_tune_flag == last_auto_tune_flag)
		return true;

	if (pump_1_set) {
		if (auto_tune_flag)
			pump1_current_boot_unrunning_time +=
				(k_uptime_get_32() / 1000 - pump1_last_switch_time);

		pump1_last_switch_time = k_uptime_get_32() / 1000;
	}

	if (pump_2_set) {
		if (auto_tune_flag)
			pump2_current_boot_unrunning_time +=
				(k_uptime_get_32() / 1000 - pump2_last_switch_time);

		pump2_last_switch_time = k_uptime_get_32() / 1000;
	}

	if (pump_3_set) {
		if (auto_tune_flag)
			pump3_current_boot_unrunning_time +=
				(k_uptime_get_32() / 1000 - pump3_last_switch_time);

		pump3_last_switch_time = k_uptime_get_32() / 1000;
	}

	last_auto_tune_flag = auto_tune_flag;

	return true;
}

bool modbus_clear_pump_running_time_function(pump_reset_struct *data, uint8_t bit_val)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);
	if (bit_val == 0) // do nothing
		return true;

	// clear eeprom
	uint8_t total_size =
		EEPROM_PUMP1_UPTIME_SIZE + EEPROM_PUMP2_UPTIME_SIZE + EEPROM_PUMP3_UPTIME_SIZE;
	uint8_t clear_data[12] = { 0 };

	if (!plat_eeprom_write(EEPROM_PUMP1_UPTIME_OFFSET, clear_data, total_size)) {
		LOG_ERR("clear pump running time fail!");
		return false;
	}

	pump1_last_switch_time = k_uptime_get_32() / 1000;
	pump2_last_switch_time = k_uptime_get_32() / 1000;
	pump3_last_switch_time = k_uptime_get_32() / 1000;
	pump1_current_boot_unrunning_time = k_uptime_get_32() / 1000;
	pump2_current_boot_unrunning_time = k_uptime_get_32() / 1000;
	pump3_current_boot_unrunning_time = k_uptime_get_32() / 1000;

	return true;
}