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

#include <drivers/pwm.h>
#include <logging/log.h>
#include "sensor.h"
#include "nct7363.h"
#include "plat_sensor_table.h"
#include "plat_hook.h"
#include "plat_fsc.h"
#include "plat_pwm.h"
#include "nct7363.h"
#include "plat_status.h"
#include "plat_threshold.h"

LOG_MODULE_REGISTER(plat_pwm);

#define MAX_FAN_DUTY_VALUE 100
#define PWM_PERIOD 40 // 25kHz

static const struct device *pwm_dev;
static uint8_t fan_group_duty_cache[PWM_GROUP_E_MAX];
static uint8_t fan_duty_cache[PWM_DEVICE_E_MAX];
static uint8_t manual_pwm_flag[MANUAL_PWM_E_MAX];
static uint8_t manual_pwm_cache[MANUAL_PWM_E_MAX];
static uint8_t redundant_dev_now = PWM_DEVICE_E_MAX;
static uint8_t redundant_dev_pre = PWM_DEVICE_E_MAX;
static enum REDUNDANCY_TRANSFORM_E redundant_phase = REDUNDANCY_TRANSFORM_DISABLE;
static uint8_t redundant_step1_count = REDUNDANT_STEP1_RETRY;
static uint8_t redundant_step2a_count = REDUNDANT_STEP2A_RETRY;
static uint8_t redundant_step2b_count = REDUNDANT_STEP2B_RETRY;
static bool is_redundant_transforming = false;

struct nct_dev_info {
	enum PWM_DEVICE_E dev;
	uint8_t tach_sen_num; /* for mapping the sensor config to get hardware information, like i2c bus, addr, mux info ... */
	uint8_t pwm_port; /* pwm pin index of nct */
} nct_dev_tbl[] = { { PWM_DEVICE_E_FB_FAN_1, SENSOR_NUM_FB_1_FAN_TACH_RPM, NCT7363_17_PORT },
		    { PWM_DEVICE_E_FB_FAN_2, SENSOR_NUM_FB_2_FAN_TACH_RPM, NCT7363_17_PORT },
		    { PWM_DEVICE_E_FB_FAN_3, SENSOR_NUM_FB_3_FAN_TACH_RPM, NCT7363_17_PORT },
		    { PWM_DEVICE_E_FB_FAN_4, SENSOR_NUM_FB_4_FAN_TACH_RPM, NCT7363_17_PORT },
		    { PWM_DEVICE_E_FB_FAN_5, SENSOR_NUM_FB_5_FAN_TACH_RPM, NCT7363_17_PORT },
		    { PWM_DEVICE_E_FB_FAN_6, SENSOR_NUM_FB_6_FAN_TACH_RPM, NCT7363_17_PORT },
		    { PWM_DEVICE_E_FB_FAN_7, SENSOR_NUM_FB_7_FAN_TACH_RPM, NCT7363_17_PORT },
		    { PWM_DEVICE_E_FB_FAN_8, SENSOR_NUM_FB_8_FAN_TACH_RPM, NCT7363_17_PORT },
		    { PWM_DEVICE_E_FB_FAN_9, SENSOR_NUM_FB_9_FAN_TACH_RPM, NCT7363_17_PORT },
		    { PWM_DEVICE_E_FB_FAN_10, SENSOR_NUM_FB_10_FAN_TACH_RPM, NCT7363_17_PORT },
		    { PWM_DEVICE_E_FB_FAN_11, SENSOR_NUM_FB_11_FAN_TACH_RPM, NCT7363_17_PORT },
		    { PWM_DEVICE_E_FB_FAN_12, SENSOR_NUM_FB_12_FAN_TACH_RPM, NCT7363_17_PORT },
		    { PWM_DEVICE_E_FB_FAN_13, SENSOR_NUM_FB_13_FAN_TACH_RPM, NCT7363_17_PORT },
		    { PWM_DEVICE_E_FB_FAN_14, SENSOR_NUM_FB_14_FAN_TACH_RPM, NCT7363_17_PORT },
		    { PWM_DEVICE_E_PB_PUMB_1, SENSOR_NUM_PB_1_PUMP_TACH_RPM, NCT7363_1_PORT },
		    { PWM_DEVICE_E_PB_PUMB_2, SENSOR_NUM_PB_2_PUMP_TACH_RPM, NCT7363_1_PORT },
		    { PWM_DEVICE_E_PB_PUMB_3, SENSOR_NUM_PB_3_PUMP_TACH_RPM, NCT7363_1_PORT },
		    { PWM_DEVICE_E_PB_PUMB_FAN_1, SENSOR_NUM_PB_1_FAN_1_TACH_RPM, NCT7363_2_PORT },
		    { PWM_DEVICE_E_PB_PUMB_FAN_2, SENSOR_NUM_PB_2_FAN_1_TACH_RPM, NCT7363_2_PORT },
		    { PWM_DEVICE_E_PB_PUMB_FAN_3, SENSOR_NUM_PB_3_FAN_1_TACH_RPM,
		      NCT7363_2_PORT } };

static uint8_t nct_pwm_ctl(enum PWM_DEVICE_E dev, uint8_t duty)
{
	uint8_t i;
	for (i = 0; i < ARRAY_SIZE(nct_dev_tbl); i++) {
		if (nct_dev_tbl[i].dev == dev)
			break;
	}

	if (i == ARRAY_SIZE(nct_dev_tbl)) {
		LOG_ERR("Invalid PWM device %d", dev);
		return 1;
	}

	sensor_cfg *cfg = get_common_sensor_cfg_info(nct_dev_tbl[i].tach_sen_num);
	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config for PWM device %d", dev);
		return 1;
	}

	LOG_DBG("Mapping Sensor Number %x, nct7363 port %d", nct_dev_tbl[i].tach_sen_num,
		nct_dev_tbl[i].pwm_port);
	LOG_DBG("cfg port %x, target_addr %x, offset %x", cfg->port, cfg->target_addr, cfg->offset);
	mux_config *mux = (mux_config *)cfg->pre_sensor_read_args;
	if (mux)
		LOG_DBG("mux bus %x, target_addr %x, channel %x", mux->bus, mux->target_addr,
			mux->channel);

	if (cfg->pre_sensor_read_hook) {
		LOG_DBG("cfg->pre_sensor_read_hook %p", cfg->pre_sensor_read_hook);
		if (cfg->pre_sensor_read_hook(cfg, cfg->pre_sensor_read_args) == false) {
			LOG_ERR("Failed to do pre sensor read for PWM device %d", dev);
			return 1;
		}
	}

	const bool ret = nct7363_set_duty(cfg, duty, nct_dev_tbl[i].pwm_port);

	if (cfg->post_sensor_read_hook) {
		LOG_DBG("cfg->post_sensor_read_hook %p", cfg->post_sensor_read_hook);
		if (cfg->post_sensor_read_hook(cfg, cfg->post_sensor_read_args, NULL) == false) {
			LOG_ERR("Failed to do post sensor read for PWM device %d", dev);
			return 1;
		}
	}

	return (ret == true) ? 0 : 1;
}
uint8_t nct7363_wdt_all_disable()
{
	for (uint8_t i = 0; i < ARRAY_SIZE(nct_dev_tbl); i++) {
		sensor_cfg *cfg = get_common_sensor_cfg_info(nct_dev_tbl[i].tach_sen_num);

		if (cfg == NULL) {
			LOG_ERR("Failed to get sensor config for wdt disable 0x%x",
				nct_dev_tbl[i].tach_sen_num);
			continue;
		}

		if (!pre_PCA9546A_read(cfg, cfg->pre_sensor_read_args))
			LOG_ERR("pre lock mutex fail !");

		nct7363_setting_wdt(cfg, WDT_DISABLE);

		if (!post_PCA9546A_read(cfg, cfg->pre_sensor_read_args, 0))
			LOG_ERR("post unlock mutex fail !");
	}

	return true;
}

uint8_t nct7363_wdt_all_enable()
{
	for (uint8_t i = 0; i < ARRAY_SIZE(nct_dev_tbl); i++) {
		sensor_cfg *cfg = get_common_sensor_cfg_info(nct_dev_tbl[i].tach_sen_num);
		if (cfg == NULL) {
			LOG_ERR("Failed to get sensor config for wdt enable 0x%x",
				nct_dev_tbl[i].tach_sen_num);
			continue;
		}

		nct7363_init_arg *init_arg = (nct7363_init_arg *)cfg->init_args;
		if (init_arg == NULL) {
			LOG_ERR("Failed to get sensor init_arg for wdt enable 0x%x",
				nct_dev_tbl[i].tach_sen_num);
			continue;
		}

		if (!pre_PCA9546A_read(cfg, cfg->pre_sensor_read_args))
			LOG_ERR("pre lock mutex fail !");

		nct7363_setting_wdt(cfg, init_arg->wdt_cfg);

		if (!post_PCA9546A_read(cfg, cfg->pre_sensor_read_args, 0))
			LOG_ERR("post unlock mutex fail !");
	}

	return true;
}

int ast_pwm_set(int duty)
{
	LOG_DBG("Set AST PWM duty %d", duty);
	if (pwm_dev == NULL) {
		LOG_ERR("PWM dev not found!");
		return 1;
	}
	return pwm_pin_set_usec(pwm_dev, PWM_PORT0, PWM_PERIOD, (PWM_PERIOD * duty / 100), 0);
}

static void set_pump_threshold(enum PWM_DEVICE_E dev, uint8_t duty)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(nct_dev_tbl); i++) {
		if (nct_dev_tbl[i].dev == dev) {
			pump_change_threshold(nct_dev_tbl[i].tach_sen_num, duty);
			break;
		}
	}
}

uint8_t plat_pwm_ctrl(enum PWM_DEVICE_E dev, uint8_t duty)
{
	if (dev >= PWM_DEVICE_E_MAX) {
		LOG_ERR("Invalid PWM device %d", dev);
		return 1;
	}

	if (duty > MAX_FAN_DUTY_VALUE) {
		LOG_ERR("Invalid PWM duty %d", duty);
		return 1;
	}

	LOG_DBG("Set PWM device %d duty %d", dev, duty);
	fan_duty_cache[dev] = duty;

	uint8_t ret = 0;
	switch (dev) {
	case PWM_DEVICE_E_BB_FAN:
		ret = ast_pwm_set(duty);
		break;

	case PWM_DEVICE_E_FB_FAN_1:
	case PWM_DEVICE_E_FB_FAN_2:
	case PWM_DEVICE_E_FB_FAN_3:
	case PWM_DEVICE_E_FB_FAN_4:
	case PWM_DEVICE_E_FB_FAN_5:
	case PWM_DEVICE_E_FB_FAN_6:
	case PWM_DEVICE_E_FB_FAN_7:
	case PWM_DEVICE_E_FB_FAN_8:
	case PWM_DEVICE_E_FB_FAN_9:
	case PWM_DEVICE_E_FB_FAN_10:
	case PWM_DEVICE_E_FB_FAN_11:
	case PWM_DEVICE_E_FB_FAN_12:
	case PWM_DEVICE_E_FB_FAN_13:
	case PWM_DEVICE_E_FB_FAN_14:
	case PWM_DEVICE_E_PB_PUMB_FAN_1:
	case PWM_DEVICE_E_PB_PUMB_FAN_2:
	case PWM_DEVICE_E_PB_PUMB_FAN_3:
		ret = nct_pwm_ctl(dev, duty);
		break;
	case PWM_DEVICE_E_PB_PUMB_1:
	case PWM_DEVICE_E_PB_PUMB_2:
	case PWM_DEVICE_E_PB_PUMB_3:
		ret = nct_pwm_ctl(dev, duty);
		set_pump_threshold(dev, duty);
		break;
	default:
		ret = 1;
		break;
	}

	return ret;
}

static uint8_t ctl_pwm_dev(uint8_t index_start, uint8_t index_end, uint8_t duty)
{
	if (duty > MAX_FAN_DUTY_VALUE) {
		LOG_ERR("Invalid PWM duty %d", duty);
		return 1;
	}

	if ((index_start > index_end) || (index_end > PWM_DEVICE_E_MAX)) {
		LOG_ERR("Invalid PWM Device index");
		return 1;
	}

	uint8_t ret = 0;
	uint32_t redundant_mode = get_status_flag(STATUS_FLAG_PUMP_REDUNDANT);
	uint8_t redundant_dev = (redundant_mode == PUMP_REDUNDANT_12) ? PWM_DEVICE_E_PB_PUMB_3 :
				(redundant_mode == PUMP_REDUNDANT_13) ? PWM_DEVICE_E_PB_PUMB_2 :
				(redundant_mode == PUMP_REDUNDANT_23) ? PWM_DEVICE_E_PB_PUMB_1 :
									PWM_DEVICE_E_MAX;

	for (uint8_t i = index_start; i <= index_end; i++) {
		if (i == redundant_dev) {
			// redundant duty 0
			if (plat_pwm_ctrl(i, 0)) {
				LOG_ERR("Failed to set PWM device %d redundant duty 0", i);
				ret = 1;
				//break;
			}
		} else {
			if (plat_pwm_ctrl(i, duty)) {
				LOG_ERR("Failed to set PWM device %d duty %d", i, duty);
				ret = 1;
				//break;
			}
		}
	}

	return ret;
}

void abnormal_pump_redundant_transform()
{
	bool exist_pump_work = true;
	for (uint8_t i = PUMP_FAIL_EMERGENCY_BUTTON; i <= PUMP_FAIL_CLOSE_PUMP; i++) {
		if ((get_status_flag(STATUS_FLAG_FAILURE) >> i) & 0x01) {
			exist_pump_work = false;
			break;
		}
	}

	uint32_t current_state = get_status_flag(STATUS_FLAG_PUMP_REDUNDANT);
	if (current_state != PUMP_REDUNDANT_DISABLE && exist_pump_work) {
		if (get_threshold_status(SENSOR_NUM_PB_1_PUMP_TACH_RPM))
			set_status_flag(STATUS_FLAG_PUMP_REDUNDANT, 0xFF, PUMP_REDUNDANT_23);

		if (get_threshold_status(SENSOR_NUM_PB_2_PUMP_TACH_RPM))
			set_status_flag(STATUS_FLAG_PUMP_REDUNDANT, 0xFF, PUMP_REDUNDANT_13);

		if (get_threshold_status(SENSOR_NUM_PB_3_PUMP_TACH_RPM))
			set_status_flag(STATUS_FLAG_PUMP_REDUNDANT, 0xFF, PUMP_REDUNDANT_12);
	}
}

bool get_is_redundant_transforming()
{
	return is_redundant_transforming;
}

void set_is_redundant_transforming(bool value)
{
	is_redundant_transforming = value;
}

uint8_t ctl_pwm_pump(uint8_t pump1_duty, uint8_t pump2_duty, uint8_t pump3_duty)
{
	if (pump1_duty > MAX_FAN_DUTY_VALUE || pump2_duty > MAX_FAN_DUTY_VALUE ||
	    pump3_duty > MAX_FAN_DUTY_VALUE) {
		LOG_ERR("Invalid PWM duty");
		return 1;
	}

	uint8_t ret = 0;
	uint32_t redundant_mode = get_status_flag(STATUS_FLAG_PUMP_REDUNDANT);
	uint8_t redundant_dev = (redundant_mode == PUMP_REDUNDANT_12) ? PWM_DEVICE_E_PB_PUMB_3 :
				(redundant_mode == PUMP_REDUNDANT_13) ? PWM_DEVICE_E_PB_PUMB_2 :
				(redundant_mode == PUMP_REDUNDANT_23) ? PWM_DEVICE_E_PB_PUMB_1 :
									PWM_DEVICE_E_MAX;

	if (!get_is_redundant_transforming())
		redundant_dev_now = redundant_dev;

	if (redundant_dev_now != PWM_DEVICE_E_MAX && redundant_dev_pre != redundant_dev_now) {
		switch (redundant_phase) {
		case REDUNDANCY_TRANSFORM_DISABLE:
			if (redundant_dev_pre != PWM_DEVICE_E_MAX) {
				set_is_redundant_transforming(true);
				for (uint8_t i = PWM_DEVICE_E_PB_PUMB_1;
				     i <= PWM_DEVICE_E_PB_PUMB_3; i++) {
					ret |= (plat_pwm_ctrl(i, 55) ? 1 : 0);
				}

				redundant_step1_count--;
				if (redundant_step1_count == 0) {
					redundant_phase = REDUNDANCY_TRANSFORM_STEP_1;
					redundant_step1_count = REDUNDANT_STEP1_RETRY;
				}
				return ret;
			} else {
				redundant_dev_pre = redundant_dev_now;
				break;
			}
		case REDUNDANCY_TRANSFORM_STEP_1:
			for (uint8_t i = PWM_DEVICE_E_PB_PUMB_1; i <= PWM_DEVICE_E_PB_PUMB_3; i++) {
				if (i == redundant_dev_now)
					ret |= (plat_pwm_ctrl(i, 20) ? 1 : 0);
				else
					ret |= (plat_pwm_ctrl(i, 100) ? 1 : 0);
			}

			redundant_step2a_count--;
			if (redundant_step2a_count == 0) {
				redundant_phase = REDUNDANCY_TRANSFORM_STEP_2A;
				redundant_step2a_count = REDUNDANT_STEP2A_RETRY;
			}
			return ret;
		case REDUNDANCY_TRANSFORM_STEP_2A:
			for (uint8_t i = PWM_DEVICE_E_PB_PUMB_1; i <= PWM_DEVICE_E_PB_PUMB_3; i++) {
				if (i == redundant_dev_now)
					ret |= (plat_pwm_ctrl(i, 0) ? 1 : 0);
				else
					ret |= (plat_pwm_ctrl(i, 100) ? 1 : 0);
			}

			redundant_step2b_count--;
			if (redundant_step2b_count == 0) {
				redundant_phase = REDUNDANCY_TRANSFORM_STEP_2B;
				redundant_step2b_count = REDUNDANT_STEP2B_RETRY;
			}
			return ret;
		case REDUNDANCY_TRANSFORM_STEP_2B:
			redundant_dev_pre = redundant_dev_now;
			break;
		}
	} else {
		set_is_redundant_transforming(false);		
		redundant_phase = REDUNDANCY_TRANSFORM_DISABLE;
		if (redundant_dev_now == PWM_DEVICE_E_MAX)
			redundant_dev_pre = PWM_DEVICE_E_MAX;
	}

	ret |= (plat_pwm_ctrl(PWM_DEVICE_E_PB_PUMB_1,
			      (redundant_mode == PUMP_REDUNDANT_23) ? 0 : pump1_duty) ?
			1 :
			0);
	ret |= (plat_pwm_ctrl(PWM_DEVICE_E_PB_PUMB_2,
			      (redundant_mode == PUMP_REDUNDANT_13) ? 0 : pump2_duty) ?
			1 :
			0);
	ret |= (plat_pwm_ctrl(PWM_DEVICE_E_PB_PUMB_3,
			      (redundant_mode == PUMP_REDUNDANT_12) ? 0 : pump3_duty) ?
			1 :
			0);

	return ret;
}

void reset_redundant_transform_status()
{
	redundant_phase = REDUNDANCY_TRANSFORM_DISABLE;
	redundant_dev_now = PWM_DEVICE_E_MAX;
	redundant_dev_pre = PWM_DEVICE_E_MAX;
	redundant_step1_count = REDUNDANT_STEP1_RETRY;
	redundant_step2a_count = REDUNDANT_STEP2A_RETRY;
	redundant_step2b_count = REDUNDANT_STEP2B_RETRY;
	is_redundant_transforming = false;
}

uint8_t ctl_all_pwm_dev(uint8_t duty)
{
	set_pwm_group(PWM_GROUP_E_HEX_FAN, duty);
	set_pwm_group(PWM_GROUP_E_PUMP, duty);
	set_pwm_group(PWM_GROUP_E_RPU_FAN, duty);

	return 0;
}

uint8_t set_pwm_group(uint8_t group, uint8_t duty)
{
	uint8_t ret = 1;

	fan_group_duty_cache[group] = duty;

	switch (group) {
	case PWM_GROUP_E_HEX_FAN:
		if (!ctl_pwm_dev(PWM_DEVICE_E_FB_FAN_1, PWM_DEVICE_E_FB_FAN_14, duty))
			ret = 0;
		break;
	case PWM_GROUP_E_PUMP:
		if (!ctl_pwm_dev(PWM_DEVICE_E_PB_PUMB_1, PWM_DEVICE_E_PB_PUMB_3, duty))
			ret = 0;
		break;
	case PWM_GROUP_E_RPU_FAN:
		if (!ctl_pwm_dev(PWM_DEVICE_E_PB_PUMB_FAN_1, PWM_DEVICE_E_BB_FAN, duty))
			ret = 0;
		break;
	};

	return ret;
}

uint8_t get_pwm_group_cache(uint8_t group)
{
	if (group >= PWM_GROUP_E_MAX)
		return 0xFF;

	return fan_group_duty_cache[group];
}


void set_pwm_group_cache(uint8_t group, uint8_t duty)
{
	fan_group_duty_cache[group] = duty;	
}

uint8_t get_pwm_cache(uint8_t idx)
{
	if (idx >= PWM_DEVICE_E_MAX)
		return 0xFF;

	return fan_duty_cache[idx];
}

uint8_t manual_pwm_idx_to_pwm_idx(uint8_t idx)
{
	return (idx == MANUAL_PWM_E_PUMP_1)	 ? PWM_DEVICE_E_PB_PUMB_1 :
	       (idx == MANUAL_PWM_E_PUMP_2)	 ? PWM_DEVICE_E_PB_PUMB_2 :
	       (idx == MANUAL_PWM_E_PUMP_3)	 ? PWM_DEVICE_E_PB_PUMB_3 :
	       (idx == MANUAL_PWM_E_PUMP_FAN_1)	 ? PWM_DEVICE_E_PB_PUMB_FAN_1 :
	       (idx == MANUAL_PWM_E_PUMP_FAN_2)	 ? PWM_DEVICE_E_PB_PUMB_FAN_2 :
	       (idx == MANUAL_PWM_E_PUMP_FAN_3)	 ? PWM_DEVICE_E_PB_PUMB_FAN_3 :
	       (idx == MANUAL_PWM_E_RPU_PCB_FAN) ? PWM_DEVICE_E_BB_FAN :
						   PWM_DEVICE_E_MAX;
}

uint8_t get_manual_pwm_flag(uint8_t idx)
{
	if (idx >= MANUAL_PWM_E_MAX)
		return 0xFF;

	return manual_pwm_flag[idx];
}

void set_manual_pwm_flag(uint8_t idx, uint8_t flag)
{
	if (idx >= MANUAL_PWM_E_MAX)
		return;

	manual_pwm_flag[idx] = flag;
}

uint8_t get_manual_pwm_cache(uint8_t idx)
{
	if (idx >= MANUAL_PWM_E_MAX)
		return 0xFF;

	return manual_pwm_cache[idx];
}

void set_manual_pwm_cache(uint8_t idx, uint8_t duty)
{
	if (idx >= MANUAL_PWM_E_MAX)
		return;

	manual_pwm_cache[idx] = duty;
}

void set_manual_pwm_cache_to_default(void)
{
	for (uint8_t i = 0; i < MANUAL_PWM_E_MAX; i++)
		manual_pwm_cache[i] = 70;
}

void set_manual_pwm_cache_to_zero(void)
{
	for (uint8_t i = 0; i < MANUAL_PWM_E_MAX; i++)
		manual_pwm_cache[i] = 0;
}

void init_pwm_dev(void)
{
	pwm_dev = device_get_binding("PWM");

	if (pwm_dev == NULL)
		LOG_ERR("FAN PWM init failed due to device not found");
}
