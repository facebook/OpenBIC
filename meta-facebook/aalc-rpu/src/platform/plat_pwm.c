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
#include "plat_pwm.h"

LOG_MODULE_REGISTER(plat_pwm);

#define MAX_FAN_DUTY_VALUE 100
#define PWM_PERIOD 40 // 25kHz
#define PWM_DEVICE_E_FAN_START PWM_DEVICE_E_FB_FAN_1
#define PWM_DEVICE_E_FAN_END PWM_DEVICE_E_FB_FAN_14
#define PWM_DEVICE_E_PUMB_START PWM_DEVICE_E_PB_PUMB_1
#define PWM_DEVICE_E_PUMB_END PWM_DEVICE_E_PB_PUMB_FAN_3

static const struct device *pwm_dev;
static uint8_t fan_duty_setting;
static uint8_t pumb_duty_setting;

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

int ast_pwm_set(int duty)
{
	LOG_DBG("Set AST PWM duty %d", duty);
	if (pwm_dev == NULL) {
		LOG_ERR("PWM dev not found!");
		return 1;
	}
	return pwm_pin_set_usec(pwm_dev, PWM_PORT0, PWM_PERIOD, (PWM_PERIOD * duty / 100), 0);
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
	case PWM_DEVICE_E_PB_PUMB_1:
	case PWM_DEVICE_E_PB_PUMB_2:
	case PWM_DEVICE_E_PB_PUMB_3:
	case PWM_DEVICE_E_PB_PUMB_FAN_1:
	case PWM_DEVICE_E_PB_PUMB_FAN_2:
	case PWM_DEVICE_E_PB_PUMB_FAN_3:
		ret = nct_pwm_ctl(dev, duty);
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

	for (uint8_t i = index_start; i <= index_end; i++) {
		if (plat_pwm_ctrl(i, duty)) {
			LOG_ERR("Failed to set PWM device %d duty %d", i, duty);
			ret = 1;
			//break;
		}
	}

	return ret;
}

uint8_t ctl_all_pwm_dev(uint8_t duty)
{
	// Hex Fan & RPU Pump Setting
	if (ctl_hex_fan_pwm_dev(duty) || ctl_pump_pwm_dev(duty))
		return 1;

	// other PWM setting
	if (plat_pwm_ctrl(PWM_DEVICE_E_BB_FAN, duty))
		return 1;
	return 0;
}

uint8_t ctl_hex_fan_pwm_dev(uint8_t duty)
{
	if (!ctl_pwm_dev(PWM_DEVICE_E_FAN_START, PWM_DEVICE_E_FAN_END, duty))
		fan_duty_setting = duty;
	else
		return 1;

	return 0;
}

uint8_t ctl_pump_pwm_dev(uint8_t duty)
{
	if (!ctl_pwm_dev(PWM_DEVICE_E_PUMB_START, PWM_DEVICE_E_PUMB_END, duty))
		pumb_duty_setting = duty;
	else
		return 1;

	return 0;
}

uint8_t ctl_other_pwm_dev(uint8_t duty)
{
	if (!ctl_pwm_dev(PWM_DEVICE_E_PUMB_START, PWM_DEVICE_E_PUMB_END, duty))
		pumb_duty_setting = duty;
	else
		return 1;

	return 0;
}

uint8_t fan_pwm_dev_duty_setting(void)
{
	return fan_duty_setting;
}

uint8_t pump_pwm_dev_duty_setting(void)
{
	return pumb_duty_setting;
}

void init_pwm_dev(void)
{
	pwm_dev = device_get_binding("PWM");

	if (pwm_dev == NULL)
		LOG_ERR("FAN PWM init failed due to device not found");
}
