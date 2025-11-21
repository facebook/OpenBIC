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
#include "libutil.h"
#include <logging/log.h>
#include "plat_cpld.h"
#include "plat_hwmon.h"

LOG_MODULE_REGISTER(plat_hwmon);

#define MAX_FAN_DUTY_VALUE 100
#define PWM_PERIOD 4000 // 25kHz

static const struct device *pwm1_dev;
static const struct device *pwm6_dev;

void init_pwm_dev(void)
{
	LOG_INF("FAN PWM init");
	pwm1_dev = device_get_binding("PWM_1");
	pwm6_dev = device_get_binding("PWM_6");

	if (pwm1_dev == NULL)
		LOG_ERR("FAN_1 PWM1 init failed due to device not found");

	if (pwm6_dev == NULL)
		LOG_ERR("FAN_6 PWM6 init failed due to device not found");
}

int ast_pwm_set(int duty, uint32_t port)
{
	if (pwm1_dev == NULL) {
		LOG_ERR("FAN_1 PWM1 dev not found!");
		return 1;
	}

	if (pwm6_dev == NULL) {
		LOG_ERR("FAN_6 PWM6 dev not found!");
		return 1;
	}

	if (duty > MAX_FAN_DUTY_VALUE) {
		LOG_ERR("Invalid PWM duty %d", duty);
		return 1;
	}

	if (port != PWM_PORT1 && port != PWM_PORT6) {
		LOG_ERR("Invalid PWM port %d", port);
		return 1;
	}

	LOG_INF("Set PWM port %d , duty %d", port, duty);
	//dev, pwm, period, pulse, flags)
	pwm_pin_set_cycles(pwm1_dev, port, PWM_PERIOD, (PWM_PERIOD * duty / 100), 0);
	pwm_pin_set_cycles(pwm6_dev, port, PWM_PERIOD, (PWM_PERIOD * duty / 100), 0);
	return 0;
}