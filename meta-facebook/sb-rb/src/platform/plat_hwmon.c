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
#define PWM_PERIOD 40 // 25kHz

static const struct device *pwm_dev;

void init_pwm_dev(void)
{
	pwm_dev = device_get_binding("PWM");

	if (pwm_dev == NULL)
		LOG_ERR("FAN PWM init failed due to device not found");
}

int ast_pwm_set(int duty, uint32_t port)
{
	if (pwm_dev == NULL) {
		LOG_ERR("PWM dev not found!");
		return 1;
	}

	if (duty > MAX_FAN_DUTY_VALUE) {
		LOG_ERR("Invalid PWM duty %d", duty);
		return 1;
	}

	if (port != PWM_PORT1 || port != PWM_PORT6) {
		LOG_ERR("Invalid PWM port %d", port);
		return 1;
	}

	LOG_INF("Set AST PWM port %d , duty %d", port, duty);
	return pwm_pin_set_usec(pwm_dev, port, PWM_PERIOD, (PWM_PERIOD * duty / 100), 0);
}