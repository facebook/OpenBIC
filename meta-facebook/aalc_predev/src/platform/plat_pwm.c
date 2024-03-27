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

#include "plat_pwm.h"

LOG_MODULE_REGISTER(plat_pwm);

#define MAX_FAN_DUTY_VALUE 100

static const struct device *pwm_dev;

int ast_pwm_set(int duty)
{
	return pwm_pin_set_cycles(pwm_dev, PWM_PORT0, MAX_FAN_DUTY_VALUE, (uint32_t)duty, 0);
}

void init_pwm_dev(void)
{
	pwm_dev = device_get_binding("PWM");

	if (pwm_dev == NULL)
		LOG_ERR("FAN PWM init failed due to device not found");
}
