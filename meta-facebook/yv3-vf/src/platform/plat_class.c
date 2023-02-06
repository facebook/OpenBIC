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
#include "hal_gpio.h"
#include "plat_class.h"
#include "expansion_board.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(plat_class);

static uint8_t e1s_hsc_config = 0;
static uint8_t e1s_adc_config = 0;

#define VERNAL_FALLS_BOARD_TYPE 0x07

void init_e1s_config()
{
	uint8_t config = 0;
	config = ((gpio_get(HSC_SEL_ID0)) | (gpio_get(HSC_SEL_ID1) << 1) |
		  (gpio_get(HSC_SEL_ID2) << 2));

	e1s_hsc_config = config & 0x03;
	e1s_adc_config = config >> 2;
	LOG_INF("hsc config 0x%x", e1s_hsc_config);
	LOG_INF("adc config 0x%x", e1s_adc_config);
}

uint8_t get_e1s_hsc_config()
{
	return e1s_hsc_config;
}

uint8_t get_e1s_adc_config()
{
	return e1s_adc_config;
}

uint8_t get_e1s_pwrgd()
{
	switch (get_e1s_hsc_config()) {
	case CONFIG_HSC_ADM1278:
	case CONFIG_HSC_MAXIN:
	case CONFIG_HSC_MPS:
		return gpio_get(PWRGD_P12V_AUX);

	case CONFIG_HSC_BYPASS:
	default:
		return gpio_get(FM_POWER_EN);
	}
}
