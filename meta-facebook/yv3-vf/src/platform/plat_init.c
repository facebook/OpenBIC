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

#include "util_worker.h"
#include "ipmi.h"
#include "sensor.h"
#include "power_status.h"
#include "expansion_board.h"

#include "plat_sensor_table.h"
#include "plat_class.h"
#include "plat_hwmon.h"
#include "plat_m2.h"
#include "plat_isr.h"
#include "plat_power_seq.h"
#include "plat_led.h"
#include "plat_gpio.h"

SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e6e2610, 0x0E000100 },
	{ 0x7e6e2614, 0x00006000 },
};

extern uint8_t ina230_init(uint8_t sensor_num);
static void BICup5secTickHandler(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(up_1sec_handler, BICup1secTickHandler);
K_WORK_DELAYABLE_DEFINE(up_5sec_handler, BICup5secTickHandler);

void pal_pre_init()
{
	init_platform_config();
	init_e1s_config();
	init_worker(); // init util_worker
	scu_init(scu_cfg, ARRAY_SIZE(scu_cfg));
}

static void BICup5secTickHandler(struct k_work *work)
{
	if (!work) {
		printf("BICup5secTickHandler get null work handler!\n");
		return;
	}

	if (!sensor_config) {
		printf("sensor_config is null!\n");
		return;
	}

	uint8_t sensor_config_num = sensor_config_count;
	if (get_e1s_adc_config() == CONFIG_ADC_INA231) {
		/* config on-board INA231 */
		for (int i = 0; i < sensor_config_num; i++) {
			if (sensor_config[i].type != sensor_dev_ina230)
				continue;

			if (ina230_init(sensor_config[i].num) != SENSOR_INIT_SUCCESS) {
				printf("sensor_config[%02x].num = %02x re-init ina230 failed!, retry it after 5 seconds\n",
				       i, sensor_config[i].num);
				k_work_schedule((struct k_work_delayable *)work, K_SECONDS(5));
				return;
			}
		}
	} else if (get_e1s_adc_config() == CONFIG_ADC_ISL28022) {
		// e1s_isl28022_init();
	}
}

void pal_set_sys_status()
{
	pwr_related_pin_init();

	SSDLEDInit();

	set_DC_status(FM_POWER_EN);
	set_DC_on_delayed_status();

	check_irq_fault();
	// BIC up 1 sec handler
	k_work_schedule(&up_1sec_handler, K_SECONDS(1));
	// BIC up 5 sec handler
	k_work_schedule(&up_5sec_handler, K_SECONDS(5));
}

#define DEF_PROJ_GPIO_PRIORITY 78

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);