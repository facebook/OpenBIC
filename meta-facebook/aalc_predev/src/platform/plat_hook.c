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
#include <string.h>
#include <logging/log.h>
#include "plat_hook.h"
#include "plat_class.h"
#include "nct7363.h"
#include "ina238.h"
#include "plat_i2c.h"
#include "sensor.h"
#include "nct214.h"
#include "libutil.h"

LOG_MODULE_REGISTER(plat_hook);

struct k_mutex i2c_1_PCA9546a_mutex;
struct k_mutex i2c_2_PCA9546a_mutex;
struct k_mutex i2c_6_pca9546a_mutex;
struct k_mutex i2c_7_PCA9546a_mutex;
struct k_mutex i2c_8_PCA9546a_mutex;
struct k_mutex i2c_9_PCA9546a_mutex;

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
adm1272_init_arg adm1272_init_args[] = {
	// init_args below need to check correct init value
	// fan board 1
	[0] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,      
	},
	// fan board 2
	[1] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// fan board 3
	[2] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// fan board 4
	[3] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// fan board 5
	[4] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// fan board 6
	[5] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// fan board 7
	[6] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// fan board 8
	[7] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// fan board 9
	[8] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// fan board 10
	[9] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// fan board 11
	[10] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// fan board 12
	[11] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// fan board 13
	[12] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// fan board 14
	[13] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// pump board 1
	[14] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// pump board 2
	[15] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// pump board 3
	[16] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 1,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// bridge board 
	[17] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 3,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
	// backplane board 
	[18] = { .is_init = false,
		.is_need_set_pwr_cfg = true,
		.pwr_monitor_cfg.value = 0x3F3F,
		.r_sense_mohm = 0.3,
		.is_record_ein = false,
		.last_energy = 0,
		.last_rollover = 0,
		.last_sample = 0,
	},
};

nct7363_init_arg nct7363_init_args[] = {
	// GPIO setting: Reserved=11, FANINx=10, PWMx=01, GPIOXX=00
	// gpio_dir == 1 : input(default), gpio_dir == 0 : output
	// Fan BD 1
	[0] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_17_PORT] = 25000, // TO DO wait to check
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},	
	// Fan BD 2
	[1] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_17_PORT] = 25000, // TO DO wait to check
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},	
	// fan BD 3
	[2] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_17_PORT] = 25000, // TO DO wait to check
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 4
	[3] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_17_PORT] = 25000, // TO DO wait to check
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 5
	[4] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_17_PORT] = 25000, // TO DO wait to check
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 6
	[5] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_17_PORT] = 25000, // TO DO wait to check
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 7
	[6] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_17_PORT] = 25000, // TO DO wait to check
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 8
	[7] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_17_PORT] = 25000, // TO DO wait to check
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 9
	[8] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_17_PORT] = 25000, // TO DO wait to check
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 10
	[9] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_17_PORT] = 25000, // TO DO wait to check
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 11
	[10] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_17_PORT] = 25000, // TO DO wait to check
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 12
	[11] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_17_PORT] = 25000, // TO DO wait to check
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 13
	[12] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_17_PORT] = 25000, // TO DO wait to check
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 14
	[13] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_17_PORT] = 25000, // TO DO wait to check
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// Pump BD 1
	[14] = { 
		.is_init = false,
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_PWM, // pump pwm
		.pin_type[NCT7363_2_PORT] = NCT7363_PIN_TPYE_PWM, // fan pwm
		.pin_type[NCT7363_5_PORT] = NCT7363_PIN_TPYE_FANIN, // pump fanin
		.pin_type[NCT7363_6_PORT] = NCT7363_PIN_TPYE_FANIN, // fan fanin
		.pin_type[NCT7363_7_PORT] = NCT7363_PIN_TPYE_FANIN, // fan fanin
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		// pwm setting
		.fan_frequency[NCT7363_1_PORT] = 500, // TO DO wait to check
		.fan_frequency[NCT7363_2_PORT] = 500, // TO DO wait to check
		// fanin setting
		.fan_poles[NCT7363_5_PORT] = 12, // TO DO wait to check
		.fan_poles[NCT7363_6_PORT] = 4, // TO DO wait to check
		.fan_poles[NCT7363_7_PORT] = 4, // TO DO wait to check
		.threshold[NCT7363_5_PORT] = 50, // TO DO wait to check
		.threshold[NCT7363_6_PORT] = 50, // TO DO wait to check
		.threshold[NCT7363_7_PORT] = 50, // TO DO wait to check
	},
	// Pump BD 2
	[15] = { 
		.is_init = false,
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_PWM, // pump pwm
		.pin_type[NCT7363_2_PORT] = NCT7363_PIN_TPYE_PWM, // fan pwm
		.pin_type[NCT7363_5_PORT] = NCT7363_PIN_TPYE_FANIN, // pump fanin
		.pin_type[NCT7363_6_PORT] = NCT7363_PIN_TPYE_FANIN, // fan fanin
		.pin_type[NCT7363_7_PORT] = NCT7363_PIN_TPYE_FANIN, // fan fanin
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		// pwm setting
		.fan_frequency[NCT7363_1_PORT] = 500, // TO DO wait to check
		.fan_frequency[NCT7363_2_PORT] = 500, // TO DO wait to check
		// fanin setting
		.fan_poles[NCT7363_5_PORT] = 12, // TO DO wait to check
		.fan_poles[NCT7363_6_PORT] = 4, // TO DO wait to check
		.fan_poles[NCT7363_7_PORT] = 4, // TO DO wait to check
		.threshold[NCT7363_5_PORT] = 50, // TO DO wait to check
		.threshold[NCT7363_6_PORT] = 50, // TO DO wait to check
		.threshold[NCT7363_7_PORT] = 50, // TO DO wait to check
	},
	// Pump BD 3
	[16] = { 
		.is_init = false,
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_PWM, // pump pwm
		.pin_type[NCT7363_2_PORT] = NCT7363_PIN_TPYE_PWM, // fan pwm
		.pin_type[NCT7363_5_PORT] = NCT7363_PIN_TPYE_FANIN, // pump fanin
		.pin_type[NCT7363_6_PORT] = NCT7363_PIN_TPYE_FANIN, // fan fanin
		.pin_type[NCT7363_7_PORT] = NCT7363_PIN_TPYE_FANIN, // fan fanin
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		// pwm setting
		.fan_frequency[NCT7363_1_PORT] = 500, // TO DO wait to check
		.fan_frequency[NCT7363_2_PORT] = 500, // TO DO wait to check
		// fanin setting
		.fan_poles[NCT7363_5_PORT] = 12, // TO DO wait to check
		.fan_poles[NCT7363_6_PORT] = 4, // TO DO wait to check
		.fan_poles[NCT7363_7_PORT] = 4, // TO DO wait to check
		.threshold[NCT7363_5_PORT] = 50, // TO DO wait to check
		.threshold[NCT7363_6_PORT] = 50, // TO DO wait to check
		.threshold[NCT7363_7_PORT] = 50, // TO DO wait to check
	},
	//Backplane BD
	[17] = { 
		.is_init = false,
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_PWM, // flow1 pwm
		.pin_type[NCT7363_2_PORT] = NCT7363_PIN_TPYE_PWM, // flow2 pwm
		.pin_type[NCT7363_3_PORT] = NCT7363_PIN_TPYE_PWM, // PUMP_ADD_WATER_PWM
		.pin_type[NCT7363_4_PORT] = NCT7363_PIN_TPYE_FANIN, // PUMP_ADD_WATER fanin
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_12_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_13_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_14_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_16_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		// pwm setting
		.fan_frequency[NCT7363_1_PORT] = 500, // TO DO wait to check
		.fan_frequency[NCT7363_2_PORT] = 500, // TO DO wait to check
		.fan_frequency[NCT7363_3_PORT] = 500, // TO DO wait to check
		// fanin setting
		.fan_poles[NCT7363_4_PORT] = 12, // TO DO wait to check
		.threshold[NCT7363_4_PORT] = 50, // TO DO wait to check
	},
	
};

ads112c_init_arg ads112c_init_args[] = {
	[0] = { .reg0_input = ADS112C_REG0_INPUT_AIN0AIN1,
		.reg0_gain = ADS112C_REG0_GAIN1,
		.reg0_pga = ADS112C_REG0_PGA_ENABLE,
		.reg1_conversion = ADS112C_REG1_CONTINUEMODE,
		.reg1_vol_refer = ADS112C_REG1_EXTERNALV,
		.vol_refer_val = 3.3,
		.reg1_temp_mode = ADS112C_REG1_TEMPMODE_DISABLE,		
	},    
	[1] = { .reg0_input = ADS112C_REG0_INPUT_AIN0AVSS,
		.reg0_gain = ADS112C_REG0_GAIN1,
		.reg0_pga = ADS112C_REG0_PGA_ENABLE,
		.reg1_conversion = ADS112C_REG1_CONTINUEMODE,
		.reg1_vol_refer = ADS112C_REG1_EXTERNALV,
		.vol_refer_val = 5,
		.reg1_temp_mode = ADS112C_REG1_TEMPMODE_DISABLE,				
	},
	[2] = { .reg0_input = ADS112C_REG0_INPUT_AIN1AVSS,
		.reg0_gain = ADS112C_REG0_GAIN1,
		.reg0_pga = ADS112C_REG0_PGA_ENABLE,
		.reg1_conversion = ADS112C_REG1_CONTINUEMODE,
		.reg1_vol_refer = ADS112C_REG1_EXTERNALV,
		.vol_refer_val = 5,
		.reg1_temp_mode = ADS112C_REG1_TEMPMODE_DISABLE,				
	},    
	[3] = { .reg0_input = ADS112C_REG0_INPUT_AIN2AVSS,
		.reg0_gain = ADS112C_REG0_GAIN1,
		.reg0_pga = ADS112C_REG0_PGA_ENABLE,
		.reg1_conversion = ADS112C_REG1_CONTINUEMODE,
		.reg1_vol_refer = ADS112C_REG1_EXTERNALV,
		.vol_refer_val = 5,
		.reg1_temp_mode = ADS112C_REG1_TEMPMODE_DISABLE,				
	},
	[4] = { .reg0_input = ADS112C_REG0_INPUT_AIN3AVSS,
		.reg0_gain = ADS112C_REG0_GAIN1,
		.reg0_pga = ADS112C_REG0_PGA_ENABLE,
		.reg1_conversion = ADS112C_REG1_CONTINUEMODE,
		.reg1_vol_refer = ADS112C_REG1_EXTERNALV,
		.vol_refer_val = 5,
		.reg1_temp_mode = ADS112C_REG1_TEMPMODE_DISABLE,				
	},
	[5] = { .reg0_input = ADS112C_REG0_INPUT_AIN0AVSS,
		.reg0_gain = ADS112C_REG0_GAIN1,
		.reg0_pga = ADS112C_REG0_PGA_ENABLE,
		.reg1_conversion = ADS112C_REG1_CONTINUEMODE,
		.reg1_vol_refer = ADS112C_REG1_INTERNALV,
		.vol_refer_val = 2.048,
		.reg1_temp_mode = ADS112C_REG1_TEMPMODE_DISABLE,
	},
	[6] = { .reg0_input = ADS112C_REG0_INPUT_AIN0AIN1,
		.reg0_gain = ADS112C_REG0_GAIN1,
		.reg0_pga = ADS112C_REG0_PGA_ENABLE,
		.reg1_conversion = ADS112C_REG1_CONTINUEMODE,
		.reg1_vol_refer = ADS112C_REG1_INTERNALV,
		.vol_refer_val = 2.048,
		.reg1_temp_mode = ADS112C_REG1_TEMPMODE_ENABLE,
	},		
};

adc_asd_init_arg adc_asd_init_args[] = {
	[0] = { .is_init = false,},
};

ina238_init_arg ina238_init_args[] = {
	// PDB board
	[0] = { 
		.is_init = false, 
		.r_shunt = 0.1, // TO DO wait to check
		.adc_range = INA238_ADC_RANGE_PN_163, // TO DO wait to check
		.alert_latch = INA238_ALERT_LATCH_ENABLE,
		.i_max = 0.1, // TO DO wait to check
	},
};

nct214_init_arg nct214_init_args[] = {
	// sensor box temp 1
	[0] = { 
		.is_init = false, 
		.temperature_range = NCT_214_TEMPERATURE_RANGE_EXTENDED, // TO DO wait to check
	},
	// sensor box temp 2
	[1] = { 
		.is_init = false, 
		.temperature_range = NCT_214_TEMPERATURE_RANGE_EXTENDED, // TO DO wait to check
	},
	// sensor box temp 3
	[2] = { 
		.is_init = false, 
		.temperature_range = NCT_214_TEMPERATURE_RANGE_EXTENDED, // TO DO wait to check
	},
	// sensor box temp 4
	[3] = { 
		.is_init = false, 
		.temperature_range = NCT_214_TEMPERATURE_RANGE_EXTENDED, // TO DO wait to check
	},
};

xdp710_init_arg xdp710_init_args[] = {
	// init_args below need to check correct init value
	// fan board
	[0] = { 
		.r_sense = 1, // TO DO wait to check
	},
	// pump board
	[1] = { 
		.r_sense = 1, // TO DO wait to check
	},
	// bridge board
	[2] = { 
		.r_sense = 1, // TO DO wait to check
	},
	// backplane board
	[3] = { 
		.r_sense = 1, // TO DO wait to check
	},
};

hdc1080_init_arg hdc1080_init_args[] = {
	// fan BD 1	
	[0] = { .is_init = false,},
	// fan BD 2	
	[1] = { .is_init = false,},
	// fan BD 3	
	[2] = { .is_init = false,},	
	// fan BD 4	
	[3] = { .is_init = false,},
	// fan BD 5	
	[4] = { .is_init = false,},
	// fan BD 6	
	[5] = { .is_init = false,},
	// fan BD 7	
	[6] = { .is_init = false,},
	// fan BD 8	
	[7] = { .is_init = false,},
	// fan BD 9	
	[8] = { .is_init = false,},
	// fan BD 10	
	[9] = { .is_init = false,},
	// fan BD 11	
	[10] = { .is_init = false,},
	// fan BD 12	
	[11] = { .is_init = false,},	
	// fan BD 13	
	[12] = { .is_init = false,},
	// fan BD 14	
	[13] = { .is_init = false,},
	// Pumb BD 1
	[14] = { .is_init = false,},
	// Pumb BD 2	
	[15] = { .is_init = false,},
	// Pumb BD 3	
	[16] = { .is_init = false,},
	// Manager BD	
	[17] = { .is_init = false,},
	// PDB BD	
	[18] = { .is_init = false,},			
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
mux_config bus_1_PCA9546A_configs[] = {
	[0] = { .target_addr = 0xE0, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = 0xE0, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = 0xE0, .channel = PCA9546A_CHANNEL_2 },
	[3] = { .target_addr = 0xE0, .channel = PCA9546A_CHANNEL_3 },
};
mux_config bus_2_PCA9546A_configs[] = {
	[0] = { .target_addr = 0xE2, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = 0xE2, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = 0xE2, .channel = PCA9546A_CHANNEL_2 },
	[3] = { .target_addr = 0xE2, .channel = PCA9546A_CHANNEL_3 },
};
mux_config bus_6_PCA9546A_configs[] = {
	[0] = { .target_addr = 0xE4, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = 0xE4, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = 0xE4, .channel = PCA9546A_CHANNEL_2 },
	[3] = { .target_addr = 0xE4, .channel = PCA9546A_CHANNEL_3 },
};
mux_config bus_7_PCA9546A_configs[] = {
	[0] = { .target_addr = 0xE6, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = 0xE6, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = 0xE6, .channel = PCA9546A_CHANNEL_2 },
	[3] = { .target_addr = 0xE6, .channel = PCA9546A_CHANNEL_3 },
};
mux_config bus_8_PCA9546A_configs[] = {
	[0] = { .target_addr = 0xE8, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = 0xE8, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = 0xE8, .channel = PCA9546A_CHANNEL_2 },
};
mux_config bus_9_PCA9546A_configs[] = {
	[0] = { .target_addr = 0xE8, .channel = PCA9546A_CHANNEL_1 }, // sensor box
	[1] = { .target_addr = 0xE8, .channel = PCA9546A_CHANNEL_2 }, // PDB
};
/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK FUNC
 **************************************************************************************************/
struct k_mutex *get_i2c_mux_mutex(uint8_t i2c_bus)
{
	struct k_mutex *mutex = NULL;

	switch (i2c_bus) {
	case I2C_BUS1:
		mutex = &i2c_1_PCA9546a_mutex;
		break;
	case I2C_BUS2:
		mutex = &i2c_2_PCA9546a_mutex;
		break;
	case I2C_BUS6:
		mutex = &i2c_6_pca9546a_mutex;
		break;
	case I2C_BUS7:
		mutex = &i2c_7_PCA9546a_mutex;
		break;
	case I2C_BUS8:
		mutex = &i2c_8_PCA9546a_mutex;
		break;
	case I2C_BUS9:
		mutex = &i2c_9_PCA9546a_mutex;
		break;
	default:
		LOG_ERR("No support for i2c bus %d mutex", i2c_bus);
		break;
	}

	return mutex;
}
bool pre_PCA9546A_read(sensor_cfg *cfg, void *args)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(args, false);

	// Select Channel
	bool ret = true;
	int mutex_status = 0;
	mux_config *pre_args = (mux_config *)args;
	pre_args->bus = cfg->port;

	struct k_mutex *mutex = get_i2c_mux_mutex(pre_args->bus);
	CHECK_NULL_ARG_WITH_RETURN(mutex, false);
	mutex_status = k_mutex_lock(mutex, K_MSEC(MUTEX_LOCK_INTERVAL_MS));
	if (mutex_status != 0) {
		LOG_ERR("Mutex lock fail, status: %d", mutex_status);
		return false;
	}

	ret = set_mux_channel(*pre_args, MUTEX_LOCK_ENABLE);
	if (ret != true) {
		k_mutex_unlock(mutex);
	}

	return ret;
}
bool post_PCA9546A_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	ARG_UNUSED(reading);
	ARG_UNUSED(args);

	int unlock_status = 0;
	uint8_t bus = cfg->port;

	struct k_mutex *mutex = get_i2c_mux_mutex(bus);
	CHECK_NULL_ARG_WITH_RETURN(mutex, false);
	if (mutex->lock_count != 0) {
		unlock_status = k_mutex_unlock(mutex);
	}

	if (unlock_status != 0) {
		LOG_ERR("Mutex unlock fail, status: %d", unlock_status);
		return false;
	}

	return true;
}
bool post_adm1272_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(reading, false);
	ARG_UNUSED(args);

	if (reading == NULL) {
		return check_reading_pointer_null_is_allowed(cfg);
	}

	sensor_val *sval = (sensor_val *)reading;
	if (cfg->offset == PMBUS_READ_IOUT || cfg->offset == PMBUS_READ_IIN) {
		// Adjust negative current value to zero according to power team suggestion
		if ((int)sval->integer < 0) {
			*reading = 0;
			return true;
		}
	}

	if (cfg->offset == PMBUS_READ_IOUT || cfg->offset == PMBUS_READ_IIN ||
	    cfg->offset == PMBUS_READ_POUT || cfg->offset == PMBUS_READ_PIN) {
		// multiply 98% for accuracy
		float val = ((float)sval->integer + (sval->fraction / 1000.0)) * 0.98;
		sval->integer = (int)val & 0xFFFF;
		sval->fraction = (val - sval->integer) * 1000;
	}

	return true;
}

bool post_ads112c_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(reading, false);

	sensor_val *oval = (sensor_val *)reading;
	double rawValue = ((float)oval->integer + (oval->fraction / 1000.0));

	double val;
	double v_val, flow_Pmax = 400, flow_Pmin = 10, press_Pmax = 50, press_Pmin = 0;

	switch (cfg->offset) {
	case ADS112C_FLOW_OFFSET: //Flow_Rate_LPM
		v_val = 5 - ((32767 - rawValue) * 0.000153);
		val = (((v_val / 5) - 0.1) / (0.8 / (flow_Pmax - flow_Pmin))) + 10;
		val = (val - 7.56494) * 1.076921;
		val = (2.5412 * val) - 25.285;
		break;

	case ADS112C_PRESS_OFFSET: //Filter_P/Outlet_P/Inlet_P
		v_val = 5 - ((32767 - rawValue) * 0.000153);
		val = (((v_val / 5) - 0.1) / (0.8 / (press_Pmax - press_Pmin))) + 10;
		val = ((0.9828 * val) - 9.9724) * 6.894759;
		break;

	case ADS112C_TEMP_OFFSET: //RDHx_Hot_Liq_T/CDU_Inlet_Liq_T
		val = (rawValue - 15888) * 0.015873;
		val = (1.1685 * val) - 4.5991;
		break;

	default:
		val = rawValue * 0.0001007;
		break;
	}

	sensor_val *sval = (sensor_val *)reading;
	sval->integer = (int)val & 0xFFFF;
	sval->fraction = (val - sval->integer) * 1000;
	//according to pre_sensor_read_fn(pre_PCA9546A_read), determinies if apply post_PCA9546A_read
	if (cfg->pre_sensor_read_hook != NULL) {
		post_PCA9546A_read(cfg, args, reading);
	}

	return true;
}
