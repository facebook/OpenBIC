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
#include "plat_sensor_table.h"

LOG_MODULE_REGISTER(plat_hook);

#define FB_PRSNT_NUM_PER_IOEXP 7

#define FB_PWR_PRSNT_ADDR_EVEN 0x4A // 8 bit address (2, 4, 6, 8, 10, 12, 14)
#define FB_PWR_PRSNT_ADDR_ODD 0x4E // 8 bit address (1, 3, 5, 7, 9, 11, 13)
#define FB_SIG_PRSNT_ADDR_EVEN 0x4A // 8 bit address (2, 4, 6, 8, 10, 12, 14)
#define FB_SIG_PRSNT_ADDR_ODD 0x4E // 8 bit address (1, 3, 5, 7, 9, 11, 13)

#define DIFFERENTIAL_MODE 0x01
#define FLOW_SENSOR_CACHE_MAX_NUM 30
static flow_cache_data_mapping flow_cache_data[FLOW_SENSOR_CACHE_MAX_NUM];

K_MUTEX_DEFINE(i2c_1_PCA9546a_mutex);
K_MUTEX_DEFINE(i2c_2_PCA9546a_mutex);
K_MUTEX_DEFINE(i2c_6_PCA9546a_mutex);
K_MUTEX_DEFINE(i2c_7_PCA9546a_mutex);
K_MUTEX_DEFINE(i2c_8_PCA9546a_mutex);
K_MUTEX_DEFINE(i2c_9_PCA9546a_mutex);

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
		.gpio_00 = 1, //gpio out default value (active low)
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_PWM,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_11_PORT] = 2, 
		.fan_frequency[NCT7363_17_PORT] = 25000, 
		.duty[NCT7363_11_PORT] = 0, 
		.duty[NCT7363_17_PORT] = 0,
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},	
	// Fan BD 2
	[1] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_00 = 1, //gpio out default value (active low)
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_PWM,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_11_PORT] = 2, 
		.fan_frequency[NCT7363_17_PORT] = 25000, 
		.duty[NCT7363_11_PORT] = 0, 
		.duty[NCT7363_17_PORT] = 0,
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},	
	// fan BD 3
	[2] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_00 = 1, //gpio out default value (active low)
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_PWM,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_11_PORT] = 2, 
		.fan_frequency[NCT7363_17_PORT] = 25000, 
		.duty[NCT7363_11_PORT] = 0, 
		.duty[NCT7363_17_PORT] = 0,
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 4
	[3] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_00 = 1, //gpio out default value (active low)
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_PWM,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_11_PORT] = 2, 
		.fan_frequency[NCT7363_17_PORT] = 25000, 
		.duty[NCT7363_11_PORT] = 0, 
		.duty[NCT7363_17_PORT] = 0,
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 5
	[4] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_00 = 1, //gpio out default value (active low)
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_PWM,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_11_PORT] = 2, 
		.fan_frequency[NCT7363_17_PORT] = 25000, 
		.duty[NCT7363_11_PORT] = 0, 
		.duty[NCT7363_17_PORT] = 0,
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 6
	[5] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_00 = 1, //gpio out default value (active low)
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_PWM,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_11_PORT] = 2, 
		.fan_frequency[NCT7363_17_PORT] = 25000, 
		.duty[NCT7363_11_PORT] = 0, 
		.duty[NCT7363_17_PORT] = 0,
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 7
	[6] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_00 = 1, //gpio out default value (active low)
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_PWM,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_11_PORT] = 2, 
		.fan_frequency[NCT7363_17_PORT] = 25000, 
		.duty[NCT7363_11_PORT] = 0, 
		.duty[NCT7363_17_PORT] = 0,
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 8
	[7] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_00 = 1, //gpio out default value (active low)
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_PWM,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_11_PORT] = 2, 
		.fan_frequency[NCT7363_17_PORT] = 25000, 
		.duty[NCT7363_11_PORT] = 0, 
		.duty[NCT7363_17_PORT] = 0,
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 9
	[8] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_00 = 1, //gpio out default value (active low)
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_PWM,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_11_PORT] = 2, 
		.fan_frequency[NCT7363_17_PORT] = 25000, 
		.duty[NCT7363_11_PORT] = 0, 
		.duty[NCT7363_17_PORT] = 0,
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 10
	[9] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_00 = 1, //gpio out default value (active low)
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_PWM,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_11_PORT] = 2, 
		.fan_frequency[NCT7363_17_PORT] = 25000, 
		.duty[NCT7363_11_PORT] = 0, 
		.duty[NCT7363_17_PORT] = 0,
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 11
	[10] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_00 = 1, //gpio out default value (active low)
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_PWM,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_11_PORT] = 2, 
		.fan_frequency[NCT7363_17_PORT] = 25000, 
		.duty[NCT7363_11_PORT] = 0, 
		.duty[NCT7363_17_PORT] = 0,
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 12
	[11] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_00 = 1, //gpio out default value (active low)
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_PWM,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_11_PORT] = 2, 
		.fan_frequency[NCT7363_17_PORT] = 25000, 
		.duty[NCT7363_11_PORT] = 0, 
		.duty[NCT7363_17_PORT] = 0,
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 13
	[12] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_00 = 1, //gpio out default value (active low)
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_PWM,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_11_PORT] = 2, 
		.fan_frequency[NCT7363_17_PORT] = 25000, 
		.duty[NCT7363_11_PORT] = 0, 
		.duty[NCT7363_17_PORT] = 0,
		// fanin setting
		.threshold[NCT7363_15_PORT] = 50, // TO DO wait to check
		.fan_poles[NCT7363_15_PORT] = 4, // TO DO wait to check
	},
	// fan BD 14
	[13] = { 
		.is_init = false, 
		.wdt_cfg = WDT_7_5_SEC,
		.pin_type[NCT7363_1_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_00 = 1, //gpio out default value (active low)
		.pin_type[NCT7363_10_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_PWM,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_FANIN,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_PWM,
		// pwm setting
		.fan_frequency[NCT7363_11_PORT] = 2, 
		.fan_frequency[NCT7363_17_PORT] = 25000, 
		.duty[NCT7363_11_PORT] = 0, 
		.duty[NCT7363_17_PORT] = 0,
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
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_11 = 0,
		// pwm setting
		.fan_frequency[NCT7363_1_PORT] = 1000, // TO DO wait to check
		.fan_frequency[NCT7363_2_PORT] = 25000, // TO DO wait to check
		.duty[NCT7363_1_PORT] = 0,
		.duty[NCT7363_2_PORT] = 0,
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
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_11 = 0,
		// pwm setting
		.fan_frequency[NCT7363_1_PORT] = 1000, // TO DO wait to check
		.fan_frequency[NCT7363_2_PORT] = 25000, // TO DO wait to check
		.duty[NCT7363_1_PORT] = 0,
		.duty[NCT7363_2_PORT] = 0,
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
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_11 = 0,
		// pwm setting
		.fan_frequency[NCT7363_1_PORT] = 1000, // TO DO wait to check
		.fan_frequency[NCT7363_2_PORT] = 25000, // TO DO wait to check
		.duty[NCT7363_1_PORT] = 0,
		.duty[NCT7363_2_PORT] = 0,
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
		.gpio_10 = 1,
		.pin_type[NCT7363_11_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_12_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_13_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_14_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_15_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.gpio_15 = 1, //gpio out default value (active low)
		.pin_type[NCT7363_16_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		.pin_type[NCT7363_17_PORT] = NCT7363_PIN_TPYE_GPIO_DEFAULT_OUTPUT,
		// pwm setting
		.fan_frequency[NCT7363_1_PORT] = 500, // TO DO wait to check
		.fan_frequency[NCT7363_2_PORT] = 500, // TO DO wait to check
		.fan_frequency[NCT7363_3_PORT] = 500, // TO DO wait to check
		.duty[NCT7363_1_PORT] = 0,
		.duty[NCT7363_2_PORT] = 0,
		.duty[NCT7363_3_PORT] = 0,
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
		.reg2_idac = ADS112C_REG2_IDAC_OFF,
		.reg3_idac1_cfg = ADS112C_REG3_IDAC1_DISABLED,	
	},    
	[1] = { .reg0_input = ADS112C_REG0_INPUT_AIN0AVSS,
		.reg0_gain = ADS112C_REG0_GAIN1,
		.reg0_pga = ADS112C_REG0_PGA_DISABLE,
		.reg1_conversion = ADS112C_REG1_CONTINUEMODE,
		.reg1_vol_refer = ADS112C_REG1_EXTERNALV,
		.vol_refer_val = 5,
		.reg1_temp_mode = ADS112C_REG1_TEMPMODE_DISABLE,
		.reg2_idac = ADS112C_REG2_IDAC_OFF,
		.reg3_idac1_cfg = ADS112C_REG3_IDAC1_DISABLED,						
	},
	[2] = { .reg0_input = ADS112C_REG0_INPUT_AIN1AVSS,
		.reg0_gain = ADS112C_REG0_GAIN1,
		.reg0_pga = ADS112C_REG0_PGA_DISABLE,
		.reg1_conversion = ADS112C_REG1_CONTINUEMODE,
		.reg1_vol_refer = ADS112C_REG1_EXTERNALV,
		.vol_refer_val = 5,
		.reg1_temp_mode = ADS112C_REG1_TEMPMODE_DISABLE,
		.reg2_idac = ADS112C_REG2_IDAC_OFF,
		.reg3_idac1_cfg = ADS112C_REG3_IDAC1_DISABLED,						
	},    
	[3] = { .reg0_input = ADS112C_REG0_INPUT_AIN2AVSS,
		.reg0_gain = ADS112C_REG0_GAIN1,
		.reg0_pga = ADS112C_REG0_PGA_DISABLE,
		.reg1_conversion = ADS112C_REG1_CONTINUEMODE,
		.reg1_vol_refer = ADS112C_REG1_EXTERNALV,
		.vol_refer_val = 5,
		.reg1_temp_mode = ADS112C_REG1_TEMPMODE_DISABLE,
		.reg2_idac = ADS112C_REG2_IDAC_OFF,
		.reg3_idac1_cfg = ADS112C_REG3_IDAC1_DISABLED,						
	},
	[4] = { .reg0_input = ADS112C_REG0_INPUT_AIN3AVSS,
		.reg0_gain = ADS112C_REG0_GAIN1,
		.reg0_pga = ADS112C_REG0_PGA_DISABLE,
		.reg1_conversion = ADS112C_REG1_CONTINUEMODE,
		.reg1_vol_refer = ADS112C_REG1_EXTERNALV,
		.vol_refer_val = 5,
		.reg1_temp_mode = ADS112C_REG1_TEMPMODE_DISABLE,
		.reg2_idac = ADS112C_REG2_IDAC_OFF,
		.reg3_idac1_cfg = ADS112C_REG3_IDAC1_DISABLED,						
	},
	[5] = { .reg0_input = ADS112C_REG0_INPUT_AIN0AVSS,
		.reg0_gain = ADS112C_REG0_GAIN1,
		.reg0_pga = ADS112C_REG0_PGA_DISABLE,
		.reg1_conversion = ADS112C_REG1_CONTINUEMODE,
		.reg1_vol_refer = ADS112C_REG1_INTERNALV,
		.vol_refer_val = 2.048,
		.reg1_temp_mode = ADS112C_REG1_TEMPMODE_DISABLE,
		.reg2_idac = ADS112C_REG2_IDAC_OFF,
		.reg3_idac1_cfg = ADS112C_REG3_IDAC1_DISABLED,
	},
	[6] = { .reg0_input = ADS112C_REG0_INPUT_AIN1AIN0,
		.reg0_gain = ADS112C_REG0_GAIN8,
		.reg0_pga = ADS112C_REG0_PGA_ENABLE,
		.reg1_conversion = ADS112C_REG1_CONTINUEMODE,
		.reg1_vol_refer = ADS112C_REG1_EXTERNALV,
		//.reg1_vol_refer = ADS112C_REG1_INTERNALV,
		.vol_refer_val = 2.048,
		//.reg1_temp_mode = ADS112C_REG1_TEMPMODE_ENABLE,
		.reg1_temp_mode = ADS112C_REG1_TEMPMODE_DISABLE,
		.reg2_idac = ADS112C_REG2_IDAC_1000UA,
		.reg3_idac1_cfg = ADS112C_REG3_IDAC1_AIN3,
	},
	[7] = { .reg0_input = ADS112C_REG0_INPUT_AIN2AIN3,
		.reg0_gain = ADS112C_REG0_GAIN1,
		.reg0_pga = ADS112C_REG0_PGA_ENABLE,
		.reg1_conversion = ADS112C_REG1_CONTINUEMODE,
		.reg1_vol_refer = ADS112C_REG1_EXTERNALV,
		.vol_refer_val = 3.3,
		.reg1_temp_mode = ADS112C_REG1_TEMPMODE_DISABLE,
		.reg2_idac = ADS112C_REG2_IDAC_OFF,
		.reg3_idac1_cfg = ADS112C_REG3_IDAC1_DISABLED,				
	},
	// leakage sensor, 1000 sps
	[8] = { .reg0_input = ADS112C_REG0_INPUT_AIN0AIN1,
		.reg0_gain = ADS112C_REG0_GAIN1,
		.reg0_pga = ADS112C_REG0_PGA_ENABLE,
		.reg1_dr = ADS112C_REG1_DR_1000_SPS,
		.reg1_conversion = ADS112C_REG1_CONTINUEMODE,
		.reg1_vol_refer = ADS112C_REG1_EXTERNALV,
		.vol_refer_val = 3.3,
		.reg1_temp_mode = ADS112C_REG1_TEMPMODE_DISABLE,
		.reg2_idac = ADS112C_REG2_IDAC_OFF,
		.reg3_idac1_cfg = ADS112C_REG3_IDAC1_DISABLED,
	},
};

ads112c_post_arg ads112c_post_args[] = {
	[0] = { .plat_sensor_type = PLATFORM_ADS112C_FLOW,}, 
	[1] = { .plat_sensor_type = PLATFORM_ADS112C_PRESS,}, 
	[2] = { .plat_sensor_type = PLATFORM_ADS112C_TEMP_RPU,}, 
	[3] = { .plat_sensor_type = PLATFORM_ADS112C_OTHER,}, 
	[4] = { .plat_sensor_type = PLATFORM_ADS112C_LEAKAGE,},
	[5] = { .plat_sensor_type = PLATFORM_ADS112C_TEMP_RACK,},	
};

adc_asd_init_arg adc_asd_init_args[] = {
	[0] = { .is_init = false,},
};

ina238_init_arg ina238_init_args[] = {
	// PDB board positive
	[0] = { 
		.is_init = false, 
		.r_shunt = 0.0005, // TO DO wait to check
		.adc_range = INA238_ADC_RANGE_PN_163, // TO DO wait to check
		.alert_latch = INA238_ALERT_LATCH_ENABLE,
		.i_max = 175, // TO DO wait to check
	},
	// PDB board negative
	[1] = { 
		.is_init = false, 
		.r_shunt = 0.0005, // TO DO wait to check
		.adc_range = INA238_ADC_RANGE_PN_163, // TO DO wait to check
		.alert_latch = INA238_ALERT_LATCH_ENABLE,
		.i_max = 175, // TO DO wait to check
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

ast_tach_init_arg ast_tach_init_args[] = {
	[0] = { .is_init = false,},
	[1] = { .is_init = false,},
};

e50sn12051_init_arg e50sn12051_init_args[] = {
	[0] = { .is_init = false,},
};

bmr4922302_803_init_arg bmr4922302_803_init_args[] = {
	[0] = { .is_init = false,},
};

/**************************************************************************************************
 *  PRE-HOOK/POST-HOOK ARGS
 **************************************************************************************************/
mux_config bus_1_PCA9546A_configs[] = {
	[0] = { .target_addr = BUS_1_MUX_ADDR, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = BUS_1_MUX_ADDR, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = BUS_1_MUX_ADDR, .channel = PCA9546A_CHANNEL_2 },
	[3] = { .target_addr = BUS_1_MUX_ADDR, .channel = PCA9546A_CHANNEL_3 },
};
mux_config bus_2_PCA9546A_configs[] = {
	[0] = { .target_addr = BUS_2_MUX_ADDR, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = BUS_2_MUX_ADDR, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = BUS_2_MUX_ADDR, .channel = PCA9546A_CHANNEL_2 },
	[3] = { .target_addr = BUS_2_MUX_ADDR, .channel = PCA9546A_CHANNEL_3 },
};
mux_config bus_6_PCA9546A_configs[] = {
	[0] = { .target_addr = BUS_6_MUX_ADDR, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = BUS_6_MUX_ADDR, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = BUS_6_MUX_ADDR, .channel = PCA9546A_CHANNEL_2 },
	[3] = { .target_addr = BUS_6_MUX_ADDR, .channel = PCA9546A_CHANNEL_3 },
};
mux_config bus_7_PCA9546A_configs[] = {
	[0] = { .target_addr = BUS_7_MUX_ADDR, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = BUS_7_MUX_ADDR, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = BUS_7_MUX_ADDR, .channel = PCA9546A_CHANNEL_2 },
	[3] = { .target_addr = BUS_7_MUX_ADDR, .channel = PCA9546A_CHANNEL_3 },
};
mux_config bus_8_PCA9546A_configs[] = {
	[0] = { .target_addr = BUS_8_MUX_ADDR, .channel = PCA9546A_CHANNEL_0 },
	[1] = { .target_addr = BUS_8_MUX_ADDR, .channel = PCA9546A_CHANNEL_1 },
	[2] = { .target_addr = BUS_8_MUX_ADDR, .channel = PCA9546A_CHANNEL_2 },
};
mux_config bus_9_PCA9546A_configs[] = {
	[0] = { .target_addr = BUS_9_MUX_ADDR, .channel = PCA9546A_CHANNEL_1 }, // sensor box
	[1] = { .target_addr = BUS_9_MUX_ADDR, .channel = PCA9546A_CHANNEL_2 }, // PDB
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
		mutex = &i2c_6_PCA9546a_mutex;
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
		LOG_ERR("change channel fail, unlock mutex");
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

	if (reading == NULL) {
		// if pre_sensor_read_hook is not null, unlock PCA9546A mutex
		if (cfg->pre_sensor_read_hook != NULL) {
			if (!post_PCA9546A_read(cfg, args, reading)) {
				LOG_ERR("adm1272 in post read unlock PCA9546A mutex fail");
				return false;
			}
		}
		return check_reading_pointer_null_is_allowed(cfg);
	}

	ARG_UNUSED(args);

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

	// if pre_sensor_read_hook is not null, unlock PCA9546A mutex
	if (cfg->pre_sensor_read_hook != NULL) {
		if (!post_PCA9546A_read(cfg, args, reading)) {
			LOG_ERR("adm1272 in post read unlock PCA9546A mutex fail");
			return false;
		}
	}

	return true;
}

static double averge_flow_rate(double val)
{
	bool is_cache = false;
	for (uint8_t i = 0; i < FLOW_SENSOR_CACHE_MAX_NUM; i++) {
		if (flow_cache_data[i].is_newest && flow_cache_data[i].is_record) {
			flow_cache_data[i].is_newest = false;

			uint8_t next = ((i + 1) % FLOW_SENSOR_CACHE_MAX_NUM);
			flow_cache_data[next].is_newest = true;
			flow_cache_data[next].is_record = true;
			flow_cache_data[next].flow_val = val;

			is_cache = true;
			break;
		}
	}

	if (!is_cache) {
		flow_cache_data[0].is_newest = true;
		flow_cache_data[0].is_record = true;
		flow_cache_data[0].flow_val = val;
	}

	uint8_t count_num = 0;
	double total_val = 0;
	for (uint8_t i = 0; i < FLOW_SENSOR_CACHE_MAX_NUM; i++) {
		if (flow_cache_data[i].is_record) {
			total_val = total_val + flow_cache_data[i].flow_val;
			count_num++;
		}
	}

	return (count_num > 0) ? (total_val / count_num) : 0.0;
}

bool post_ads112c_read(sensor_cfg *cfg, void *args, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	if (reading == NULL) {
		// if pre_sensor_read_hook is not null, unlock PCA9546A mutex
		if (cfg->pre_sensor_read_hook != NULL) {
			if (!post_PCA9546A_read(cfg, args, reading)) {
				LOG_ERR("adm1272 in post read unlock PCA9546A mutex fail");
				return false;
			}
		}
		return check_reading_pointer_null_is_allowed(cfg);
	}

	sensor_val *oval = (sensor_val *)reading;
	double rawValue = ((uint16_t)oval->integer + (oval->fraction / 1000.0));

	double val;
	double v_val, flow_Pmax = 400, flow_Pmin = 10, press_Pmax = 50, press_Pmin = 0;

	ads112c_post_arg *post_args = (ads112c_post_arg *)args;
	switch (post_args->plat_sensor_type) {
	case PLATFORM_ADS112C_FLOW: //Flow_Rate_LPM
		v_val = 5 - ((32767 - rawValue) * 0.000153);
		val = (((v_val / 5.0) - 0.1) * (flow_Pmax - flow_Pmin) / 0.8);
		val = 1.3232 * val - 1.0314;
		val = (val < 0) ? 0 : val;
		val = averge_flow_rate(val);
		break;
	case PLATFORM_ADS112C_PRESS: //Filter_P/Outlet_P/Inlet_P
		v_val = 5 - ((32767 - rawValue) * 0.000153);
		val = (((v_val / 5.0) - 0.1) * (press_Pmax - press_Pmin) / 0.8);
		val = 6.89475729 * val;
		break;
	case PLATFORM_ADS112C_TEMP_RACK:
		val = (rawValue - 15664) * 0.015873;
		val = 1.0678 * val - 5.8373;
		val = 1.0031 * val + 0.1566;
		break;
	case PLATFORM_ADS112C_TEMP_RPU: //CDU_Inlet_Liq_T
		val = (rawValue - 15888) * 0.015873;
		val = 1.0678 * val - 5.8373;
		val = 1.0031 * val + 0.1566;
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

/**
 * return fan board power cable present status
 * 1: present, 0: not present
 */
static uint16_t get_fb_pwr_prsnt(void)
{
	/* worst case assume all fan boards present */
	uint16_t prsnt = 0x3fff;

	/**
	 * since the fan board present status is in the pdb io expander,
	 * use pdb's sensor to switch mutex and i2c mux
	 */
	sensor_cfg *cfg = get_common_sensor_cfg_info(SENSOR_NUM_PDB_HDC1080DMBR_TEMP_C);
	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config info in get_fb_pwr_prsnt");
		return prsnt;
	}

	if (pre_PCA9546A_read(cfg, cfg->pre_sensor_read_args) == false) {
		LOG_ERR("Failed to set mux channel in get_fb_pwr_prsnt");
		return prsnt;
	}

	/* read the fan board present status */
	const uint8_t i2c_retry = 3;
	uint8_t input_reg = 0;
	I2C_MSG msg = construct_i2c_message(I2C_BUS9, FB_PWR_PRSNT_ADDR_EVEN >> 1, 1, &input_reg,
					    1); // read even io expander
	if (i2c_master_read(&msg, i2c_retry)) {
		LOG_ERR("Failed to read pdb io expander %x", FB_PWR_PRSNT_ADDR_EVEN);
		goto exit;
	}

	for (uint8_t i = 0; i < FB_PRSNT_NUM_PER_IOEXP; i++)
		WRITE_BIT(prsnt, i * 2 + 1, !((msg.data[0] >> i) & 0x01));

	msg = construct_i2c_message(I2C_BUS9, FB_PWR_PRSNT_ADDR_ODD >> 1, 1, &input_reg,
				    1); // read odd io expander
	if (i2c_master_read(&msg, i2c_retry)) {
		LOG_ERR("Failed to read pdb io expander %x", FB_PWR_PRSNT_ADDR_ODD);
		goto exit;
	}

	for (uint8_t i = 0; i < FB_PRSNT_NUM_PER_IOEXP; i++)
		WRITE_BIT(prsnt, i * 2, !((msg.data[0] >> i) & 0x01));

exit:
	if (post_PCA9546A_read(cfg, cfg->pre_sensor_read_args, NULL) == false)
		LOG_ERR("Failed to set mux channel in get_fb_pwr_prsnt");

	return prsnt;
}

/*
 * return fan board signal cable present status
 * 1: present, 0: not present
 */
static uint16_t get_fb_sig_prsnt(void)
{
	/* worst case assume all fan boards present */
	uint16_t prsnt = 0x3fff;

	/* read the fan board present status */
	const uint8_t i2c_retry = 3;
	uint8_t input_reg = 0;
	I2C_MSG msg = construct_i2c_message(I2C_BUS4, FB_SIG_PRSNT_ADDR_EVEN >> 1, 1, &input_reg,
					    1); // read even io expander
	if (i2c_master_read(&msg, i2c_retry)) {
		LOG_ERR("Failed to read backplane io expander %x", FB_SIG_PRSNT_ADDR_EVEN);
		goto exit;
	}

	for (uint8_t i = 0; i < FB_PRSNT_NUM_PER_IOEXP; i++)
		WRITE_BIT(prsnt, i * 2 + 1, !((msg.data[0] >> i) & 0x01));

	msg = construct_i2c_message(I2C_BUS4, FB_SIG_PRSNT_ADDR_ODD >> 1, 1, &input_reg,
				    1); // read odd io expander
	if (i2c_master_read(&msg, i2c_retry)) {
		LOG_ERR("Failed to read backplane io expander %x", FB_SIG_PRSNT_ADDR_ODD);
		goto exit;
	}

	for (uint8_t i = 0; i < FB_PRSNT_NUM_PER_IOEXP; i++)
		WRITE_BIT(prsnt, i * 2, !((msg.data[0] >> i) & 0x01));

exit:
	return prsnt;
}

bool get_fb_present_status(uint16_t *fb_present_status)
{
	CHECK_NULL_ARG_WITH_RETURN(fb_present_status, false);

	const uint16_t pwr_prsnt = get_fb_pwr_prsnt();
	const uint16_t sig_prsnt = get_fb_sig_prsnt();

	*fb_present_status = pwr_prsnt & sig_prsnt;

	return true;
}

void clean_flow_cache_data()
{
	for (uint8_t i = 0; i < FLOW_SENSOR_CACHE_MAX_NUM; i++) {
		flow_cache_data[i].is_record = false;
		flow_cache_data[i].is_newest = false;
		flow_cache_data[i].flow_val = 0.0;
	}
}

max11617_init_arg max11617_init_args[] = {
	[0] = {
		.is_init = false,
		.mode = DIFFERENTIAL_MODE,
		.setup_byte = 0x80,
		.config_byte = 0x66,
		.scalefactor[0] = 1,
	},
};

#if 0
static uint8_t get_fb_index(uint8_t sen_num)
{
	sensor_cfg *cfg = get_common_sensor_cfg_info(sen_num);
	if (cfg == NULL) {
		LOG_ERR("Failed to get sensor config info in update_fb_present_status");
		return 0xff;
	}

	/* mapping the fan board index by i2c mux info */
	uint8_t index = (cfg->pre_sensor_read_args == bus_1_PCA9546A_configs)	    ? 0 :
			(cfg->pre_sensor_read_args == (bus_1_PCA9546A_configs + 1)) ? 1 :
			(cfg->pre_sensor_read_args == (bus_1_PCA9546A_configs + 2)) ? 2 :
			(cfg->pre_sensor_read_args == (bus_1_PCA9546A_configs + 3)) ? 3 :
			(cfg->pre_sensor_read_args == bus_2_PCA9546A_configs)	    ? 4 :
			(cfg->pre_sensor_read_args == (bus_2_PCA9546A_configs + 1)) ? 5 :
			(cfg->pre_sensor_read_args == (bus_2_PCA9546A_configs + 2)) ? 6 :
			(cfg->pre_sensor_read_args == (bus_2_PCA9546A_configs + 3)) ? 7 :
			(cfg->pre_sensor_read_args == bus_6_PCA9546A_configs)	    ? 8 :
			(cfg->pre_sensor_read_args == (bus_6_PCA9546A_configs + 1)) ? 9 :
			(cfg->pre_sensor_read_args == (bus_6_PCA9546A_configs + 2)) ? 10 :
			(cfg->pre_sensor_read_args == (bus_6_PCA9546A_configs + 3)) ? 11 :
			(cfg->pre_sensor_read_args == bus_7_PCA9546A_configs)	    ? 12 :
			(cfg->pre_sensor_read_args == (bus_7_PCA9546A_configs + 1)) ? 13 :
			(cfg->pre_sensor_read_args == (bus_7_PCA9546A_configs + 2)) ? 14 :
			(cfg->pre_sensor_read_args == (bus_7_PCA9546A_configs + 3)) ? 15 :
										      0xff;

	return index;
}
#endif