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

#ifndef PLAT_HOOK_H
#define PLAT_HOOK_H

typedef struct _i2c_proc_arg {
	uint8_t bus;
	uint8_t channel;
} i2c_proc_arg;

/**************************************************************************************************
 * INIT ARGS
**************************************************************************************************/
extern adc_asd_init_arg adc_asd_init_args[];
extern ina233_init_arg ina233_init_args[];
extern sq52205_init_arg sq52205_init_args[];
extern i2c_proc_arg i2c_proc_args[];
extern pt5161l_init_arg pt5161l_init_args[];
extern struct k_mutex i2c_hub_mutex;

#define I2C_HUB_MUTEX_TIMEOUT_MS 300
#define I2C_HUB_CHANNEL_0 0x01
#define I2C_HUB_CHANNEL_1 0x02
#define I2C_HUB_CHANNEL_2 0x04
#define I2C_HUB_CHANNEL_3 0x08
#define I2C_HUB_CHANNEL_4 0x10
#define I2C_HUB_CHANNEL_5 0x20

bool pre_i2c_bus_read(uint8_t sensor_num, void *args);
bool post_i2c_bus_read(uint8_t sensor_num, void *args, int *reading);
bool pre_retimer_read(uint8_t sensor_num, void *args);

#endif
