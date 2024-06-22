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
#include "plat_pwm.h"
#include <logging/log.h>
#include "plat_class.h"
#include "plat_modbus.h"
#include "plat_util.h"
#include "hal_gpio.h"
#include "plat_threshold.h"
#include "plat_log.h"
#include "plat_gpio.h"
#include "hal_i2c.h"
#include "nct7363.h"
#include "plat_i2c.h"
#include "plat_hook.h"
#include "plat_sensor_table.h"

LOG_MODULE_REGISTER(plat_init);

#define DEF_PROJ_GPIO_PRIORITY 78

SCU_CFG scu_cfg[] = {
	//register    value
	{ 0x7e789108, 0x00000500 }, // uart1,2 RS485 DE/nRE
	{ 0x7e6e24b0, 0x00008000 }, // uart2 NRTS2
	{ 0x7e6e24bc, 0x00000001 }, // uart1 NRTS1
	// disable internal PD
	{ 0x7e6e2610, 0x0FFFBFFF },
	{ 0x7e6e2614, 0x018F0100 },
	{ 0x7e6e2618, 0x0F00FF00 },
	{ 0x7e6e261C, 0xFE300005 },
	{ 0x7e6e2630, 0x00000002 },
};

void pal_pre_init()
{
	scu_init(scu_cfg, sizeof(scu_cfg) / sizeof(SCU_CFG));
	init_aalc_config();
	gpio_set(FM_BIC_READY_R_N, 0); //MM4 for bus3 power up
	k_msleep(10);
	LOG_WRN("pull bpb nct7363 sensor box power high 11");
	sensor_cfg plat_sensor_config[] = {
	{ SENSOR_NUM_BPB_RACK_LEVEL_1, sensor_dev_nct7363, I2C_BUS5, BPB_NCT7363_ADDR,
	  NCT7363_GPIO_READ_OFFSET, stby_access, NCT7363_5_PORT, 0, SAMPLE_COUNT_DEFAULT,
	  POLL_TIME_DEFAULT, ENABLE_SENSOR_POLLING, 0, SENSOR_INIT_STATUS, NULL, NULL, NULL, NULL,
	  &nct7363_init_args[17] }
	};
	uint8_t ret = nct7363_init(&plat_sensor_config[0]);
	printf("init result :0x%x", ret);
	k_msleep(1000);
}

void pal_post_init()
{
	init_load_eeprom_log();
	init_pwm_dev();
	init_custom_modbus_server();
	init_modbus_command_table();
	//threshold_poll_init();
	set_rpu_ready();
	uint8_t retry = 3;
	I2C_MSG msg = { 0 };

	msg.bus = 8;
	msg.target_addr = 0xe8 >> 1;
	msg.tx_len = 1;
	msg.rx_len = 0;
	msg.data[0] = 0x02;

	if (i2c_master_write(&msg, retry)){
		printk("set mux failed\n");
	}

	memset(&msg, 0, sizeof(msg));

	msg.bus = 8;
	msg.target_addr = 0x30 >> 1;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = 0x03;

	if (!i2c_master_read(&msg, retry))
		printk("post init when access sensorboard nct214 ok 74 \n");
	
}

void pal_device_init()
{
	return;
}

void pal_set_sys_status()
{
	return;
}

DEVICE_DEFINE(PRE_DEF_PROJ_GPIO, "PRE_DEF_PROJ_GPIO_NAME", &gpio_init, NULL, NULL, NULL,
	      POST_KERNEL, DEF_PROJ_GPIO_PRIORITY, NULL);
