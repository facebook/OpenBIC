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

#include "plat_modbus.h"
#include "plat_sensor_table.h"
#include <sys/util.h>
#include <drivers/gpio.h>
#include <modbus/modbus.h>
#include <logging/log.h>
#include "sensor.h"
#include "modbus_server.h"

LOG_MODULE_REGISTER(plat_modbus);

struct k_thread modbus_server_thread;
K_KERNEL_STACK_MEMBER(modbus_server_thread_stack, MODBUS_SERVER_THREAD_SIZE);

static char server_iface_name[] = "MODBUS0";
//{DT_PROP(DT_INST(1, zephyr_modbus_serial), label)};
static uint16_t holding_reg[100];
int reading;

modbus_sensor_cfg plat_modbus_sensors[] = {
	/* sensor number,  modbus data addr  */
	{ SENSOR_NUM_TEMP_BB_TMP75, MODBUS_TEMP_BB_TMP75_ADDR },
	{ SENSOR_NUM_TEMP_BPB_TMP75, MODBUS_TEMP_BPB_TMP75_ADDR },
};

static int holding_reg_rd(uint16_t addr, uint16_t *reg)
{
	if (addr >= ARRAY_SIZE(holding_reg)) {
		return -ENOTSUP;
	}

	//*reg = holding_reg[addr];
	int reading = 0;
	uint8_t index = 0;
	if (addr < 0x9200) { // sensor addr is less than 0x9200
		for (index = 0; index < sensor_config_count; index++) {
			if (plat_modbus_sensors[index].data_addr == addr) {
				uint8_t status =
					get_sensor_reading(sensor_config, sensor_config_count,
							   plat_modbus_sensors[index].sensor_num,
							   &reading, GET_FROM_CACHE);
				*reg = reading;
				if (status != SENSOR_READ_SUCCESS) {
					LOG_ERR("Read Modbus Sensor failed");
					return MODBUS_READ_WRITE_REGISTER_FAIL;
				} else {
					return MODBUS_READ_WRITE_REGISTER_SUCCESS;
				}
			}
		}

		*reg = reading;
		LOG_ERR("Wrong Modbus Sensor Addr");
		return MODBUS_ADDR_NOT_DEFINITION;
	} else {
		switch (addr) {
		case MODBUS_POWER_RPU_ADDR:
			break;
		default:
			LOG_ERR("Read Modbus Sensor failed");
			return MODBUS_READ_WRITE_REGISTER_FAIL;
		}

		//printk("Holding register read, addr %u, val %u\n", addr, *reg);

		return MODBUS_READ_WRITE_REGISTER_SUCCESS;
	}
}

static int holding_reg_wr(uint16_t addr, uint16_t reg)
{
	if (addr >= ARRAY_SIZE(holding_reg)) {
		return -ENOTSUP;
	}

	holding_reg[addr] = reg;

	printk("Holding register write, addr %u\n", addr);

	return 0;
}

static struct modbus_user_callbacks mbs_cbs = {
	.holding_reg_rd = holding_reg_rd,
	.holding_reg_wr = holding_reg_wr,
};

const static struct modbus_iface_param server_param = {
	.mode = MODBUS_MODE_RTU,
	.rx_timeout = MODBUS_UART_RESPONSE_T,
	.server = {
		.user_cb = &mbs_cbs,
		.unit_id = MODBUS_UART_NODE_ADDR,
	},
	.serial = {
		.baud = MODBUS_UART_BAUDRATE_HIGH,
		.parity = UART_CFG_PARITY_NONE,
	},
};

static void modbus_server_handler(void *arug0, void *arug1, void *arug2)
{
	int val;

	val = init_modbus_server(*server_iface_name, server_param);
	if (val != 0) {
		LOG_ERR("Failed to initialize server");
		return;
	}
}

//void sensor_poll_init()
void modbus_server_handler_init(void)
{
	k_thread_create(&modbus_server_thread, modbus_server_thread_stack,
			K_THREAD_STACK_SIZEOF(modbus_server_thread_stack), modbus_server_handler,
			NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&modbus_server_thread, "modbus_server_handler");
	return;
}
