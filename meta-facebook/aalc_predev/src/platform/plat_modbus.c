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
#include <stdlib.h>
#include <sys/util.h>
#include <drivers/gpio.h>
#include <modbus/modbus.h>
#include <time.h>
#include <logging/log.h>
#include "sensor.h"
#include "modbus_server.h"
#include "fru.h"
#include "eeprom.h"
#include "libutil.h"

#include "plat_modbus.h"
#include "plat_sensor_table.h"
#include "plat_fru.h"

LOG_MODULE_REGISTER(plat_modbus);

static char server_iface_name[] = "MODBUS0";

struct k_thread modbus_server_thread;
K_KERNEL_STACK_MEMBER(modbus_server_thread_stack, MODBUS_SERVER_THREAD_SIZE);

static uint8_t modbus_get_senser_reading(modbus_command_mapping *cmd)
{
	CHECK_NULL_ARG_WITH_RETURN(cmd, MODBUS_ARG_NULL);

	int reading = 0;
	uint8_t status = get_sensor_reading(sensor_config, sensor_config_count, cmd->arg0, &reading,
					    GET_FROM_CACHE);

	if (status == SENSOR_READ_SUCCESS) {
		memcpy(&cmd->data, &reading, sizeof(uint16_t) * cmd->size);
		return MODBUS_READ_WRITE_REGISTER_SUCCESS;
	}

	return MODBUS_READ_WRITE_REGISTER_FAIL;
}

modbus_command_mapping modbus_command_table[] = {
	{ MODBUS_TEMP_BB_TMP75_ADDR, NULL, modbus_get_senser_reading, SENSOR_NUM_BB_TMP75_TEMP_C, 0,
	  1 },
	{ MODBUS_TEMP_BPB_TMP75_ADDR, NULL, modbus_get_senser_reading,
	  SENSOR_NUM_BPB_RPU_OUTLET_TEMP_C, 0, 1 },
};

static modbus_command_mapping *ptr_to_modbus_table(uint16_t addr)
{
	for (uint8_t i = 0; i < ARRAY_SIZE(modbus_command_table); i++) {
		if (addr >= modbus_command_table[i].addr &&
		    addr < (modbus_command_table[i].addr + modbus_command_table[i].size))
			return &modbus_command_table[i];
	}

	return NULL;
}

void init_modbus_command_table(void)
{
	for (uint16_t i = 0; i < ARRAY_SIZE(modbus_command_table); i++) {
		if (modbus_command_table[i].data)
			SAFE_FREE(modbus_command_table[i].data);

		modbus_command_table[i].data =
			(uint16_t *)malloc(modbus_command_table[i].size * sizeof(uint16_t));
		if (modbus_command_table[i].data == NULL) {
			LOG_ERR("modbus_command_mapping[%i] malloc fail", i);
			return;
		}
	}
}

static int holding_reg_wr(uint16_t addr, uint16_t reg)
{
	modbus_command_mapping *ptr = ptr_to_modbus_table(addr);
	if (!ptr) {
		LOG_ERR("modbus write command 0x%x not find!\n", addr);
		return MODBUS_ADDR_NOT_DEFINITION;
	}

	int ret = MODBUS_READ_WRITE_REGISTER_SUCCESS;
	uint8_t offset = addr - ptr->addr;

	ptr->data[offset] = reg;

	if (offset == (ptr->size - 1) && ptr->wr_fn)
		ret = ptr->wr_fn(ptr);

	return ret;
}

static int holding_reg_rd(uint16_t addr, uint16_t *reg)
{
	CHECK_NULL_ARG_WITH_RETURN(reg, MODBUS_ARG_NULL);

	modbus_command_mapping *ptr = ptr_to_modbus_table(addr);
	if (!ptr) {
		LOG_ERR("modbus read command 0x%x not find!\n", addr);
		return MODBUS_ADDR_NOT_DEFINITION;
	}

	int ret = MODBUS_READ_WRITE_REGISTER_SUCCESS;
	uint8_t offset = addr - ptr->addr;

	if (offset == 0 && ptr->rd_fn)
		ret = ptr->rd_fn(ptr);

	*reg = ptr->data[offset];
	return ret;
}

static struct modbus_user_callbacks mbs_cbs = {
	.holding_reg_rd = holding_reg_rd,
	.holding_reg_wr = holding_reg_wr,
};

const static struct modbus_iface_param server_param = {
	.mode = MODBUS_MODE_RTU,
	.server = {
		.user_cb = &mbs_cbs,
		.unit_id = MODBUS_UART_NODE_ADDR,
	},
	.serial = {
		.baud = MODBUS_UART_BAUDRATE_LOW,
		.parity = MODBUS_UART_PARITY,
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
