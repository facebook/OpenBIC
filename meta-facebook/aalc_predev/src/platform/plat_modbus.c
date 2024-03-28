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
#include "fru.h"
#include "eeprom.h"
#include "plat_fru.h"

LOG_MODULE_REGISTER(plat_modbus);

struct k_thread modbus_server_thread;
K_KERNEL_STACK_MEMBER(modbus_server_thread_stack, MODBUS_SERVER_THREAD_SIZE);

static char server_iface_name[] = "MODBUS0";
//{DT_PROP(DT_INST(1, zephyr_modbus_serial), label)};
static uint16_t regs_wr[16];
static uint16_t regs_rd[16];
static uint8_t cb_num;
static uint16_t cb_addr[2];

modbus_sensor_cfg plat_modbus_sensors[] = {
	/* sensor number,  modbus data addr  */
	{ SENSOR_NUM_BB_TMP75_TEMP_C, MODBUS_TEMP_BB_TMP75_ADDR },
	{ SENSOR_NUM_BPB_RPU_OUTLET_TEMP_C, MODBUS_TEMP_BPB_TMP75_ADDR },
};

static int holding_reg_rd(uint16_t addr, uint16_t *reg)
{
	if (cb_num > 0 && (addr > cb_addr[0] && addr <= cb_addr[1])) {
		*reg = regs_rd[cb_num - 1];
		cb_num--;
		return MODBUS_READ_WRITE_REGISTER_SUCCESS;
	}

	cb_num = 0;
	memset(&cb_addr, 0, sizeof(cb_addr));
	memset(&regs_rd, 0, sizeof(regs_rd));

	if (addr < 0x9200) { // sensor addr is less than 0x9200
		int reading = 0;
		uint8_t index = 0;
		for (index = 0; index < sensor_config_count; index++) {
			if (plat_modbus_sensors[index].data_addr == addr) {
				uint8_t status =
					get_sensor_reading(sensor_config, sensor_config_count,
							   plat_modbus_sensors[index].sensor_num,
							   &reading, GET_FROM_CACHE);

				if (status ==
				    SENSOR_READ_SUCCESS) { //reading type is int(4Bytes = 2 registers)
					*reg = reading & 0x0000FFFF;
					cb_num = sizeof(reading) - 1;
					cb_addr[1] = addr + cb_num;
					cb_addr[0] = addr;
					regs_rd[0] = (reading >> 8) >> 8;
					return MODBUS_READ_WRITE_REGISTER_SUCCESS;
				} else {
					LOG_ERR("Read Modbus Sensor failed");
					return MODBUS_READ_WRITE_REGISTER_FAIL;
				}
			}
		}

		LOG_ERR("Wrong Modbus Sensor Addr");
		return MODBUS_ADDR_NOT_DEFINITION;
	} else {
		uint8_t status = 0;
		switch (addr) {
		case FRU_FB_PART_ADDR:
		case FRU_MFR_MODEL_ADDR:
		case FRU_MFR_DATE_ADDR:
		case FRU_MFR_SERIEL_ADDR:
		case MODBUS_POWER_RPU_ADDR:
			status = modbus_read_fruid_data(regs_rd, addr, cb_num);
			if (status == FRU_READ_SUCCESS) {
				*reg = regs_rd[cb_num - 1];
				cb_num--;
				cb_addr[1] = addr + cb_num;
				cb_addr[0] = addr;
				return MODBUS_READ_WRITE_REGISTER_SUCCESS;
			} else {
				memset(&regs_rd, 0, sizeof(regs_rd));
				cb_num = 0;
				return MODBUS_READ_WRITE_REGISTER_FAIL;
			}
		default:
			LOG_ERR("Read Modbus Sensor failed");
			return MODBUS_READ_WRITE_REGISTER_FAIL;
		}

		//printk("Holding register read, addr %u, val %u\n", addr, *reg);

		return MODBUS_READ_WRITE_REGISTER_SUCCESS;
	}

	LOG_ERR("Wrong Modbus Sensor Addr");
	return MODBUS_ADDR_NOT_DEFINITION;
}

static int holding_reg_wr(uint16_t addr, uint16_t reg)
{
	uint8_t status = 0;
	if (cb_num > 0 && (addr > cb_addr[0] && addr <= cb_addr[1])) {
		regs_wr[cb_num - 1] = reg;
		cb_num--;
		if (cb_num == 0) {
			status = modbus_write_fruid_data(regs_wr, addr);
			if (status == FRU_WRITE_SUCCESS) {
				return MODBUS_READ_WRITE_REGISTER_SUCCESS;
			} else {
				return MODBUS_READ_WRITE_REGISTER_FAIL;
			}
		}
		return MODBUS_READ_WRITE_REGISTER_SUCCESS;
	}

	cb_num = 0;
	memset(&cb_addr, 0, sizeof(cb_addr));
	memset(&regs_rd, 0, sizeof(regs_rd));	

	switch (addr) {
	case FRU_FB_PART_ADDR:
	case FRU_MFR_MODEL_ADDR:
	case FRU_MFR_DATE_ADDR:
	case FRU_MFR_SERIEL_ADDR:
	case MODBUS_POWER_RPU_ADDR:
		cb_num = fru_field_datasize(addr);
		if (cb_num > 0) {
			regs_wr[cb_num - 1] = reg;
			cb_num--;
			cb_addr[1] = addr + cb_num;
			cb_addr[0] = addr;			
			return MODBUS_READ_WRITE_REGISTER_SUCCESS;
		} else {
			cb_num = 0;
			return MODBUS_READ_WRITE_REGISTER_FAIL;
		}

	default:
		LOG_ERR("Read Modbus Sensor failed");
		return MODBUS_READ_WRITE_REGISTER_FAIL;
	}

	LOG_ERR("Wrong Modbus Sensor Addr");
	return MODBUS_ADDR_NOT_DEFINITION;
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
