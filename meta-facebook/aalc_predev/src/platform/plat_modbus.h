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

#include <stdbool.h>
#include <stdint.h>
#include <kernel.h>

#define MODBUS_SERVER_THREAD_SIZE 3056

#define MODBUS_UART_NODE_ADDR 0x0C
#define MODBUS_UART_FP_OFFSET 5000

void modbus_server_handler_init(void);

typedef struct _modbus_sensor_cfg {
	uint8_t sensor_num;
	uint16_t data_addr;
} modbus_sensor_cfg;
//sensor_monitor_table_info

/* define modbus data address */
#define MODBUS_TEMP_BB_TMP75_ADDR 0x0101
#define MODBUS_TEMP_BPB_TMP75_ADDR 0x0102
#define MODBUS_POWER_RPU_ADDR 0x9999
