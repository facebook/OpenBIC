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
typedef struct _modbus_command_mapping {
	uint16_t addr;
	uint8_t (*wr_fn)(struct _modbus_command_mapping *);
	uint8_t (*rd_fn)(struct _modbus_command_mapping *);
	uint8_t arg0;
	uint8_t arg1;
	uint8_t size;
	uint16_t *data;
} modbus_command_mapping;

/* define modbus data address */
#define MODBUS_TEMP_BB_TMP75_ADDR 0x0101
#define MODBUS_TEMP_BPB_TMP75_ADDR 0x0102
#define MODBUS_POWER_RPU_ADDR 0x9999

//FRU Info
#define FRU_FB_PART_ADDR 0x19C4
#define FRU_MFR_MODEL_ADDR 0x19CC
#define FRU_MFR_DATE_ADDR 0x19D4
#define FRU_MFR_SERIEL_ADDR 0x19D8
#define FRU_WORKORDER_ADDR 0x19E0
#define FRU_HW_REVISION_ADDR 0x19E4
#define FRU_FW_REVISION_ADDR 0x19E8
#define FRU_TOTAL_UP_TIME_ADDR 0x19EC
#define FRU_LAST_ON_TIME_ADDR 0x19EF
#define FRU_HMI_REVISION_ADDR 0x19F2
#define FRU_NOAH_ARK_CONFIG_ADDR 0x19F8
#define FRU_HEATEXCHANGER_CONTROLBOX_FPBN_ADDR 0x1A00
#define FRU_QUANTA_FB_PART_ADDR 0x1A0C
