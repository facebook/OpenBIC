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

#ifndef PLAT_DEV
#define PLAT_DEV

#include <stdint.h>
#include "sensor.h"
#include "cci.h"

typedef struct _pm8702_dev_info {
	bool is_init;
	cci_fw_info_resp dev_info;
} pm8702_dev_info;

extern pm8702_dev_info pm8702_table[];

bool pal_sensor_drive_init(uint8_t card_id, sensor_cfg *cfg, uint8_t *init_status);
bool pal_sensor_drive_read(uint8_t card_id, sensor_cfg *cfg, int *reading, uint8_t *sensor_status);
uint8_t pal_tmp75_init(uint8_t card_id, sensor_cfg *cfg);
uint8_t pal_tmp75_read(uint8_t card_id, sensor_cfg *cfg, int *reading);
uint8_t pal_emc1412_init(uint8_t card_id, sensor_cfg *cfg);
uint8_t pal_emc1412_read(uint8_t card_id, sensor_cfg *cfg, int *reading);
uint8_t pal_nvme_init(uint8_t card_id, sensor_cfg *cfg);
uint8_t pal_nvme_read(uint8_t card_id, sensor_cfg *cfg, int *reading);
uint8_t pal_ina233_init(uint8_t card_id, sensor_cfg *cfg);
uint8_t pal_ina233_read(uint8_t card_id, sensor_cfg *cfg, int *reading);
uint8_t pal_ltc2991_init(uint8_t card_id, sensor_cfg *cfg);
uint8_t pal_ltc2991_read(uint8_t card_id, sensor_cfg *cfg, int *reading);
uint8_t pal_xdpe12284c_init(uint8_t card_id, sensor_cfg *cfg);
uint8_t pal_xdpe12284c_read(uint8_t card_id, sensor_cfg *cfg, int *reading);
bool cxl_single_ioexp_init(uint8_t ioexp_name);
int cxl_ioexp_init(uint8_t cxl_channel);
uint8_t pal_pm8702_read(uint8_t card_id, sensor_cfg *cfg, int *reading);
uint8_t pal_pm8702_init(uint8_t card_id, sensor_cfg *cfg);
void cxl_mb_status_init(uint8_t cxl_id);
bool pal_init_pm8702_info(uint8_t cxl_id);
bool pal_pm8702_command_handler(uint8_t pcie_card_id, uint16_t opcode, uint8_t *data_buf,
				int data_len, uint8_t *response, uint8_t *response_len);
bool pal_get_pm8702_hbo_status(uint8_t pcie_card_id, uint8_t *resp_buf, uint8_t *resp_len);
bool pal_pm8702_transfer_fw(uint8_t pcie_card_id, uint8_t *req_buf, int req_len);
bool pal_set_pm8702_active_slot(uint8_t pcie_card_id, uint8_t *req_buf, int req_len);

#endif
