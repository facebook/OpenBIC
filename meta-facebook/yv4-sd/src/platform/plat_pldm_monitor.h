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

#ifndef PLAT_PLDM_MONITOR_H
#define PLAT_PLDM_MONITOR_H

#define PLAT_PLDM_MAX_STATE_EFFECTER_IDX 170

#define PLAT_PLDM_HOST_PWR_CTRL_DEFAULT 0xFF
#define PLAT_PLDM_HOST_PWR_BTN_LOW 0xFE
#define PLAT_PLDM_HOST_RST_BTN_LOW 0xFD

enum pldm_plat_effecter_id_high_byte {
	PLAT_EFFECTER_ID_GPIO_HIGH_BYTE = (0xFF << 8),
};

enum plat_pldm_effecter_id {
	PLAT_PLDM_EFFECTER_ID_REINIT_I3C_HUB = 0x0101,
};

extern struct pldm_state_effecter_info plat_state_effecter_table[];

void host_power_on();
void host_power_off();
#endif
