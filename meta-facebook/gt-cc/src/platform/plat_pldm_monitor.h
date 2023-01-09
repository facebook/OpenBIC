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

extern uint8_t e1s_prsnt_pin[4][4];
extern uint8_t nic_prsnt_pin[];
extern uint8_t pex_sensor_num_table[];

enum plat_pldm_event_sensor_num {
	// BIC
	PLDM_EVENT_SENSOR_BIC = 0xFF,
	// NIC_0 - NIC_7, NIC_0_7
	PLDM_EVENT_SENSOR_NIC_0 = 0x10,
	PLDM_EVENT_SENSOR_NIC_1 = 0x11,
	PLDM_EVENT_SENSOR_NIC_2 = 0x12,
	PLDM_EVENT_SENSOR_NIC_3 = 0x13,
	PLDM_EVENT_SENSOR_NIC_4 = 0x14,
	PLDM_EVENT_SENSOR_NIC_5 = 0x15,
	PLDM_EVENT_SENSOR_NIC_6 = 0x16,
	PLDM_EVENT_SENSOR_NIC_7 = 0x17,
	PLDM_EVENT_SENSOR_NIC_0_7 = 0x18,
	/* E1S_0 - E1S_15 */
	PLDM_EVENT_SENSOR_E1S_0 = 0x20,
	PLDM_EVENT_SENSOR_E1S_1 = 0x21,
	PLDM_EVENT_SENSOR_E1S_2 = 0x22,
	PLDM_EVENT_SENSOR_E1S_3 = 0x23,
	PLDM_EVENT_SENSOR_E1S_4 = 0x24,
	PLDM_EVENT_SENSOR_E1S_5 = 0x25,
	PLDM_EVENT_SENSOR_E1S_6 = 0x26,
	PLDM_EVENT_SENSOR_E1S_7 = 0x27,
	PLDM_EVENT_SENSOR_E1S_8 = 0x28,
	PLDM_EVENT_SENSOR_E1S_9 = 0x29,
	PLDM_EVENT_SENSOR_E1S_10 = 0x2A,
	PLDM_EVENT_SENSOR_E1S_11 = 0x2B,
	PLDM_EVENT_SENSOR_E1S_12 = 0x2C,
	PLDM_EVENT_SENSOR_E1S_13 = 0x2D,
	PLDM_EVENT_SENSOR_E1S_14 = 0x2E,
	PLDM_EVENT_SENSOR_E1S_15 = 0x2F,
	/* PEX0 - PEX1, PEX */
	PLDM_EVENT_SENSOR_PEX_0 = 0x40,
	PLDM_EVENT_SENSOR_PEX_1 = 0x41,
	PLDM_EVENT_SENSOR_PEX_2 = 0x42,
	PLDM_EVENT_SENSOR_PEX_3 = 0x43,
	PLDM_EVENT_SENSOR_PEX = 0x44,
	/* VR0 - VR1, VR */
	PLDM_EVENT_SENSOR_VR_0 = 0x50,
	PLDM_EVENT_SENSOR_VR_1 = 0x51,
	PLDM_EVENT_SENSOR_VR = 0x52,
	/* CPLD */
	PLDM_EVENT_SENSOR_CPLD = 0x60,
	/* HSC */
	PLDM_EVENT_SENSOR_HSC = 0x62,
};

enum plat_pldm_bic_state_set_offset {
	PLDM_STATE_SET_OFFSET_BIC_BOOT_RESTART_CAUSE = 0,
};

enum plat_pldm_device_state_set_offset {
	PLDM_STATE_SET_OFFSET_DEVICE_PRESENCE = 0,
	PLDM_STATE_SET_OFFSET_DEVICE_STATUS = 1,
};

enum plat_pldm_state_set_device_presense {
	PLDM_STATE_SET_DEVICE_PRESENT = 1,
	PLDM_STATE_SET_DEVICE_NOT_PRESENT = 2,
	PLDM_STATE_SET_DEVICE_PLUG_IN = 3,
	PLDM_STATE_SET_DEVICE_PLUG_OUT = 4,
};

void ssd_alert_check(uint8_t group);
void ssd_present_check();
void nic_present_check();

#endif