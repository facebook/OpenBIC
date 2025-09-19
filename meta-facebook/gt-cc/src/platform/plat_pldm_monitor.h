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
#include "pldm_monitor.h"
#include "plat_sensor_table.h"

#ifndef PLAT_PLDM_MONITOR_H
#define PLAT_PLDM_MONITOR_H

extern uint8_t e1s_prsnt_pin[4][4];
extern uint8_t nic_prsnt_pin[];
extern uint8_t pex_sensor_num_table[];
extern uint8_t e1s_sensor_table[SSD_MAX_NUMBER];
extern uint8_t nic_temp_sensor_table[NIC_MAX_NUMBER];
extern uint8_t nic_optics_sensor_table[NIC_MAX_NUMBER];

#define MAX_STATE_EFFECTER_IDX 187

enum pldm_plat_effecter_id_high_byte {
	PLAT_EFFECTER_ID_GPIO_HIGH_BYTE = (0xFF << 8),
	PLAT_EFFECTER_ID_LED_HIGH_BYTE = (0xE0 << 8),
	PLAT_EFFECTER_ID_NIC_TYPE_HIGH_BYTE = (0xD0 << 8),
};

extern struct pldm_state_effecter_info plat_state_effecter_table[];

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

#define PLDM_PLATFORM_OEM_LED_EFFECTER_STATE_FIELD_COUNT 1
#define PLDM_PLATFORM_OEM_NIC_TYPE_EFFECTER_STATE_FIELD_COUNT 1

enum plat_effecter_states_set_led_value {
	EFFECTER_STATE_LED_VALUE_UNKNOWN = 0x00,
	EFFECTER_STATE_LED_VALUE_ON = 0x01,
	EFFECTER_STATE_LED_VALUE_OFF = 0x02,
	EFFECTER_STATE_LED_VALUE_MAX,
};

enum plat_effecter_states_nic_type_value {
	EFFECTER_STATE_NIC_TYPE_UNKNOWN = 0x00,
	EFFECTER_STATE_NIC_TYPE_CX7 = 0x01,
	EFFECTER_STATE_NIC_TYPE_CX7_IB = 0x02,
	EFFECTER_STATE_NIC_TYPE_THOR2 = 0x03,
	EFFECTER_STATE_NIC_TYPE_POLLARA = 0x04,
	EFFECTER_STATE_NIC_TYPE_MAX,
};

enum plat_pldm_effecter_id {
	PLAT_EFFECTER_ID_POWER_LED = 0x00,
	PLAT_EFFECTER_ID_FAULT_LED = 0x01,
	PLAT_EFFECTER_ID_LED_E1S_0 = 0x10,
	PLAT_EFFECTER_ID_LED_E1S_1 = 0x11,
	PLAT_EFFECTER_ID_LED_E1S_2 = 0x12,
	PLAT_EFFECTER_ID_LED_E1S_3 = 0x13,
	PLAT_EFFECTER_ID_LED_E1S_4 = 0x14,
	PLAT_EFFECTER_ID_LED_E1S_5 = 0x15,
	PLAT_EFFECTER_ID_LED_E1S_6 = 0x16,
	PLAT_EFFECTER_ID_LED_E1S_7 = 0x17,
	PLAT_EFFECTER_ID_LED_E1S_8 = 0x18,
	PLAT_EFFECTER_ID_LED_E1S_9 = 0x19,
	PLAT_EFFECTER_ID_LED_E1S_10 = 0x1A,
	PLAT_EFFECTER_ID_LED_E1S_11 = 0x1B,
	PLAT_EFFECTER_ID_LED_E1S_12 = 0x1C,
	PLAT_EFFECTER_ID_LED_E1S_13 = 0x1D,
	PLAT_EFFECTER_ID_LED_E1S_14 = 0x1E,
	PLAT_EFFECTER_ID_LED_E1S_15 = 0x1F,
	PLAT_EFFECTER_ID_NIC_TYPE = 0x00,
};

void ssd_alert_check(uint8_t group);
void ssd_present_check();
void nic_present_check();
void pal_load_pldm_effcter_table();
void vr_alert_check();
void vr_status_mfr_specific_check_handler();

#endif
