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

#ifndef PLAT_EVENT_H
#define PLAT_EVENT_H

#define POLLING_CPLD_STACK_SIZE 4096

#define POWER_AND_RESET_BUTTON_REG 0x00
#define VR_AND_CLK_ENABLE_PIN_READING_REG 0x01
#define VR_ENABLE_PIN_READING_1_REG 0x02
#define VR_ENABLE_PIN_READING_2_REG 0x03
#define VR_ENABLE_PIN_READING_3_REG 0x04
#define VR_ENABLE_PIN_READING_4_REG 0x05
#define MB_POWER_GOOD_AND_PERST_PIN_READING_REG 0x06
#define VR_POWER_GOOD_PIN_READING_1_REG 0x07
#define VR_POWER_GOOD_PIN_READING_2_REG 0x08
#define VR_POWER_GOOD_PIN_READING_3_REG 0x09
#define VR_POWER_GOOD_PIN_READING_4_REG 0x0A
#define VR_POWER_GOOD_PIN_READING_5_REG 0x0B
#define RSVD_1_REG 0x0C
#define VR_POWER_FAULT_1_REG 0x0D
#define VR_POWER_FAULT_2_REG 0x0E
#define VR_POWER_FAULT_3_REG 0x0F
#define VR_POWER_FAULT_4_REG 0x10
#define VR_POWER_FAULT_5_REG 0x11
#define RSVD_2_REG 0x12
#define RSVD_3_REG 0x13
#define OSFP_PRSNT_PIN_READING_1_REG 0x14
#define OSFP_PRSNT_PIN_READING_2_REG 0x15
#define OSFP_POWER_ENABLE_PIN_READING_1_REG 0x16
#define OSFP_POWER_ENABLE_PIN_READING_2_REG 0x17
#define OSFP_POWER_ENABLE_PIN_READING_3_REG 0x18
#define OSFP_POWER_ENABLE_PIN_READING_4_REG 0x19
#define BOARD_TYPE_REG 0x1A
#define BOARD_REV_ID_REG 0x1B
#define VR_VENDOR_TYPE_REG 0x1C
#define OWL_JTAG_SEL_MUX_REG 0x1D
#define ATH_JTAG_SEL_MUX_REG 0x1E
#define OWL_UART_SEL_MUX_REG 0x1F
#define AEGIS_JTAG_SWITCH_REG 0x20
#define ATH_BOOT_SOURCE_REG 0x21
#define S_OWL_BOOT_SOURCE_REG 0x22
#define N_OWL_BOOT_SOURCE_REG 0x23
#define VR_SMBUS_ALERT_1_REG 0x24
#define VR_SMBUS_ALERT_2_REG 0x25
#define RSVD_4_REG 0x26
#define ASIC_OC_WARN_REG 0x27
#define SYSTEM_ALERT_FAULT_REG 0x28
#define VR_HOT_FAULT_1_REG 0x29
#define VR_HOT_FAULT_2_REG 0x2A
#define TEMPERATURE_IC_OVERT_FAULT_REG 0x2B
#define VR_POWER_INPUT_FAULT_1_REG 0x2C
#define VR_POWER_INPUT_FAULT_2_REG 0x2D
#define LEAK_DETCTION_REG 0x2E
#define RESET_PIN_TO_ICS_STATUS_REG 0x2F
#define CRD_STATUS_REG 0x30
#define CMN_STATUS_REG 0x31
#define RSVD_GPIO_STATUS_REG 0x32
#define UART_IC_STATUS_REG 0x33
#define UBC_MODULE_OC_WARNING_REG 0x34
#define CPLD_EEPROM_STATUS_REG 0x35
#define MTIA_N_OWL_TEST_PIN_STATUS_REG 0x36
#define MTIA_S_OWL_TEST_PIN_STATUS_REG 0x37
#define MTIA_ATH_TEST_PIN_STATUS_REG 0x38
#define MTIA_VQPS_TO_EFUSE_PROGRAMMING_REG 0x39
#define BUFFER_100M_CLK_LOSE_OF_INPUT_SIGNAL_REG 0x3A
#define MTIA_QSPI_BOOT_DISABLE_REG 0x3B
#define ATH_RSVD_GPIO_REG 0x3C

void check_ubc_delayed(struct k_timer *timer);
void set_dc_status_changing_status(bool status);
void init_cpld_polling(void);

typedef struct _aegis_cpld_info_ aegis_cpld_info;

typedef struct _aegis_cpld_info_ {
	uint8_t cpld_offset;
	uint8_t dc_off_defaut;
	uint8_t dc_on_defaut;
	bool is_fault_log; // if true, check the value is defaut or not
	uint8_t is_fault_bit_map; //flag for fault

	//flag for 1st polling
	bool is_first_polling;

	//flag for 1st polling after changing DC status
	bool is_first_polling_after_dc_change;

	//temp data for last polling
	uint8_t last_polling_value;

	bool (*status_changed_cb)(aegis_cpld_info *, uint8_t *);

} aegis_cpld_info;

#endif
