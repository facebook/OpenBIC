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

#ifndef PLAT_CPLD_H
#define PLAT_CPLD_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <zephyr.h>

#define RESET 0x00
#define CPLD_OFFSET_BOARD_REV_ID 0x14
#define CPLD_OFFSET_VR_VENDER_TYPE 0x15
#define CPLD_OFFSET_POWER_CLAMP 0x25
#define CPLD_OFFSET_USERCODE 0x32
#define CPLD_OFFSET_MMC_PWR_EN 0x38
#define CPLD_OFFSET_ASIC_BOARD_ID 0x3C
#define CPLD_OFFSET_ADC_IDX 0xA0
#define CPLD_OFFSET_ASIC_RST_DELAY 0xA4
#define CPLD_OFFSET_MODULE_PG_DELAY 0xA5
#define VR_AND_CLK_EN 0x3E
#define VR_1_EN 0x3F
#define VR_2_EN 0x40
#define VR_3_EN 0x41
#define VR_4_EN 0x42
#define CPLD_HAMSA_PCIE0_PERST_DELAY_REG 0x9D
#define CPLD_HAMSA_PCIE1_PERST_DELAY_REG 0xB3
#define CPLD_HAMSA_PCIE2_PERST_DELAY_REG 0xB4
#define CPLD_HAMSA_PCIE3_PERST_DELAY_REG 0xB5
#define CPLD_POWER_INFO_0_REG 0xB6
#define CPLD_POWER_INFO_1_REG 0xB7
#define PREST_DELAY_REG 0x9D
#define VR_SMBUS_ALERT_EVENT_LOG_REG 0x26
#define HAMSA_MFIO_REG 0x17
#define VR_AND_CLK_EN_PIN_CTRL 0xA1
#define ASIC_VR_HOT_SWITCH 0x12
#define ASIC_JTAG_MUX_SEL 0x39
#define ASIC_VQPS 0x13
#define VR_PWRGD_PIN_READING_1_REG 0x07
#define VR_PWRGD_PIN_READING_2_REG 0x08
#define VR_PWRGD_PIN_READING_3_REG 0x09
#define VR_PWRGD_PIN_READING_4_REG 0x0A
#define VR_PWRGD_PIN_READING_5_REG 0x0B
#define VR_PWRGD_PIN_READING_6_REG 0x0C
#define VR_CLK_ENABLE_PIN_CTRL_REG 0xA1 // pin control (1-step only)
#define VR_1STEP_FUNC_EN_REG 0xA9

#define CPLD_ADDR (0x4C >> 1)
#define I2C_BUS_CPLD I2C_BUS11
typedef struct _cpld_info_ cpld_info;

typedef struct _cpld_info_ {
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

	bool (*status_changed_cb)(cpld_info *, uint8_t *);

	uint8_t bit_check_mask; //bit check mask

	uint8_t event_type;

} cpld_info;

typedef struct {
	const char *name;
	uint8_t bit;
	uint8_t offset;
} cpld_pin_map_t;

void check_ubc_delayed(struct k_work *work);
bool is_ubc_enabled_delayed_enabled(void);
bool plat_read_cpld(uint8_t offset, uint8_t *data, uint8_t len);
bool plat_write_cpld(uint8_t offset, uint8_t *data);
void init_cpld_polling(void);
void check_cpld_polling_alert_status(void);
void check_ubc_delayed_timer_handler(struct k_timer *timer);
bool set_cpld_bit(uint8_t cpld_offset, uint8_t bit, uint8_t value);
void give_all_vr_pm_alert_sem();
void get_cpld_polling_power_info(int *reading);
#endif
