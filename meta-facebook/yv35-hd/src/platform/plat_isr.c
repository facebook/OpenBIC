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
#include "plat_isr.h"
#include "plat_gpio.h"
#include "plat_sensor_table.h"
#include "libipmi.h"
#include "power_status.h"
#include "ipmi.h"
#include "plat_class.h"
#include "plat_i2c.h"
#include "pmbus.h"
#include "kcs.h"
#include "pcc.h"
#include "libutil.h"
#include "logging/log.h"
#include "apml.h"
#include "plat_def.h"

LOG_MODULE_REGISTER(plat_isr);

void ISR_POST_COMPLETE()
{
	set_post_status(FM_BIOS_POST_CMPLT_BIC_N);
	if (apml_write_byte(I2C_BUS14, SB_TSI_ADDR, SBTSI_HIGH_TEMP_INTEGER_THRESHOLD,
			    TSI_HIGH_TEMP_THRESHOLD)) {
		LOG_ERR("Failed to set TSI high temperature threshold.");
	}
}

static void PROC_FAIL_handler(struct k_work *work)
{
	/* if have not received kcs and post code, add FRB3 event log. */
	if ((get_kcs_ok() == false) && (get_4byte_postcode_ok() == false)) {
		common_addsel_msg_t sel_msg;
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.sensor_number = SENSOR_NUM_PROC_FAIL;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.event_data1 = IPMI_EVENT_OFFSET_PROCESSOR_FRB3;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("Failed to assert FRE3 event log.");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(set_DC_on_5s_work, set_DC_on_delayed_status);
K_WORK_DELAYABLE_DEFINE(set_DC_off_10s_work, set_DC_off_delayed_status);
K_WORK_DELAYABLE_DEFINE(PROC_FAIL_work, PROC_FAIL_handler);
#define DC_ON_5_SECOND 5
#define DC_OFF_10_SECOND 10
#define PROC_FAIL_START_DELAY_SECOND 10
void ISR_DC_ON()
{
	set_DC_status(PWRGD_CPU_LVC3);
	if (get_DC_status() == true) {
		reset_pcc_buffer();
		k_work_schedule(&set_DC_on_5s_work, K_SECONDS(DC_ON_5_SECOND));
		k_work_schedule(&PROC_FAIL_work, K_SECONDS(PROC_FAIL_START_DELAY_SECOND));
		if (k_work_cancel_delayable(&set_DC_off_10s_work) != 0) {
			LOG_ERR("Failed to cancel set dc off delay work.");
		}
		set_DC_off_delayed_status();
	} else {
		if (k_work_cancel_delayable(&PROC_FAIL_work) != 0) {
			LOG_ERR("Failed to cancel proc_fail delay work.");
		}
		reset_kcs_ok();
		reset_4byte_postcode_ok();

		k_work_schedule(&set_DC_off_10s_work, K_SECONDS(DC_OFF_10_SECOND));

		if (k_work_cancel_delayable(&set_DC_on_5s_work) != 0) {
			LOG_ERR("Failed to cancel set dc on delay work.");
		}
		set_DC_on_delayed_status();

		if ((gpio_get(FM_CPU_BIC_SLP_S3_N) == GPIO_HIGH) &&
		    (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH)) {
			common_addsel_msg_t sel_msg;
			sel_msg.InF_target = BMC_IPMB;
			sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_OEM_C3;
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			sel_msg.sensor_number = SENSOR_NUM_POWER_ERROR;
			sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PWROK_FAIL;
			sel_msg.event_data2 = 0xFF;
			sel_msg.event_data3 = 0xFF;
			if (!common_add_sel_evt_record(&sel_msg)) {
				LOG_ERR("Failed to add system PWROK failure sel");
			}
		}
	}
}

static void SLP3_handler()
{
	common_addsel_msg_t sel_msg;
	if ((gpio_get(FM_CPU_BIC_SLP_S3_N) == GPIO_HIGH) &&
	    (gpio_get(PWRGD_CPU_LVC3) == GPIO_LOW)) {
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_VRWATCHDOG;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("Failed to add VR watchdog timeout sel.");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(SLP3_work, SLP3_handler);
void ISR_SLP3()
{
	if (gpio_get(FM_CPU_BIC_SLP_S3_N) == GPIO_HIGH) {
		LOG_ERR("slp3");
		k_work_schedule(&SLP3_work, K_MSEC(10000));
		return;
	} else {
		if (k_work_cancel_delayable(&SLP3_work) != 0) {
			LOG_ERR("Failed to cancel delayable work.");
		}
	}
}

void ISR_DBP_PRSNT()
{
	common_addsel_msg_t sel_msg;
	gpio_set(BIC_JTAG_SEL_R, gpio_get(FM_DBP_PRESENT_N));
	if ((gpio_get(FM_DBP_PRESENT_N) == GPIO_HIGH)) {
		sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
	} else {
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
	}
	sel_msg.InF_target = BMC_IPMB;
	sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_HDT;
	sel_msg.sensor_number = SENSOR_NUM_HDT_PRESENT;
	sel_msg.event_data1 = 0xFF;
	sel_msg.event_data2 = 0xFF;
	sel_msg.event_data3 = 0xFF;
	if (!common_add_sel_evt_record(&sel_msg)) {
		LOG_ERR("Failed to add HDT present sel.");
	}
}

void ISR_HSC_THROTTLE()
{
	common_addsel_msg_t sel_msg;
	static bool is_hsc_throttle_assert = false; // Flag for filt out fake alert
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		if ((gpio_get(PWRGD_CPU_LVC3) == GPIO_LOW) &&
		    (get_DC_off_delayed_status() == false)) {
			return;
		} else {
			if ((gpio_get(IRQ_HSC_ALERT1_N) == GPIO_HIGH) &&
			    (is_hsc_throttle_assert == true)) {
				sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
				is_hsc_throttle_assert = false;
			} else if ((gpio_get(IRQ_HSC_ALERT1_N) == GPIO_LOW) &&
				   (is_hsc_throttle_assert == false)) {
				sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
				is_hsc_throttle_assert = true;
			} else { // Fake alert
				return;
			}

			sel_msg.InF_target = BMC_IPMB;
			sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
			sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
			sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PMBUSALERT;
			sel_msg.event_data2 = 0xFF;
			sel_msg.event_data3 = 0xFF;
			if (!common_add_sel_evt_record(&sel_msg)) {
				LOG_ERR("Failed to add HSC Throttle sel.");
			}
		}
	}
}

void ISR_MB_THROTTLE()
{
	common_addsel_msg_t sel_msg;
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH && get_DC_status()) {
		if (gpio_get(FAST_PROCHOT_N) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		}
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_FIRMWAREASSERT;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("Failed to add MB Throttle sel.\n");
		}
	}
}

void ISR_SOC_THMALTRIP()
{
	common_addsel_msg_t sel_msg;
	if (gpio_get(RST_PLTRST_BIC_N) == GPIO_HIGH) {
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_THERMAL_TRIP;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("Failed to add SOC Thermal trip sel.");
		}
	}
}

void ISR_SYS_THROTTLE()
{
	common_addsel_msg_t sel_msg;
	if ((gpio_get(RST_PLTRST_BIC_N) == GPIO_HIGH) && (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH)) {
		if (gpio_get(FM_CPU_BIC_PROCHOT_LVT3_N) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		}
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_THROTTLE;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("Failed to add System Throttle sel.");
		}
	}
}

void ISR_HSC_OC()
{
	common_addsel_msg_t sel_msg;
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		if (gpio_get(FM_HSC_TIMER) == GPIO_LOW) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		}
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_HSCTIMER;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("Failed to add HSC OC sel.");
		}
	}
}

static void add_vr_ocp_sel(uint8_t gpio_num, uint8_t vr_num)
{
	common_addsel_msg_t sel_msg;
	if (gpio_get(gpio_num) == GPIO_HIGH) {
		sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
	} else {
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
	}
	sel_msg.InF_target = BMC_IPMB;
	sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_VR;
	sel_msg.sensor_number = SENSOR_NUM_VR_OCP;
	sel_msg.event_data1 = vr_num;
	sel_msg.event_data2 = 0xFF;
	sel_msg.event_data3 = 0xFF;
	if (!common_add_sel_evt_record(&sel_msg)) {
		LOG_ERR("Failed to add VR OCP sel.");
	}
}

void ISR_PVDDCR_CPU0_OCP()
{
	if (get_DC_status() == true) {
		add_vr_ocp_sel(PVDDCR_CPU0_BIC_OCP_N, 0);
	}
}

void ISR_PVDDCR_CPU1_OCP()
{
	if (get_DC_status() == true) {
		add_vr_ocp_sel(PVDDCR_CPU1_BIC_OCP_N, 1);
	}
}

void ISR_PVDD11_S3_OCP()
{
	if (get_DC_status() == true) {
		add_vr_ocp_sel(PVDD11_S3_BIC_OCP_N, 2);
	}
}

static void add_vr_pmalert_sel(uint8_t gpio_num, uint8_t vr_addr, uint8_t vr_num)
{
	uint8_t retry = 5;
	I2C_MSG *msg = (I2C_MSG *)malloc(sizeof(I2C_MSG));
	if (msg == NULL) {
		LOG_ERR("Failed to allocate I2C_MSG.");
		return;
	}

	for (int page = 0; page < 2; page++) {
		msg->bus = I2C_BUS5;
		msg->target_addr = vr_addr;
		msg->tx_len = 2;
		msg->data[0] = PMBUS_PAGE;
		msg->data[1] = page;

		if (i2c_master_write(msg, retry)) {
			LOG_ERR("Failed to write page.");
			continue;
		}

		msg->bus = I2C_BUS5;
		msg->target_addr = vr_addr;
		msg->tx_len = 1;
		msg->rx_len = 2;
		msg->data[0] = PMBUS_STATUS_WORD;

		if (i2c_master_read(msg, retry)) {
			LOG_ERR("Failed to read PMBUS_STATUS_WORD.");
			continue;
		}

		common_addsel_msg_t sel_msg;
		if (gpio_get(gpio_num) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		}
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_VR;
		sel_msg.sensor_number = SENSOR_NUM_VR_ALERT;
		sel_msg.event_data1 = (vr_num << 1) | (page & 0x01);
		sel_msg.event_data2 = msg->data[0];
		sel_msg.event_data3 = msg->data[1];
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("Failed to add VR PMALERT sel.");
		}
	}
	SAFE_FREE(msg);
}

void ISR_PVDDCR_CPU0_PMALERT()
{
	if (get_DC_status() == true) {
		uint8_t board_rev = get_board_revision();
		if (board_rev == SYS_BOARD_EVT_BOM2) {
			add_vr_pmalert_sel(PVDDCR_CPU0_PMALERT_N, XDPE19283B_PVDDCR_CPU0_ADDR, 0);
		} else if (board_rev == SYS_BOARD_EVT_BOM3) {
			add_vr_pmalert_sel(PVDDCR_CPU0_PMALERT_N, MP2856GUT_PVDDCR_CPU0_ADDR, 0);
		} else {
			add_vr_pmalert_sel(PVDDCR_CPU0_PMALERT_N, RAA229621_PVDDCR_CPU0_ADDR, 0);
		}
	}
}

void ISR_PVDDCR_CPU1_PMALERT()
{
	if (get_DC_status() == true) {
		uint8_t board_rev = get_board_revision();
		if (board_rev == SYS_BOARD_EVT_BOM2) {
			add_vr_pmalert_sel(PVDDCR_CPU1_PMALERT_N, XDPE19283B_PVDDCR_CPU1_ADDR, 1);
		} else if (board_rev == SYS_BOARD_EVT_BOM3) {
			add_vr_pmalert_sel(PVDDCR_CPU1_PMALERT_N, MP2856GUT_PVDDCR_CPU1_ADDR, 1);
		} else {
			add_vr_pmalert_sel(PVDDCR_CPU1_PMALERT_N, RAA229621_PVDDCR_CPU1_ADDR, 1);
		}
	}
}

void ISR_PVDD11_S3_PMALERT()
{
	if (get_DC_status() == true) {
		uint8_t board_rev = get_board_revision();
		if (board_rev == SYS_BOARD_EVT_BOM2) {
			add_vr_pmalert_sel(PVDD11_S3_PMALERT_N, XDPE19283B_PVDD11_S3_ADDR, 2);
		} else if (board_rev == SYS_BOARD_EVT_BOM3) {
			add_vr_pmalert_sel(PVDD11_S3_PMALERT_N, MP2856GUT_PVDD11_S3_ADDR, 2);
		} else {
			add_vr_pmalert_sel(PVDD11_S3_PMALERT_N, RAA229621_PVDD11_S3_ADDR, 2);
		}
	}
}

void ISR_UV_DETECT()
{
	common_addsel_msg_t sel_msg;
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		if (gpio_get(IRQ_UV_DETECT_N) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		}
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_UV;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("Failed to add under voltage sel.");
		}
	}
}
