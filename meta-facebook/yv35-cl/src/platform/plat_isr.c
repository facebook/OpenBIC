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

#include "plat_isr.h"
#include <logging/log.h>
#include "libipmi.h"
#include "kcs.h"
#include "power_status.h"
#include "sensor.h"
#include "snoop.h"
#include "plat_gpio.h"
#include "plat_class.h"
#include "plat_ipmi.h"
#include "plat_sensor_table.h"
#include "plat_i2c.h"
#include "plat_pmic.h"
#include "plat_dimm.h"
#include "oem_1s_handler.h"
#include "hal_gpio.h"
#include "hal_i2c.h"
#include "util_sys.h"
#include "util_worker.h"
#include "plat_mctp.h"
#include "plat_power.h"

LOG_MODULE_REGISTER(plat_isr);

static bool is_smi_assert = false;

uint8_t _1ou_m2_name_mapping_table[4] = {
	0x3A,
	0x3C,
	0x3E,
	0x37,
};

void send_gpio_interrupt(uint8_t gpio_num)
{
	ipmi_msg msg = { 0 };
	uint8_t gpio_val = gpio_get(gpio_num);
	int ret = 0;

	LOG_INF("Send gpio interrupt to BMC, gpio number(%d) status(%d)", gpio_num, gpio_val);

	msg.data_len = 5;
	msg.InF_source = SELF;
	msg.InF_target = PLDM;
	msg.netfn = NETFN_OEM_1S_REQ;
	msg.cmd = CMD_OEM_1S_SEND_INTERRUPT_TO_BMC;

	msg.data[0] = IANA_ID & 0xFF;
	msg.data[1] = (IANA_ID >> 8) & 0xFF;
	msg.data[2] = (IANA_ID >> 16) & 0xFF;
	msg.data[3] = gpio_num;
	msg.data[4] = gpio_val;

	ret = pldm_send_ipmi_request(&msg);
	if (ret < 0) {
		LOG_ERR("Failed to send GPIO interrupt event to BMC, gpio number(%d) ret(%d)",
			gpio_num, ret);
	}
}

static void SLP3_handler()
{
	common_addsel_msg_t sel_msg;
	if ((gpio_get(FM_SLPS3_PLD_N) == GPIO_HIGH) && (gpio_get(PWRGD_SYS_PWROK) == GPIO_LOW)) {
		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_VRWATCHDOG;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			LOG_ERR("VR watchdog timeout addsel fail");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(SLP3_work, SLP3_handler);
#define DETECT_VR_WDT_DELAY_S 10
void ISR_SLP3()
{
	if (gpio_get(FM_SLPS3_PLD_N) == GPIO_HIGH) {
		LOG_DBG("slp3");
		k_work_schedule_for_queue(&plat_work_q, &SLP3_work,
					  K_SECONDS(DETECT_VR_WDT_DELAY_S));
		return;
	}
	if (k_work_cancel_delayable(&SLP3_work) != 0) {
		LOG_ERR("Failed to cancel delayable work");
	}
}

void ISR_POST_COMPLETE()
{
	if (gpio_get(FM_BIOS_POST_CMPLT_BMC_N) == GPIO_LOW) { // Post complete
		if (get_me_mode() == ME_INIT_MODE) {
			init_me_firmware();
		}
	}

	set_post_status(FM_BIOS_POST_CMPLT_BMC_N);
}

K_WORK_DELAYABLE_DEFINE(set_DC_on_5s_work, set_DC_on_delayed_status);
K_WORK_DELAYABLE_DEFINE(read_pmic_critical_work, read_pmic_error_when_dc_off);
#define DC_ON_5_SECOND 5
// The PMIC needs a total of 100ms from CAMP signal assertion to complete the write operation
#define READ_PMIC_CRITICAL_ERROR_MS 100
void ISR_DC_ON()
{
	set_DC_status(PWRGD_SYS_PWROK);

	ipmi_msg msg = { 0 };
	int ret = 0;
	bool dc_status = get_DC_status();

	if (dc_status) {
		k_work_schedule(&set_DC_on_5s_work, K_SECONDS(DC_ON_5_SECOND));

		clear_pmic_error();

	} else {
		set_DC_on_delayed_status();

		// Read PMIC error when DC off
		k_work_schedule(&read_pmic_critical_work, K_MSEC(READ_PMIC_CRITICAL_ERROR_MS));

		if ((gpio_get(FM_SLPS3_PLD_N) == GPIO_HIGH) &&
		    (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH)) {
			common_addsel_msg_t sel_msg;
			sel_msg.InF_target = PLDM;
			sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_OEM_C3;
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			sel_msg.sensor_number = SENSOR_NUM_POWER_ERROR;
			sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PWROK_FAIL;
			sel_msg.event_data2 = 0xFF;
			sel_msg.event_data3 = 0xFF;
			if (!mctp_add_sel_to_ipmi(&sel_msg)) {
				LOG_ERR("System PWROK failure addsel fail");
			}
		}
	}

	msg.data_len = 4;
	msg.InF_source = SELF;
	msg.InF_target = PLDM;
	msg.netfn = NETFN_OEM_1S_REQ;
	msg.cmd = CMD_OEM_1S_SEND_HOST_POWER_STATE_TO_BMC;

	msg.data[0] = IANA_ID & 0xFF;
	msg.data[1] = (IANA_ID >> 8) & 0xFF;
	msg.data[2] = (IANA_ID >> 16) & 0xFF;
	msg.data[3] = dc_status ? GPIO_HIGH : GPIO_LOW;

	ret = pldm_send_ipmi_request(&msg);
	if (ret < 0) {
		LOG_ERR("Failed to send host power state to BMC, gpio number %d ret %d",
			PWRGD_SYS_PWROK, ret);
	}
}

void ISR_BMC_PRDY()
{
	send_gpio_interrupt(H_BMC_PRDY_BUF_N);
}

static void PROC_FAIL_handler(struct k_work *work)
{
	/* if have not received kcs and post code, add FRB3 event log. */
	if ((get_kcs_ok() == false) && (get_postcode_ok() == false)) {
		common_addsel_msg_t sel_msg;
		bool ret = false;

		memset(&sel_msg, 0, sizeof(common_addsel_msg_t));
		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.sensor_number = SENSOR_NUM_PROC_FAIL;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.event_data1 = IPMI_EVENT_OFFSET_PROCESSOR_FRB3;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		ret = mctp_add_sel_to_ipmi(&sel_msg);
		if (!ret) {
			LOG_ERR("Fail to assert FRE3 event log.");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(PROC_FAIL_work, PROC_FAIL_handler);
#define PROC_FAIL_START_DELAY_SECOND 10
void ISR_PWRGD_CPU()
{
	set_CPU_power_status(PWRGD_CPU_LVC3);
	if (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH) {
		init_snoop_thread();
		init_send_postcode_thread();
		/* start thread proc_fail_handler after 10 seconds */
		k_work_schedule_for_queue(&plat_work_q, &PROC_FAIL_work,
					  K_SECONDS(PROC_FAIL_START_DELAY_SECOND));
	} else {
		abort_snoop_thread();

		if (k_work_cancel_delayable(&PROC_FAIL_work) != 0) {
			LOG_ERR("Cancel proc_fail delay work fail");
		}
		reset_kcs_ok();
		reset_postcode_ok();
	}
	send_gpio_interrupt(PWRGD_CPU_LVC3);
}

static void CAT_ERR_handler(struct k_work *work)
{
	if ((gpio_get(RST_PLTRST_BUF_N) == GPIO_HIGH) || (gpio_get(PWRGD_SYS_PWROK) == GPIO_HIGH)) {
		common_addsel_msg_t sel_msg;
		bool ret = false;

		memset(&sel_msg, 0, sizeof(common_addsel_msg_t));
		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.sensor_number = SENSOR_NUM_CATERR;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		/* MCERR: one pulse, IERR: keep low */
		if (gpio_get(FM_CATERR_LVT3_N) == GPIO_HIGH) {
			sel_msg.event_data1 = IPMI_EVENT_OFFSET_PROCESSOR_MCERR;
		} else {
			sel_msg.event_data1 = IPMI_EVENT_OFFSET_PROCESSOR_IERR;
		}
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		ret = mctp_add_sel_to_ipmi(&sel_msg);
		if (!ret) {
			LOG_ERR("Fail to assert CatErr event log.");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(CAT_ERR_work, CAT_ERR_handler);
#define CATERR_START_DELAY_SECOND 2
void ISR_CATERR()
{
	if ((gpio_get(RST_PLTRST_BUF_N) == GPIO_HIGH)) {
		if (k_work_cancel_delayable(&CAT_ERR_work) != 0) {
			LOG_ERR("Cancel caterr delay work fail");
		}
		/* start thread CatErr_handler after 2 seconds */
		k_work_schedule_for_queue(&plat_work_q, &CAT_ERR_work,
					  K_SECONDS(CATERR_START_DELAY_SECOND));
	}
}

void ISR_PLTRST()
{
	send_gpio_interrupt(RST_PLTRST_BUF_N);
}

void ISR_DBP_PRSNT()
{
	send_gpio_interrupt(FM_DBP_PRESENT_N);
}

void ISR_FM_THROTTLE()
{
	common_addsel_msg_t sel_msg;
	if (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH) {
		if (gpio_get(FM_THROTTLE_R_N) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		}

		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_FMTHROTTLE;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			LOG_ERR("FM Throttle addsel fail");
		}
	}
}

void ISR_HSC_THROTTLE()
{
	common_addsel_msg_t sel_msg;
	static bool is_hsc_throttle_assert = false; // Flag for filt out fake alert
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		if (gpio_get(PWRGD_SYS_PWROK) == GPIO_LOW) {
			return;
		} else {
			if ((gpio_get(IRQ_SML1_PMBUS_ALERT_N) == GPIO_HIGH) &&
			    (is_hsc_throttle_assert == true)) {
				sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
				is_hsc_throttle_assert = false;
			} else if ((gpio_get(IRQ_SML1_PMBUS_ALERT_N) == GPIO_LOW) &&
				   (is_hsc_throttle_assert == false)) {
				sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
				is_hsc_throttle_assert = true;
			} else { // Fake alert
				return;
			}

			sel_msg.InF_target = PLDM;
			sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
			sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
			sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PMBUSALERT;
			sel_msg.event_data2 = 0xFF;
			sel_msg.event_data3 = 0xFF;
			if (!mctp_add_sel_to_ipmi(&sel_msg)) {
				LOG_ERR("HSC Throttle addsel fail");
			}
		}
	}
}

static void mb_throttle_handler(struct k_work *work)
{
	static bool is_mb_throttle_assert = false;
	common_addsel_msg_t sel_msg;

	if (((gpio_get(FAST_PROCHOT_N) == LOW_ACTIVE) && (is_mb_throttle_assert == false)) ||
	    ((gpio_get(FAST_PROCHOT_N) == LOW_INACTIVE) && (is_mb_throttle_assert == true))) {
		memset(&sel_msg, 0, sizeof(common_addsel_msg_t));
		if (gpio_get(FAST_PROCHOT_N) == LOW_INACTIVE) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
			is_mb_throttle_assert = false;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			is_mb_throttle_assert = true;
		}

		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_FIRMWAREASSERT;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			LOG_ERR("MB Throttle addsel fail");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(mb_throttle_work, mb_throttle_handler);
#define MB_THROTTLE_DELAY_US 4

void ISR_MB_THROTTLE()
{
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		if (k_work_cancel_delayable(&mb_throttle_work) != 0) {
			LOG_ERR("Cancel caterr delay work fail");
		}
		/* start thread mb_throttle_handler after 4us */
		k_work_schedule_for_queue(&plat_work_q, &mb_throttle_work,
					  K_USEC(MB_THROTTLE_DELAY_US));
	}
}

void ISR_SOC_THMALTRIP()
{
	common_addsel_msg_t sel_msg;
	if (gpio_get(RST_PLTRST_PLD_N) == GPIO_HIGH) {
		if (gpio_get(H_CPU_MEMTRIP_LVC3_N) ==
		    GPIO_HIGH) { // Reference pin for memory thermal trip event
			sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_THERMAL_TRIP;
		} else {
			sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_MEMORY_THERMALTRIP;
		}

		sel_msg.InF_target = PLDM;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			if (sel_msg.event_data1 == IPMI_OEM_EVENT_OFFSET_SYS_THERMAL_TRIP) {
				LOG_ERR("SOC Thermal trip addsel fail");
			} else {
				LOG_ERR("Memory Thermal trip addsel fail");
			}
		}
	}
}

void ISR_SYS_THROTTLE()
{
	common_addsel_msg_t sel_msg;
	if ((gpio_get(RST_PLTRST_PLD_N) == GPIO_HIGH) && (gpio_get(PWRGD_SYS_PWROK) == GPIO_HIGH)) {
		if (gpio_get(FM_CPU_BIC_PROCHOT_LVT3_N) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		}

		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_THROTTLE;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			LOG_ERR("System Throttle addsel fail");
		}
	}
}

void ISR_PCH_THMALTRIP()
{
	common_addsel_msg_t sel_msg;
	static bool is_pch_assert = 0;
	if (gpio_get(FM_PCHHOT_N) == GPIO_LOW) {
		if ((gpio_get(RST_PLTRST_PLD_N) == GPIO_HIGH) && (get_post_status() == true) &&
		    (is_pch_assert == false)) {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			is_pch_assert = true;
		}
	} else if (gpio_get(FM_PCHHOT_N) && (is_pch_assert == true)) {
		sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
		is_pch_assert = false;
	} else {
		return;
	}

	sel_msg.InF_target = PLDM;
	sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
	sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
	sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PCHHOT;
	sel_msg.event_data2 = 0xFF;
	sel_msg.event_data3 = 0xFF;
	if (!mctp_add_sel_to_ipmi(&sel_msg)) {
		LOG_ERR("PCH Thermal trip addsel fail");
	}
}

void ISR_HSC_OC()
{
	common_addsel_msg_t sel_msg;
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		if (gpio_get(FM_HSC_TIMER) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		}

		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_HSCTIMER;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			LOG_ERR("HSC OC addsel fail");
		}
	}
}

void ISR_CPU_MEMHOT()
{
	common_addsel_msg_t sel_msg;
	if ((gpio_get(RST_PLTRST_PLD_N) == GPIO_HIGH) && (gpio_get(PWRGD_SYS_PWROK) == GPIO_HIGH)) {
		if (gpio_get(H_CPU_MEMHOT_OUT_LVC3_N) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		}

		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_CPU_DIMM_HOT;
		sel_msg.sensor_number = SENSOR_NUM_CPUDIMM_HOT;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_DIMM_HOT;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			LOG_ERR("CPU MEM HOT addsel fail");
		}
	}
}

void ISR_CPUVR_HOT()
{
	common_addsel_msg_t sel_msg;
	if ((gpio_get(RST_PLTRST_PLD_N) == GPIO_HIGH) && (gpio_get(PWRGD_SYS_PWROK) == GPIO_HIGH)) {
		if (gpio_get(IRQ_CPU0_VRHOT_N) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		}

		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_CPU_DIMM_VR_HOT;
		sel_msg.sensor_number = SENSOR_NUM_VR_HOT;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_CPU_VR_HOT;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			LOG_ERR("CPU VR HOT addsel fail");
		}
	}
}

void ISR_PCH_PWRGD()
{
	common_addsel_msg_t sel_msg;
	if (gpio_get(FM_SLPS3_PLD_N) == GPIO_HIGH) {
		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_OEM_C3;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_number = SENSOR_NUM_POWER_ERROR;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_PCH_PWROK_FAIL;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			LOG_ERR("PCH PWROK failure addsel fail");
		}
	}
}

void ISR_RMCA()
{
	if ((gpio_get(RST_PLTRST_BUF_N) == GPIO_HIGH) || (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH)) {
		common_addsel_msg_t sel_msg;
		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_number = SENSOR_NUM_CATERR;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_MEM_RMCA;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			LOG_ERR("RMCA addsel fail");
		}
	}
}

void ISR_CPU_VPP_INT()
{
	if (gpio_get(PWRGD_CPU_LVC3) == POWER_ON) {
		int i = 0, ret = 0;
		uint8_t retry = 3;
		uint8_t vpp_pwr_status_bit = 0;
		uint8_t device_id = 0;
		uint8_t set_power_status = DEVICE_SET_POWER_OFF;
		// CPLD register define VPP PWREN# (1OU Expansion)
		// Bit 4: M.2 device 3
		// Bit 3: M.2 device 2
		// Bit 2: M.2 device 1
		// Bit 1: M.2 device 0
		// Bit 0: RSVD
		// default device0, 1, 3 are ON (bit1, 2, 4 = 0)
		uint8_t last_vpp_pwr_status = 0;
		uint8_t vpp_pwr_status = 0;
		ipmi_msg msg;
		common_addsel_msg_t sel_msg;

		last_vpp_pwr_status = get_last_vpp_power_status();
		vpp_pwr_status = get_updated_vpp_power_status();

		// Check BIOS is ready to handle VPP (post complete)
		if (gpio_get(FM_BIOS_POST_CMPLT_BMC_N) != LOW_ACTIVE) {
			return;
		}

		for (device_id = 0; device_id < MAX_1OU_M2_COUNT; device_id++) {
			// Shift to skip bit 0
			// VPP power status is start on bit 1
			vpp_pwr_status_bit = (vpp_pwr_status >> (device_id + 1)) & 0x1;
			if (vpp_pwr_status_bit ==
			    ((last_vpp_pwr_status >> (device_id + 1)) & 0x1)) {
				continue;
			}

			if (vpp_pwr_status_bit == 0) {
				set_power_status = DEVICE_SET_POWER_ON;
			} else {
				set_power_status = DEVICE_SET_POWER_OFF;
			}

			// Check 1OU E1.S power status before control
			// If power status isn't changed, control the power
			ret = get_set_1ou_m2_power(&msg, device_id, DEVICE_GET_POWER_STATUS);
			if ((msg.data[3] != set_power_status) || (ret < 0)) {
				// Notify 1OU BIC to turn on/off E1.S power
				for (i = 0; i < retry; i++) {
					ret = get_set_1ou_m2_power(&msg, device_id,
								   set_power_status);
					if (ret == 0) {
						break;
					}
				}

				if (i == retry) {
					LOG_ERR("Failed to send OEM_1S_GET_SET_M2 command 0x%x to 0x%x device%x",
						CMD_OEM_1S_GET_SET_M2, EXP1_IPMB,
						_1ou_m2_name_mapping_table[device_id]);
					continue;
				}
			}

			// Add SEL about VPP power event
			if (gpio_get(FM_SLPS3_PLD_N) == LOW_INACTIVE) {
				memset(&sel_msg, 0, sizeof(common_addsel_msg_t));
				sel_msg.InF_target = PLDM;
				sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
				sel_msg.event_type = IPMI_OEM_EVENT_TYPE_NOTIFY;
				sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
				sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_VPP_EVENT;
				sel_msg.event_data2 = IPMI_OEM_EVENT_OFFSET_1OU;
				sel_msg.event_data3 = _1ou_m2_name_mapping_table[device_id];
				if (!mctp_add_sel_to_ipmi(&sel_msg)) {
					LOG_ERR("addsel fail");
				}
			}
		}
	}
}

void ISR_NMI()
{
	if ((gpio_get(RST_PLTRST_PLD_N) == GPIO_HIGH) && (gpio_get(PWRGD_SYS_PWROK) == GPIO_HIGH)) {
		common_addsel_msg_t sel_msg;
		memset(&sel_msg, 0, sizeof(common_addsel_msg_t));
		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_SENSOR_TYPE_CRITICAL_INT;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_number = SENSOR_NUM_NMI;
		sel_msg.event_data1 = IPMI_EVENT_CRITICAL_INT_FP_NMI;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			LOG_ERR("addsel fail");
		}
	}
}

void ISR_RST_PLTRST_PLD()
{
	// Switch I3C mux to cpu when host reset or host DC on
	if (k_mutex_lock(&i3c_dimm_mux_mutex, K_MSEC(I3C_DIMM_MUX_MUTEX_TIMEOUT_MS))) {
		LOG_ERR("Failed to lock I3C dimm MUX");
	}

	switch_i3c_dimm_mux(I3C_MUX_TO_CPU, DIMM_MUX_TO_DIMM_A0A1A3);

	if (k_mutex_unlock(&i3c_dimm_mux_mutex)) {
		LOG_ERR("Failed to unlock I3C dimm MUX");
	}
}

void smi_handler()
{
	if (gpio_get(RST_PLTRST_PLD_N) == GPIO_HIGH && gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH &&
	    gpio_get(IRQ_SMI_ACTIVE_BMC_N) == GPIO_LOW) {
		common_addsel_msg_t sel_msg;
		memset(&sel_msg, 0, sizeof(common_addsel_msg_t));

		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_SMI90s;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			LOG_ERR("Failed to add SMI assertion SEL");
		}
		is_smi_assert = true;
	}
}

K_WORK_DELAYABLE_DEFINE(smi_work, smi_handler);
void ISR_SMI()
{
	/* Refer to "16.6 I/O Signal Planes and States"
	 *,which is in "6th Generation Intel® Core™ Processor Families I/O Platform".
	 * 
	 * BIC doesn't need to check SMI GPIO in "during reset" and "S5 power off" 
	 *states
	 */
	if (gpio_get(RST_PLTRST_PLD_N) == GPIO_HIGH && gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH) {
		if (gpio_get(IRQ_SMI_ACTIVE_BMC_N) == GPIO_LOW) {
			/* start thread smi_handler after 90 seconds */
			k_work_schedule_for_queue(&plat_work_q, &smi_work,
						  K_SECONDS(DETECT_SMI_DELAY_90S));
		} else {
			if (k_work_cancel_delayable(&smi_work) != 0) {
				LOG_ERR("Failed to cancel SMI delayable work");
			}
			if (is_smi_assert) {
				common_addsel_msg_t sel_msg;
				memset(&sel_msg, 0, sizeof(common_addsel_msg_t));

				sel_msg.InF_target = BMC_IPMB;
				sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
				sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
				sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
				sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_SMI90s;
				sel_msg.event_data2 = 0xFF;
				sel_msg.event_data3 = 0xFF;
				if (!common_add_sel_evt_record(&sel_msg)) {
					LOG_ERR("Failed to add SMI deassertion SEL");
				}

				is_smi_assert = false;
			}
		}
	}
}
