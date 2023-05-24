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
#include <logging/log.h>

#include "hal_gpio.h"
#include "hal_vw_gpio.h"
#include "libipmi.h"
#include "libutil.h"
#include "snoop.h"
#include "ipmb.h"
#include "ipmi.h"
#include "power_status.h"
#include "kcs.h"

#include "plat_i2c.h"
#include "plat_isr.h"
#include "plat_gpio.h"
#include "plat_ipmb.h"
#include "plat_sensor_table.h"

LOG_MODULE_REGISTER(plat_isr);

void send_gpio_interrupt(uint8_t gpio_num)
{
	ipmb_error status;
	ipmi_msg msg;
	uint8_t gpio_val;

	gpio_val = gpio_get(gpio_num);
	LOG_INF("Send gpio interrupt to BMC, gpio number(%d) status(%d)\n", gpio_num, gpio_val);

	msg.data_len = 5;
	msg.InF_source = SELF;
	msg.InF_target = BMC_IPMB;
	msg.netfn = NETFN_OEM_1S_REQ;
	msg.cmd = CMD_OEM_1S_SEND_INTERRUPT_TO_BMC;

	msg.data[0] = IANA_ID & 0xFF;
	msg.data[1] = (IANA_ID >> 8) & 0xFF;
	msg.data[2] = (IANA_ID >> 16) & 0xFF;
	msg.data[3] = gpio_num;
	msg.data[4] = gpio_val;

	status = ipmb_read(&msg, IPMB_inf_index_map[msg.InF_target]);
	if (status != IPMB_ERROR_SUCCESS) {
		LOG_ERR("Failed to send GPIO interrupt event to BMC, gpio number(%d) status(%d)",
			gpio_num, status);
	}
}

static void SLP3_handler()
{
	common_addsel_msg_t sel_msg;
	if ((gpio_get(FM_SLPS3_LVC3_N) == GPIO_HIGH) && (gpio_get(PWRGD_CPU_LVC3) == GPIO_LOW)) {
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_VRWATCHDOG;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("VR watchdog timeout addsel fail");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(SLP3_work, SLP3_handler);
void ISR_SLP3()
{
	if (gpio_get(FM_SLPS3_LVC3_N) == GPIO_HIGH) {
		LOG_INF("slp3\n");
		k_work_schedule(&SLP3_work, K_MSEC(10000));
		return;
	}
	if (k_work_cancel_delayable(&SLP3_work) != 0) {
		LOG_ERR("Failed to cancel delayable work");
	}
}

K_WORK_DELAYABLE_DEFINE(set_DC_on_5s_work, set_DC_on_delayed_status);
K_WORK_DELAYABLE_DEFINE(set_DC_off_10s_work, set_DC_off_delayed_status);

void Set_DC_status()
{
	set_DC_status(PWRGD_CPU_LVC3);

	if (get_DC_status() == true) {
		k_work_schedule(&set_DC_on_5s_work, K_SECONDS(DC_ON_5_SECOND));

		if (k_work_cancel_delayable(&set_DC_off_10s_work) != 0) {
			LOG_ERR("Cancel set dc off delay work fail");
		}
		set_DC_off_delayed_status();
	} else {
		set_DC_on_delayed_status();
		k_work_schedule(&set_DC_off_10s_work, K_SECONDS(DC_OFF_10_SECOND));

		if ((gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH) &&
		    (gpio_get(PWRGD_AUX_PWRGD_BMC_LVC3) == GPIO_HIGH)) {
			common_addsel_msg_t sel_msg;
			sel_msg.InF_target = BMC_IPMB;
			sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_OEM_C3;
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			sel_msg.sensor_number = SENSOR_NUM_POWER_ERROR;
			sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PWROK_FAIL;
			sel_msg.event_data2 = 0xFF;
			sel_msg.event_data3 = 0xFF;
			if (!common_add_sel_evt_record(&sel_msg)) {
				LOG_ERR("System PWROK failure addsel fail");
			}
		}
	}
}

static void PROC_FAIL_handler(struct k_work *work)
{
	/* if have not received kcs and post code, add FRB3 event log. */
	if ((get_kcs_ok() == false) && (get_postcode_ok() == false)) {
		common_addsel_msg_t sel_msg;
		bool ret = false;

		memset(&sel_msg, 0, sizeof(common_addsel_msg_t));
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.sensor_number = SENSOR_NUM_PROC_FAIL;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.event_data1 = IPMI_EVENT_OFFSET_PROCESSOR_FRB3;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		ret = common_add_sel_evt_record(&sel_msg);
		if (!ret) {
			LOG_ERR("Fail to assert FRB3 event log.");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(PROC_FAIL_work, PROC_FAIL_handler);

void Initialize_CPU()
{
	//check CPU's status by PWRGD_CPU_LVC3
	set_CPU_power_status(PWRGD_CPU_LVC3);
	if (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH) {
		init_snoop_thread();
		init_send_postcode_thread();
		/* start thread proc_fail_handler after 10 seconds */
		k_work_schedule(&PROC_FAIL_work, K_SECONDS(PROC_FAIL_START_DELAY_SECOND));
	} else {
		ISR_POST_COMPLETE(VW_GPIO_LOW);
		abort_snoop_thread();

		if (k_work_cancel_delayable(&PROC_FAIL_work) != 0) {
			LOG_ERR("Cancel proc_fail delay work fail");
		}

		reset_kcs_ok();
		reset_postcode_ok();
	}
	send_gpio_interrupt(PWRGD_CPU_LVC3);
}

void PWRGD_CPU_ACTIVE_HANDLE()
{
	Set_DC_status();
	Initialize_CPU();
}

void ISR_BMC_PRDY()
{
	send_gpio_interrupt(H_BMC_PRDY_BUF_N);
}

static void CAT_ERR_handler(struct k_work *work)
{
	if ((gpio_get(RST_PLTRST_BUF_N) == GPIO_HIGH) || (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH)) {
		common_addsel_msg_t sel_msg;
		bool ret = false;

		memset(&sel_msg, 0, sizeof(common_addsel_msg_t));
		sel_msg.InF_target = BMC_IPMB;
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
		ret = common_add_sel_evt_record(&sel_msg);
		if (!ret) {
			LOG_ERR("Fail to assert CatErr event log.");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(CAT_ERR_work, CAT_ERR_handler);

void ISR_CATERR()
{
	if (gpio_get(RST_PLTRST_BUF_N) == GPIO_LOW) {
		if (k_work_cancel_delayable(&CAT_ERR_work) != 0) {
			LOG_ERR("Cancel caterr delay work fail");
		}
		/* start thread CatErr_handler after 2 seconds */
		k_work_schedule(&CAT_ERR_work, K_SECONDS(CATERR_START_DELAY_SECOND));
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

void ISR_HSC_THROTTLE()
{
	common_addsel_msg_t sel_msg;
	// Flag for filter out fake alert
	static bool is_hsc_throttle_assert = false;
	if (gpio_get(PWRGD_AUX_PWRGD_BMC_LVC3) == GPIO_HIGH) {
		if (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH) {
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

			sel_msg.InF_target = BMC_IPMB;
			sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
			sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
			sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PMBUSALERT;
			sel_msg.event_data2 = 0xFF;
			sel_msg.event_data3 = 0xFF;
			if (!common_add_sel_evt_record(&sel_msg)) {
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

		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_FIRMWAREASSERT;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("MB Throttle addsel fail");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(mb_throttle_work, mb_throttle_handler);

void ISR_MB_THROTTLE()
{
	if (gpio_get(PWRGD_AUX_PWRGD_BMC_LVC3) == GPIO_HIGH) {
		if (k_work_cancel_delayable(&mb_throttle_work) != 0) {
			LOG_ERR("Cancel caterr delay work fail");
		}
		/* start thread mb_throttle_handler after 4us */
		k_work_schedule(&mb_throttle_work, K_USEC(MB_THROTTLE_DELAY_US));
	}
}

void ISR_SOC_THMALTRIP()
{
	common_addsel_msg_t sel_msg;
	if (gpio_get(RST_PLTRST_SYNC_LVC3_N) == GPIO_LOW) {
		if (gpio_get(H_CPU_MEMTRIP_LVC3_N) ==
		    GPIO_LOW) { // Reference pin for memory thermal trip event
			sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_THERMAL_TRIP;
		} else {
			sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_MEMORY_THERMALTRIP;
		}

		sel_msg.InF_target = BMC_IPMB;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			if (sel_msg.event_data1 == IPMI_OEM_EVENT_OFFSET_SYS_THERMAL_TRIP) {
				LOG_ERR("Failed to add SOC Thermal trip SEL");
			} else {
				LOG_ERR("Failed to add Memory Thermal trip SEL");
			}
		}
	}
}

void ISR_SYS_THROTTLE()
{
	common_addsel_msg_t sel_msg;
	if ((gpio_get(RST_PLTRST_SYNC_LVC3_N) == GPIO_LOW) &&
	    (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH)) {
		if (gpio_get(FM_CPU_BIC_PROCHOT_LVT3_N) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
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
			LOG_ERR("Failed to add system Throttle SEL");
		}
	}
}

void ISR_HSC_OC()
{
	common_addsel_msg_t sel_msg;
	if (gpio_get(PWRGD_AUX_PWRGD_BMC_LVC3) == GPIO_HIGH) {
		if (gpio_get(FM_HSC_TIMER) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
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
			LOG_ERR("Failed to add HSC OC SEL");
		}
	}
}

void ISR_CPU_MEMHOT()
{
	common_addsel_msg_t sel_msg;
	if ((gpio_get(RST_PLTRST_SYNC_LVC3_N) == GPIO_LOW) &&
	    (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH)) {
		if (gpio_get(H_CPU_MEMHOT_OUT_LVC3_N) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		}

		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_CPU_DIMM_HOT;
		sel_msg.sensor_number = SENSOR_NUM_CPUDIMM_HOT;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_DIMM_HOT;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("Failed to add CPU MEM HOT SEL");
		}
	}
}

void ISR_CPUVR_HOT()
{
	common_addsel_msg_t sel_msg;
	if ((gpio_get(RST_PLTRST_SYNC_LVC3_N) == GPIO_HIGH) &&
	    (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH)) {
		if ((gpio_get(IRQ_PVCCIN_CPU0_VRHOT_N) == GPIO_HIGH) ||
		    (gpio_get(IRQ_PVCCINF_CPU0_VRHOT_N) == GPIO_LOW)) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		}

		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_CPU_DIMM_VR_HOT;
		sel_msg.sensor_number = SENSOR_NUM_VR_HOT;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_CPU_VR_HOT;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("Failed to add CPU VR HOT SEL\n");
		}
	}
}

void ISR_RMCA()
{
	if ((gpio_get(RST_PLTRST_BUF_N) == GPIO_LOW) || (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH)) {
		common_addsel_msg_t sel_msg;
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_number = SENSOR_NUM_CATERR;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_MEM_RMCA;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("Failed to add RMCA SEL");
		}
	}
}

void ISR_POST_COMPLETE(uint8_t gpio_value)
{
	bool is_post_completed = (gpio_value == VW_GPIO_HIGH) ? true : false;
	set_post_complete(is_post_completed);

	// Add "END_OF_POST" event log to BMC
	common_addsel_msg_t sel_msg;
	sel_msg.InF_target = BMC_IPMB;
	sel_msg.sensor_type = IPMI_SENSOR_TYPE_SYS_EVENT;
	sel_msg.event_type =
		is_post_completed ? IPMI_EVENT_TYPE_SENSOR_SPECIFIC : IPMI_OEM_EVENT_TYPE_DEASSERT;
	sel_msg.sensor_number = SENSOR_NUM_END_OF_POST;
	sel_msg.event_data1 = IPMI_EVENT_OEM_SYSTEM_BOOT_EVENT;
	sel_msg.event_data2 = 0xFF;
	sel_msg.event_data3 = 0xFF;
	if (!common_add_sel_evt_record(&sel_msg)) {
		LOG_ERR("failed to add end_of_post sel");
	}
}

void ISR_FM_ADR_MODE0(uint8_t gpio_value)
{
	/* Set ADR mode into GL CPLD register
	 * Offset: 16h ADR mode 0 GPIO control
	 * Bit[0]: FM_ADR_MODE0, 0'b: (default) disable ADR
	 */
	int status;
	uint8_t value;
	I2C_MSG msg;
	msg.bus = SB_CPLD_BUS;
	msg.target_addr = SB_CPLD_ADDR;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = SB_CPLD_REG_ADR_MODE0_GPIO_CTRL;
	status = i2c_master_read(&msg, 5);
	if (status) {
		LOG_ERR("failed to get cpld register, ret %d", status);
		return;
	}
	value = msg.data[0];
	value = (gpio_value == VW_GPIO_HIGH) ? SETBIT(value, 0) : CLEARBIT(value, 0);

	msg.tx_len = 2;
	msg.data[0] = SB_CPLD_REG_ADR_MODE0_GPIO_CTRL;
	msg.data[1] = value;
	status = i2c_master_write(&msg, 5);
	if (status)
		LOG_ERR("failed to set cpld register, ret %d", status);
}
