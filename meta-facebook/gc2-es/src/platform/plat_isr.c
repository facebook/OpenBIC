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
#include <pmbus.h>
#include "sensor.h"
#include "snoop.h"
#include "plat_gpio.h"
#include "plat_class.h"
#include "plat_ipmi.h"
#include "plat_sensor_table.h"
#include "plat_i2c.h"
#include "plat_pmic.h"
#include "oem_1s_handler.h"
#include "hal_gpio.h"
#include "hal_i2c.h"
#include "util_sys.h"
#include "util_worker.h"

LOG_MODULE_REGISTER(plat_isr);

void send_gpio_interrupt(uint8_t gpio_num)
{
	ipmb_error status;
	ipmi_msg msg;
	uint8_t gpio_val;

	gpio_val = gpio_get(gpio_num);
	printf("Send gpio interrupt to BMC, gpio number(%d) status(%d)\n", gpio_num, gpio_val);

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
		printf("Failed to send GPIO interrupt event to BMC, gpio number(%d) status(%d)\n",
		       gpio_num, status);
	}
}

static void SLP3_handler()
{
	common_addsel_msg_t sel_msg;
	if ((gpio_get(FM_SLPS3_PLD_N) == GPIO_HIGH) && (gpio_get(PWRGD_SYS_PWROK) == GPIO_LOW)) {
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_VRWATCHDOG;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			printf("VR watchdog timeout addsel fail\n");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(SLP3_work, SLP3_handler);
#define DETECT_VR_WDT_DELAY_S 10
void ISR_SLP3()
{
	if (gpio_get(FM_SLPS3_PLD_N) == GPIO_HIGH) {
		printf("slp3\n");
		k_work_schedule_for_queue(&plat_work_q, &SLP3_work,
					  K_SECONDS(DETECT_VR_WDT_DELAY_S));
		return;
	}
	if (k_work_cancel_delayable(&SLP3_work) != 0) {
		printf("[%s] Failed to cancel delayable work\n", __func__);
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

#define DC_ON_5_SECOND 5
#define VR_EVENT_DELAY_MS 10

void ISR_DC_ON()
{
	set_DC_status(PWRGD_SYS_PWROK);

	ipmb_error status;
	ipmi_msg msg;
	bool dc_status = get_DC_status();

	if (dc_status) {
		k_work_schedule(&set_DC_on_5s_work, K_SECONDS(DC_ON_5_SECOND));

	} else {
		set_DC_on_delayed_status();

		if ((gpio_get(FM_SLPS3_PLD_N) == GPIO_HIGH) &&
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
				printf("System PWROK failure addsel fail\n");
			}
		}
	}

	msg.data_len = 4;
	msg.InF_source = SELF;
	msg.InF_target = BMC_IPMB;
	msg.netfn = NETFN_OEM_1S_REQ;
	msg.cmd = CMD_OEM_1S_SEND_HOST_POWER_STATE_TO_BMC;

	msg.data[0] = IANA_ID & 0xFF;
	msg.data[1] = (IANA_ID >> 8) & 0xFF;
	msg.data[2] = (IANA_ID >> 16) & 0xFF;
	msg.data[3] = dc_status ? GPIO_HIGH : GPIO_LOW;

	status = ipmb_read(&msg, IPMB_inf_index_map[msg.InF_target]);
	if (status != IPMB_ERROR_SUCCESS) {
		LOG_ERR("Failed to send host power state to BMC, gpio number %d status %d",
			PWRGD_SYS_PWROK, status);
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
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.sensor_number = SENSOR_NUM_PROC_FAIL;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.event_data1 = IPMI_EVENT_OFFSET_PROCESSOR_FRB3;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		ret = common_add_sel_evt_record(&sel_msg);
		if (!ret) {
			printf("Fail to assert FRE3 event log.\n");
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
			printf("Cancel proc_fail delay work fail\n");
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
			printf("Fail to assert CatErr event log.\n");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(CAT_ERR_work, CAT_ERR_handler);
#define CATERR_START_DELAY_SECOND 2
static volatile uint32_t caterr_irq_cnt;
void ISR_CATERR()
{
	caterr_irq_cnt++;
	printf("[BRING_UP_DEBUG]CATERR IRQ #%u at %u us\n", caterr_irq_cnt,
	       k_cycle_get_32() / CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);

	if ((gpio_get(RST_PLTRST_BUF_N) == GPIO_HIGH)) {
		if (k_work_cancel_delayable(&CAT_ERR_work) != 0) {
			printf("Cancel caterr delay work fail\n");
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

		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_FMTHROTTLE;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			printf("FM Throttle addsel fail\n");
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

			sel_msg.InF_target = BMC_IPMB;
			sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
			sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
			sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PMBUSALERT;
			sel_msg.event_data2 = 0xFF;
			sel_msg.event_data3 = 0xFF;
			if (!common_add_sel_evt_record(&sel_msg)) {
				printf("HSC Throttle addsel fail\n");
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
#define MB_THROTTLE_DELAY_US 4

void ISR_MB_THROTTLE()
{
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		if (k_work_cancel_delayable(&mb_throttle_work) != 0) {
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

		sel_msg.InF_target = BMC_IPMB;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			if (sel_msg.event_data1 == IPMI_OEM_EVENT_OFFSET_SYS_THERMAL_TRIP) {
				printf("SOC Thermal trip addsel fail\n");
			} else {
				printf("Memory Thermal trip addsel fail\n");
			}
		}
	}
}
/* Delay handler to prevent false SYS_THROTTLE SEL during power-off.
 * FM_CPU_BIC_PROCHOT_LVT3_N may have a short low pulse from CPLD during
 * power-off, triggering a false falling edge interrupt. By delaying 1ms,
 * PWRGD_SYS_PWROK will have dropped, and the condition check below will
 * correctly filter out the false event.
 */
static void sys_throttle_handler(struct k_work *work)
{
	common_addsel_msg_t sel_msg;
	memset(&sel_msg, 0, sizeof(common_addsel_msg_t));

	if ((gpio_get(RST_PLTRST_PLD_N) == GPIO_HIGH) && (gpio_get(PWRGD_SYS_PWROK) == GPIO_HIGH)) {
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
			printf("System Throttle addsel fail\n");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(sys_throttle_work, sys_throttle_handler);
#define SYS_THROTTLE_DELAY_MS 1

void ISR_SYS_THROTTLE()
{
	/* Cancel any pending work to handle rapid edge toggling,
	 * then reschedule with delay to filter out glitch during power-off.
	 */
	if (k_work_cancel_delayable(&sys_throttle_work) != 0) {
	}
	k_work_schedule_for_queue(&plat_work_q, &sys_throttle_work, K_MSEC(SYS_THROTTLE_DELAY_MS));
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

	sel_msg.InF_target = BMC_IPMB;
	sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
	sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
	sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PCHHOT;
	sel_msg.event_data2 = 0xFF;
	sel_msg.event_data3 = 0xFF;
	if (!common_add_sel_evt_record(&sel_msg)) {
		printf("PCH Thermal trip addsel fail\n");
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

		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_HSCTIMER;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			printf("HSC OC addsel fail\n");
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

		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_CPU_DIMM_HOT;
		sel_msg.sensor_number = SENSOR_NUM_CPUDIMM_HOT;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_DIMM_HOT;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			printf("CPU MEM HOT addsel fail\n");
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

		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_CPU_DIMM_VR_HOT;
		sel_msg.sensor_number = SENSOR_NUM_VR_HOT;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_CPU_VR_HOT;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			printf("CPU VR HOT addsel fail\n");
		}
	}
}

void ISR_PCH_PWRGD()
{
	common_addsel_msg_t sel_msg;
	if (gpio_get(FM_SLPS3_PLD_N) == GPIO_HIGH) {
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_OEM_C3;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_number = SENSOR_NUM_POWER_ERROR;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_PCH_PWROK_FAIL;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			printf("PCH PWROK failure addsel fail\n");
		}
	}
}

void ISR_RMCA()
{
	if ((gpio_get(RST_PLTRST_BUF_N) == GPIO_HIGH) || (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH)) {
		common_addsel_msg_t sel_msg;
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_number = SENSOR_NUM_CATERR;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_MEM_RMCA;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			printf("RMCA addsel fail\n");
		}
	}
}

void ISR_NMI()
{
	if ((gpio_get(RST_PLTRST_PLD_N) == GPIO_HIGH) && (gpio_get(PWRGD_SYS_PWROK) == GPIO_HIGH)) {
		common_addsel_msg_t sel_msg;
		memset(&sel_msg, 0, sizeof(common_addsel_msg_t));
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_SENSOR_TYPE_CRITICAL_INT;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_number = SENSOR_NUM_NMI;
		sel_msg.event_data1 = IPMI_EVENT_CRITICAL_INT_FP_NMI;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("%s addsel fail", __func__);
		}
	}
}

static bool send_smi_sel(event_state_t state)
{
	common_addsel_msg_t sel_msg = { 0 };

	sel_msg.InF_target = BMC_IPMB;
	sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
	sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
	sel_msg.event_type =
		(state == ASSERT) ? IPMI_EVENT_TYPE_SENSOR_SPECIFIC : IPMI_OEM_EVENT_TYPE_DEASSERT;
	sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_SMI90s;
	sel_msg.event_data2 = 0xFF;
	sel_msg.event_data3 = 0xFF;

	return common_add_sel_evt_record(&sel_msg);
}

static volatile bool smi_stuck_active;
static volatile bool smi_assert_sent;

void smi_handler()
{
	if (gpio_get(RST_PLTRST_PLD_N) != GPIO_HIGH || gpio_get(PWRGD_CPU_LVC3) != GPIO_HIGH) {
		return;
	}

	if (gpio_get(IRQ_SMI_ACTIVE_BMC_N) == GPIO_LOW) {
		smi_stuck_active = true;

		if (!smi_assert_sent) {
			if (send_smi_sel(ASSERT)) {
				smi_assert_sent = true;
				LOG_WRN("SMI+");
			} else {
				LOG_ERR("Failed to add SMI assertion SEL");
			}
		}
	}
}

K_WORK_DELAYABLE_DEFINE(smi_work, smi_handler);
void ISR_SMI(void)
{
	/* Refer to "16.6 I/O Signal Planes and States"
	 *,which is in "6th Generation Intel® Core™ Processor Families I/O Platform".
	 *
	 * BIC doesn't need to check SMI GPIO in "during reset" and "S5 power off"
	 *states
	 */
	if (gpio_get(RST_PLTRST_PLD_N) != GPIO_HIGH || gpio_get(PWRGD_CPU_LVC3) != GPIO_HIGH) {
		return;
	}

	if (gpio_get(IRQ_SMI_ACTIVE_BMC_N) == GPIO_LOW) {
		if (!k_work_delayable_is_pending(&smi_work) && !smi_stuck_active) {
			k_work_schedule_for_queue(&plat_work_q, &smi_work,
						  K_SECONDS(DETECT_SMI_DELAY_90S));
		}
	} else {
		smi_stuck_active = false;
		(void)k_work_cancel_delayable(&smi_work);

		if (smi_assert_sent) {
			if (send_smi_sel(DEASSERT)) {
				smi_assert_sent = false;
				LOG_WRN("SMI-");
			} else {
				LOG_ERR("Failed to add SMI deassertion SEL");
			}
		}
	}
}

static bool send_alert_sel(event_state_t state, uint8_t event_offset)
{
	common_addsel_msg_t sel_msg = { 0 };

	sel_msg.InF_target = BMC_IPMB;
	sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
	sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
	sel_msg.event_type =
		(state == ASSERT) ? IPMI_EVENT_TYPE_SENSOR_SPECIFIC : IPMI_OEM_EVENT_TYPE_DEASSERT;
	sel_msg.event_data1 = event_offset;
	sel_msg.event_data2 = 0xFF;
	sel_msg.event_data3 = 0xFF;

	return common_add_sel_evt_record(&sel_msg);
}

void ISR_E1S_0_INA233_ALRT(void)
{
	if (gpio_get(SMB_E1S_0_INA233_ALRT_N) == GPIO_LOW) {
		LOG_INF("GPIOB0+");
		if (!send_alert_sel(ASSERT, IPMI_OEM_EVENT_OFFSET_SYS_E1S0_ALERT)) {
			LOG_ERR("E1S INA233 alert addsel fail");
		}
	}
}

void ISR_SMB_SENSOR_LVC3_ALERT(void)
{
	if (gpio_get(SMB_SENSOR_LVC3_ALERT_N) == GPIO_LOW) {
		LOG_INF("GPIOE4+");
		if (!send_alert_sel(ASSERT, IPMI_OEM_EVENT_OFFSET_SYS_TEMP_ALERT)) {
			LOG_ERR("Failed to send SMB_SENSOR_LVC3_ALERT_N SEL");
		}
	}
}

add_vr_sel_info vr_event_work_item[] = {
	{
		.is_init = false,
		.gpio_num = FM_FORCE_ADR_N_R,
	},
};

const vr_fault_info vr_fault_table[] = {
	// { vr_source_id, cpld_reg_data_idx, cpld_reg_bit, is_pmbus_vr, vr_i2c_bus, vr_addr, vr_page }
	{ PVCCIN_CPU0, CPLD_REG_INFO_IDX_0, BIT(2), true, I2C_BUS5, PVCCIN_ADDR, 0 },
	{ PVCCFA_EHV_FIVRA_CPU0, CPLD_REG_INFO_IDX_0, BIT(5), true, I2C_BUS5, PVCCFA_EHV_FIVRA_ADDR,
	  1 },
	{ PVCCINFAON_CPU0, CPLD_REG_INFO_IDX_0, BIT(4), true, I2C_BUS5, PVCCINFAON_ADDR, 0 },
	{ PVCCFA_EHV_CPU0, CPLD_REG_INFO_IDX_0, BIT(6), true, I2C_BUS5, PVCCFA_EHV_ADDR, 1 },
	{ PVCCD_HV_CPU, CPLD_REG_INFO_IDX_0, BIT(7), true, I2C_BUS5, PVCCD_HV_ADDR, 0 },
	{ P1V05_PCH_STB, CPLD_REG_INFO_IDX_1, BIT(2), false, 0, 0, 0 },
	{ P1V8_STBY, CPLD_REG_INFO_IDX_1, BIT(4), false, 0, 0, 0 },
	{ P5V_STBY, CPLD_REG_INFO_IDX_1, BIT(3), false, 0, 0, 0 },
	{ P3V3_STBY, CPLD_REG_INFO_IDX_1, BIT(5), false, 0, 0, 0 },
	{ VR_P12V_E1S_0, CPLD_REG_INFO_IDX_1, BIT(6), false, 0, 0, 0 },
	{ VR_P3V3_E1S_0, CPLD_REG_INFO_IDX_1, BIT(7), false, 0, 0, 0 },
};

const cpld_reg_info cpld_reg_table[CPLD_REG_INFO_IDX_MAX] = {
	// { cpld_reg_i2c_bus, cpld_reg_addr, cpld_reg_offset }
	{ I2C_BUS1, CPLD_ADDR, 0x0A }, // CPLD_REG_INFO_IDX_0
	{ I2C_BUS1, CPLD_ADDR, 0x03 }, // CPLD_REG_INFO_IDX_1
};

const uint8_t vr_reg_list[][9] = {
	// RNS INFINEON
	{ PMBUS_STATUS_WORD, PMBUS_STATUS_BYTE, PMBUS_STATUS_VOUT, PMBUS_STATUS_IOUT,
	  PMBUS_STATUS_INPUT, PMBUS_STATUS_TEMPERATURE, PMBUS_STATUS_CML,
	  PMBUS_STATUS_MFR_SPECIFIC },

	// TI
	{ PMBUS_STATUS_WORD, PMBUS_STATUS_BYTE, PMBUS_STATUS_VOUT, PMBUS_STATUS_IOUT,
	  PMBUS_STATUS_INPUT, PMBUS_STATUS_TEMPERATURE, PMBUS_STATUS_CML, PMBUS_STATUS_OTHER,
	  PMBUS_STATUS_MFR_SPECIFIC },
};

static const uint8_t vr_reg_list_len_tbl[] = {
	8, /* RNS / INFINEON */
	9, /* TI */
};

static bool is_cpld_reg_bit_set(uint8_t bus, uint8_t addr, uint8_t reg, uint8_t bit)
{
	uint8_t bit_mask;
	uint8_t retry = 5;
	I2C_MSG msg = { 0 };

	if (bit >= 8) {
		return false;
	}

	bit_mask = BIT(bit);

	msg.bus = bus;
	msg.target_addr = addr;
	msg.tx_len = 1;
	msg.rx_len = 1;
	msg.data[0] = reg;

	if (i2c_master_read(&msg, retry)) {
		LOG_ERR("[%s] Failed to get cpld reg, bus: 0x%x, addr: 0x%x, reg: 0x%x", __func__,
			msg.bus, msg.target_addr, reg);
		return false;
	}
	LOG_INF("[%s] read value: 0x%02x from bus: 0x%x, addr: 0x%x, reg: 0x%x bit: 0x%x", __func__,
		msg.data[0], msg.bus, msg.target_addr, reg, bit_mask);

	return !!(msg.data[0] & bit_mask);
}

void process_vr_power_fault_sel(struct k_work *work_item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work_item);
	add_vr_sel_info *work_info = CONTAINER_OF(dwork, add_vr_sel_info, add_sel_work);
	LOG_INF("[%s] Handle GPIO(%d) interrupt", __func__, work_info->gpio_num);

	uint8_t cpld_reg_data[CPLD_REG_INFO_IDX_MAX] = { 0 };
	uint8_t retry = 5;

	// collect CPLD power fault register data
	for (int i = 0; i < CPLD_REG_INFO_IDX_MAX; i++) {
		I2C_MSG msg = { 0 };
		msg.bus = cpld_reg_table[i].cpld_reg_i2c_bus;
		msg.target_addr = cpld_reg_table[i].cpld_reg_addr;
		msg.tx_len = 1;
		msg.rx_len = 1;
		msg.data[0] = cpld_reg_table[i].cpld_reg_offset;
		if (i2c_master_read(&msg, retry)) {
			LOG_ERR("[%s] Failed to get cpld reg, bus: 0x%x, addr: 0x%x, reg: 0x%x",
				__func__, msg.bus, msg.target_addr,
				cpld_reg_table[i].cpld_reg_offset);
			cpld_reg_data[i] = 0x00;
			continue;
		}
		cpld_reg_data[i] = msg.data[0];
		LOG_INF("[%s] Get cpld reg. bus: 0x%x, addr: 0x%x, reg: 0x%x, data: 0x%x", __func__,
			msg.bus, msg.target_addr, cpld_reg_table[i].cpld_reg_offset,
			cpld_reg_data[i]);
	}

	uint8_t vr_dev = detect_vr_module_via_pmbus();
	// check VR power rail
	for (int i = 0; i < sizeof(vr_fault_table) / sizeof(vr_fault_table[0]); i++) {
		if (vr_fault_table[i].cpld_reg_data_idx >= CPLD_REG_INFO_IDX_MAX) {
			LOG_ERR("[%s] Invalid CPLD reg data index: %d", __func__,
				vr_fault_table[i].cpld_reg_data_idx);
			continue;
		}

		bool is_power_fault = (cpld_reg_data[vr_fault_table[i].cpld_reg_data_idx] &
				       vr_fault_table[i].cpld_reg_bit) ?
					      false :
					      true;

		if (vr_fault_table[i].vr_source_id == VR_P3V3_E1S_0) {
			if (is_cpld_reg_bit_set(I2C_BUS1, CPLD_ADDR, CPLD_OFFSET_10,
						CPLD_BIT_E1S_0_3V3_POWER_R_EN) == false) {
				LOG_WRN("Ignore P3V3_E1S_0 fault due to DC off");
				is_power_fault = false;
			}
		} else if (vr_fault_table[i].vr_source_id == VR_P12V_E1S_0) {
			if (is_cpld_reg_bit_set(I2C_BUS1, CPLD_ADDR, CPLD_OFFSET_10,
						CPLD_BIT_E1S_0_12V_POWER_R_EN) == false) {
				LOG_WRN("Ignore P12V_E1S_0 fault due to DC off");
				is_power_fault = false;
			}
		}

		LOG_INF("[%s] Check VR fault, src: %d, data: 0x%x, bitmask: 0x%x, is_power_fault: %d",
			__func__, vr_fault_table[i].vr_source_id,
			cpld_reg_data[vr_fault_table[i].cpld_reg_data_idx],
			vr_fault_table[i].cpld_reg_bit, is_power_fault);

		if (is_power_fault == false) {
			continue;
		}

		if (vr_fault_table[i].is_pmbus_vr == false) {
			// non-PMBus VR
			common_addsel_msg_t sel_msg = { 0 };
			sel_msg.InF_target = BMC_IPMB;
			sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
			sel_msg.sensor_number = SENSOR_NUM_VR_FAULT;
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			sel_msg.event_data1 = vr_fault_table[i].vr_source_id;
			sel_msg.event_data2 = 0xFF;
			sel_msg.event_data3 = 0xFF;

			if (!common_add_sel_evt_record(&sel_msg)) {
				LOG_ERR("[%s] Failed to send VR FAULT assert SEL, event data: 0x%x 0x%x 0x%x",
					__func__, sel_msg.event_data1, sel_msg.event_data2,
					sel_msg.event_data3);
			} else {
				LOG_INF("[%s] Send VR FAULT assert SEL, event data: 0x%x 0x%x 0x%x",
					__func__, sel_msg.event_data1, sel_msg.event_data2,
					sel_msg.event_data3);
			}
		} else {
			// PMBus VR
			I2C_MSG msg = { 0 };
			msg.bus = vr_fault_table[i].vr_i2c_bus;
			msg.target_addr = vr_fault_table[i].vr_addr;

			uint8_t vr_reg_list_idx = 0;
			uint8_t vr_reg_len = 0;
			uint16_t vr_status_word_mask = 0;

			switch (vr_dev) {
			case VR_MODULE_ISL69259: // main-source: RNS
			case VR_MODULE_XDPE15284D: // 2nd-source: INFINEON
				vr_reg_list_idx = 0;
				vr_status_word_mask =
					(uint16_t)(BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(13) |
						   BIT(14) | BIT(15)); //0xE03C
				break;
			case VR_MODULE_TPS53689: // 3rd-source: TI
				vr_reg_list_idx = 1;
				vr_status_word_mask =
					(uint16_t)(BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(9) |
						   BIT(13) | BIT(14) | BIT(15)); //0xE23C
				break;
			default: // other-source: RNS (Default)
				vr_reg_list_idx = 0;
				vr_status_word_mask =
					(uint16_t)(BIT(2) | BIT(3) | BIT(4) | BIT(5) | BIT(13) |
						   BIT(14) | BIT(15)); //0xE03C
				break;
			}

			vr_reg_len = vr_reg_list_len_tbl[vr_reg_list_idx];

			disable_sensor_poll();
			// wait 10ms for vr monitor stop
			k_msleep(10);

			// set page for power rail
			msg.tx_len = 2;
			msg.data[0] = PMBUS_PAGE;
			msg.data[1] = vr_fault_table[i].vr_page;
			if (i2c_master_write(&msg, retry)) {
				LOG_ERR("[%s] Failed to set VR page, bus: %d, addr: 0x%x, page: 0x%x",
					__func__, vr_fault_table[i].vr_i2c_bus,
					vr_fault_table[i].vr_addr, vr_fault_table[i].vr_page);
				enable_sensor_poll();
				continue;
			}

			// collect status data
			bool report_sel = true;
			common_addsel_msg_t sel_msg[vr_reg_len];
			memset(sel_msg, 0, sizeof(sel_msg));
			uint8_t sel_msg_idx = 0;
			for (int j = 0; j < vr_reg_len; j++) {
				msg.tx_len = 1;
				msg.rx_len = 1;
				msg.data[0] = vr_reg_list[vr_reg_list_idx][j];
				if (vr_reg_list[vr_reg_list_idx][j] == PMBUS_STATUS_WORD) {
					msg.rx_len = 2;
				}
				if (i2c_master_read(&msg, retry)) {
					LOG_ERR("[%s] Failed to get vr reg, bus: %d, addr: 0x%x, reg: 0x%x",
						__func__, msg.bus, msg.target_addr,
						vr_reg_list[vr_reg_list_idx][j]);
					continue;
				}
				sel_msg[sel_msg_idx].InF_target = BMC_IPMB;
				sel_msg[sel_msg_idx].sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
				sel_msg[sel_msg_idx].sensor_number = SENSOR_NUM_VR_FAULT;
				sel_msg[sel_msg_idx].event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
				sel_msg[sel_msg_idx].event_data1 = vr_fault_table[i].vr_source_id;
				sel_msg[sel_msg_idx].event_data2 =
					(vr_reg_list[vr_reg_list_idx][j] == PMBUS_STATUS_WORD) ?
						msg.data[1] :
						vr_reg_list[vr_reg_list_idx][j];
				sel_msg[sel_msg_idx].event_data3 = msg.data[0];

				if (vr_reg_list[vr_reg_list_idx][j] == PMBUS_STATUS_WORD) {
					uint16_t status_word =
						((uint16_t)msg.data[1] << 8) | msg.data[0];
					if ((status_word & vr_status_word_mask) == 0) {
						LOG_WRN("[%s] src:%d skip SEL because STATUS_WORD is 0x%x",
							__func__, vr_fault_table[i].vr_source_id,
							status_word);
						report_sel = false;
						break;
					}
				}

				sel_msg_idx += 1;
			}

			if (report_sel == false) {
				enable_sensor_poll();
				continue;
			}
			enable_sensor_poll();

			// Send SEL to BMC
			for (int j = 0; j < sel_msg_idx; j++) {
				if (!common_add_sel_evt_record(&sel_msg[j])) {
					LOG_ERR("[%s] Failed to send VR FAULT assert SEL, event data: 0x%x 0x%x 0x%x",
						__func__, sel_msg[j].event_data1,
						sel_msg[j].event_data2, sel_msg[j].event_data3);
				} else {
					LOG_INF("[%s] Send VR FAULT assert SEL, event data: 0x%x 0x%x 0x%x",
						__func__, sel_msg[j].event_data1,
						sel_msg[j].event_data2, sel_msg[j].event_data3);
				}
			}
		}
	}
}

void init_vr_event_work()
{
	for (int index = 0; index < ARRAY_SIZE(vr_event_work_item); ++index) {
		if (vr_event_work_item[index].is_init != true) {
			k_work_init_delayable(&vr_event_work_item[index].add_sel_work,
					      process_vr_power_fault_sel);
			vr_event_work_item[index].is_init = true;
		}
	}
}

void ISR_VR_PWR_FAULT()
{
	LOG_INF("VR power fault event triggered");
	k_work_schedule_for_queue(&plat_work_q, &vr_event_work_item[0].add_sel_work,
				  K_MSEC(VR_EVENT_DELAY_MS));
}
