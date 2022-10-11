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
#include "plat_ipmi.h"
#include "plat_sensor_table.h"
#include "oem_1s_handler.h"
#include "hal_gpio.h"
#include "util_sys.h"

LOG_MODULE_REGISTER(plat_isr);

static void isr_dbg_print(uint8_t gpio_num)
{
	switch (gpio_cfg[gpio_num].int_type) {
	case GPIO_INT_EDGE_FALLING:
		LOG_INF("gpio[%-3d] isr type[fall] trigger 1 -> 0", gpio_num);
		break;

	case GPIO_INT_EDGE_RISING:
		LOG_INF("gpio[%-3d] isr type[rise] trigger 0 -> 1", gpio_num);
		break;

	case GPIO_INT_EDGE_BOTH:
		if (gpio_get(gpio_num))
			LOG_INF("gpio[%-3d] isr type[both] trigger 0 -> 1", gpio_num);
		else
			LOG_INF("gpio[%-3d] isr type[both] trigger 1 -> 0", gpio_num);
		break;

	default:
		LOG_WRN("gpio[%-3d] isr trigger unexpected", gpio_num);
		break;
	}
}

void send_gpio_interrupt(uint8_t gpio_num)
{
	ipmb_error status;
	ipmi_msg msg;
	uint8_t gpio_val;

	gpio_val = gpio_get(gpio_num);
	LOG_INF("Send gpio interrupt to BMC, gpio number(%d) status(%d)", gpio_num, gpio_val);

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
	if ((gpio_get(FM_SLPS3_PLD_N) == GPIO_HIGH) && (gpio_get(PWRGD_SYS_PWROK) == GPIO_LOW)) {
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_VRWATCHDOG;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("VR watchdog timeout addsel fail\n");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(SLP3_work, SLP3_handler);
void ISR_SLP3()
{
	isr_dbg_print(FM_SLPS3_PLD_N);

	if (gpio_get(FM_SLPS3_PLD_N) == GPIO_HIGH) {
		LOG_INF("slp3");
		k_work_schedule(&SLP3_work, K_MSEC(10000));
		return;
	}
	if (k_work_cancel_delayable(&SLP3_work) != 0) {
		LOG_ERR("[%s] Failed to cancel delayable work", __func__);
	}
}

void ISR_POST_COMPLETE()
{
	isr_dbg_print(FM_BIOS_POST_CMPLT_BIC_N);

	if (gpio_get(FM_BIOS_POST_CMPLT_BIC_N) == GPIO_LOW) { // Post complete
		if (get_me_mode() == ME_INIT_MODE) {
			init_me_firmware();
		}
	}

	set_post_status(FM_BIOS_POST_CMPLT_BIC_N);
}

K_WORK_DELAYABLE_DEFINE(set_DC_on_5s_work, set_DC_on_delayed_status);
K_WORK_DELAYABLE_DEFINE(set_DC_off_10s_work, set_DC_off_delayed_status);
#define DC_ON_5_SECOND 5
#define DC_OFF_10_SECOND 10
void ISR_DC_ON()
{
	isr_dbg_print(PWRGD_SYS_PWROK);
	set_DC_status(PWRGD_SYS_PWROK);

	if (get_DC_status() == true) {
		gpio_set(BMC_PWR_LED, GPIO_HIGH);
		k_work_schedule(&set_DC_on_5s_work, K_SECONDS(DC_ON_5_SECOND));

		if (k_work_cancel_delayable(&set_DC_off_10s_work) != 0) {
			LOG_ERR("Cancel set dc off delay work fail");
		}
		set_DC_off_delayed_status();
	} else {
		gpio_set(BMC_PWR_LED, GPIO_LOW);
		set_DC_on_delayed_status();
		k_work_schedule(&set_DC_off_10s_work, K_SECONDS(DC_OFF_10_SECOND));

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
				LOG_ERR("System PWROK failure addsel fail");
			}
		}
	}
}

void ISR_BMC_PRDY()
{
	isr_dbg_print(JTAG_DBP_BMC_PRDY_N);
	send_gpio_interrupt(JTAG_DBP_BMC_PRDY_N);
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
			LOG_ERR("Fail to assert FRE3 event log");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(PROC_FAIL_work, PROC_FAIL_handler);
#define PROC_FAIL_START_DELAY_SECOND 10
void ISR_PWRGD_CPU()
{
	isr_dbg_print(PWRGD_CPU_LVC3);
	set_CPU_power_status(PWRGD_CPU_LVC3);

	if (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH) {
		init_snoop_thread();
		init_send_postcode_thread();

		/* start thread proc_fail_handler after 10 seconds */
		k_work_schedule(&PROC_FAIL_work, K_SECONDS(PROC_FAIL_START_DELAY_SECOND));
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
	if ((gpio_get(RST_PLTRST_BIC_N) == GPIO_HIGH) || (gpio_get(PWRGD_SYS_PWROK) == GPIO_HIGH)) {
		common_addsel_msg_t sel_msg;
		bool ret = false;

		memset(&sel_msg, 0, sizeof(common_addsel_msg_t));
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.sensor_number = SENSOR_NUM_CATERR;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		/* MCERR: one pulse, IERR: keep low */
		if (gpio_get(FM_CPU_CATERR_LVT3_N) == GPIO_HIGH) {
			sel_msg.event_data1 = IPMI_EVENT_OFFSET_PROCESSOR_MCERR;
		} else {
			sel_msg.event_data1 = IPMI_EVENT_OFFSET_PROCESSOR_IERR;
		}
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		ret = common_add_sel_evt_record(&sel_msg);
		if (!ret) {
			LOG_ERR("Fail to assert CatErr event log");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(CAT_ERR_work, CAT_ERR_handler);
#define CATERR_START_DELAY_SECOND 2
void ISR_CATERR()
{
	isr_dbg_print(FM_CPU_CATERR_LVT3_N);

	if ((gpio_get(RST_PLTRST_BIC_N) == GPIO_HIGH)) {
		if (k_work_cancel_delayable(&CAT_ERR_work) != 0) {
			LOG_ERR("Cancel caterr delay work fail");
		}
		/* start thread CatErr_handler after 2 seconds */
		k_work_schedule(&CAT_ERR_work, K_SECONDS(CATERR_START_DELAY_SECOND));
	}
}

void ISR_PLTRST()
{
	isr_dbg_print(RST_PLTRST_BIC_N);
	send_gpio_interrupt(RST_PLTRST_BIC_N);
}

void ISR_DBP_PRSNT()
{
	isr_dbg_print(FM_DBP_PRESENT_N);
	send_gpio_interrupt(FM_DBP_PRESENT_N);
}

void ISR_FM_THROTTLE()
{
	isr_dbg_print(FM_THROTTLE_R_N);

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
			LOG_ERR("FM Throttle addsel fail");
		}
	}
}

void ISR_HSC_THROTTLE()
{
	isr_dbg_print(IRQ_SML1_PMBUS_BMC_ALERT_N);

	common_addsel_msg_t sel_msg;
	static bool is_hsc_throttle_assert = false; // Flag for filt out fake alert
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		if (gpio_get(PWRGD_SYS_PWROK) == GPIO_LOW) {
			return;
		} else {
			if ((gpio_get(IRQ_SML1_PMBUS_BMC_ALERT_N) == GPIO_HIGH) &&
			    (is_hsc_throttle_assert == true)) {
				sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
				is_hsc_throttle_assert = false;
			} else if ((gpio_get(IRQ_SML1_PMBUS_BMC_ALERT_N) == GPIO_LOW) &&
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

void ISR_MB_THROTTLE()
{
	isr_dbg_print(P12V_HS_D_OC_R_N);
}

void ISR_SOC_THMALTRIP()
{
	isr_dbg_print(FM_CPU_THERMTRIP_LATCH_LVT3_N);

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
				LOG_ERR("SOC Thermal trip addsel fail");
			} else {
				LOG_ERR("Memory Thermal trip addsel fail");
			}
		}
	}
}

void ISR_SYS_THROTTLE()
{
	isr_dbg_print(FM_CPU_BIC_PROCHOT_LVT3_N);

	common_addsel_msg_t sel_msg;
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
			LOG_ERR("System Throttle addsel fail");
		}
	}
}

void ISR_PCH_THMALTRIP()
{
	isr_dbg_print(FM_PCHHOT_N);

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
		LOG_ERR("PCH Thermal trip addsel fail");
	}
}

void ISR_HSC_OC()
{
	// TODO
}

void ISR_CPU_MEMHOT()
{
	isr_dbg_print(H_CPU0_MEMHOT_OUT_LVC3_N);

	common_addsel_msg_t sel_msg;
	if ((gpio_get(RST_PLTRST_PLD_N) == GPIO_HIGH) && (gpio_get(PWRGD_SYS_PWROK) == GPIO_HIGH)) {
		if (gpio_get(H_CPU0_MEMHOT_OUT_LVC3_N) == GPIO_HIGH) {
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
			LOG_ERR("CPU MEM HOT addsel fail");
		}
	}
}

void ISR_CPUVR_HOT()
{
	isr_dbg_print(IRQ_CPU0_VRHOT_N);

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
			LOG_ERR("CPU VR HOT addsel fail");
		}
	}
}

void ISR_PCH_PWRGD()
{
	isr_dbg_print(RST_RSMRST_BMC_N);

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
			LOG_ERR("PCH PWROK failure addsel fail");
		}
	}
}

void ISR_RMCA()
{
	isr_dbg_print(FM_CPU_RMCA_LVT3_N);

	if ((gpio_get(RST_PLTRST_BIC_N) == GPIO_HIGH) || (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH)) {
		common_addsel_msg_t sel_msg;
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.sensor_number = SENSOR_NUM_RMCA;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_MEM_RMCA;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("RMCA addsel fail");
		}
	}
}

static bool is_smi_assert = false;
static void SMI_handler(struct k_work *work)
{
	if (gpio_get(IRQ_SMI_ACTIVE_BIC_N) == GPIO_LOW) {
		common_addsel_msg_t sel_msg;
		memset(&sel_msg, 0, sizeof(common_addsel_msg_t));

		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYS_STA;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_SMI90s;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			printf("SMI addsel fail\n");
		}

		gpio_interrupt_conf(IRQ_SMI_ACTIVE_BIC_N, GPIO_INT_EDGE_RISING);
		is_smi_assert = true;
	}
}

K_WORK_DELAYABLE_DEFINE(SMI_work, SMI_handler);
#define SMI_START_DELAY_SECOND 90
void ISR_SMI()
{
	isr_dbg_print(IRQ_SMI_ACTIVE_BIC_N);

	if (gpio_get(RST_PLTRST_BIC_N) == GPIO_HIGH) {
		if (gpio_get(IRQ_SMI_ACTIVE_BIC_N) == GPIO_LOW) {
			/* start thread SMI_handler after 90 seconds */
			k_work_schedule(&SMI_work, K_SECONDS(SMI_START_DELAY_SECOND));
		} else {
			if (is_smi_assert == true) {
				common_addsel_msg_t sel_msg;
				memset(&sel_msg, 0, sizeof(common_addsel_msg_t));

				sel_msg.InF_target = BMC_IPMB;
				sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
				sel_msg.sensor_number = SENSOR_NUM_SYS_STA;
				sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
				sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_SMI90s;
				sel_msg.event_data2 = 0xFF;
				sel_msg.event_data3 = 0xFF;
				if (!common_add_sel_evt_record(&sel_msg)) {
					printf("SMI addsel fail\n");
				}

				gpio_interrupt_conf(IRQ_SMI_ACTIVE_BIC_N, GPIO_INT_EDGE_FALLING);
				is_smi_assert = false;
			}
		}
	}
}
