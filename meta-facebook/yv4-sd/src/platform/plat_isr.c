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
#include <stdlib.h>
#include <pmbus.h>
#include "libipmi.h"
#include "app_handler.h"
#include "kcs.h"
#include "rg3mxxb12.h"
#include "power_status.h"
#include "sensor.h"
#include "snoop.h"
#include "apml.h"
#include "hal_gpio.h"
#include "hal_i2c.h"
#include "hal_i3c.h"
#include "util_sys.h"
#include "util_worker.h"
#include "plat_gpio.h"
#include "plat_class.h"
#include "plat_pldm_sensor.h"
#include "plat_sensor_table.h"
#include "plat_i2c.h"
#include "plat_mctp.h"
#include "plat_apml.h"
#include "plat_i3c.h"
#include "plat_isr.h"
#include "plat_dimm.h"
#include "pcc.h"
#include "plat_pmic.h"

#ifdef ENABLE_PLDM
#include "pldm_oem.h"
#endif

LOG_MODULE_REGISTER(plat_isr, LOG_LEVEL_DBG);

uint8_t hw_event_register[13] = { 0 };

add_vr_sel_info vr_event_work_item[] = {
	{
		.is_init = false,
		.gpio_num = PVDDCR_CPU0_OCP_N,
		.vr_addr = ADDR_VR_CPU0,
		.page_cnt = 2,
		.is_asserted = 0,
	},
	{
		.is_init = false,
		.gpio_num = PVDDCR_CPU1_OCP_N,
		.vr_addr = ADDR_VR_CPU1,
		.page_cnt = 2,
		.is_asserted = 0,
	},
	{
		.is_init = false,
		.gpio_num = PVDDCR_CPU0_PMALERT_N,
		.vr_addr = ADDR_VR_CPU0,
		.page_cnt = 2,
		.is_asserted = 0,
	},
	{
		.is_init = false,
		.gpio_num = PVDDCR_CPU1_PMALERT_N,
		.vr_addr = ADDR_VR_CPU1,
		.page_cnt = 2,
		.is_asserted = 0,
	},
	{
		.is_init = false,
		.gpio_num = PVDD11_S3_PMALERT_N,
		.vr_addr = ADDR_VR_PVDD11,
		.page_cnt = 1,
		.is_asserted = 0,
	},
};

add_sel_info event_work_items[] = {
	{
		.is_init = false,
		.gpio_num = RST_PLTRST_BIC_N,
		.event_type = PLTRST_ASSERT,
		.assert_type = EVENT_ASSERTED,
	},
	{
		.is_init = false,
		.gpio_num = FM_BIOS_POST_CMPLT_BIC_N,
		.event_type = POST_COMPLETED,
		.assert_type = EVENT_ASSERTED,
	},
	{
		.is_init = false,
		.gpio_num = FM_DBP_PRESENT_N,
		.event_type = HDT_PRSNT_ASSERT,
		.assert_type = EVENT_ASSERTED,
	},
	{
		.is_init = false,
		.gpio_num = FAST_PROCHOT_N,
		.event_type = FAST_PROCHOT_ASSERT,
		.assert_type = EVENT_ASSERTED,
	},
	{
		.is_init = false,
		.gpio_num = FM_CPU_BIC_THERMTRIP_N,
		.event_type = CPU_THERMAL_TRIP,
		.assert_type = EVENT_ASSERTED,
	},
	{
		.is_init = false,
		.gpio_num = FM_CPU_BIC_PROCHOT_LVT3_N,
		.event_type = SYS_THROTTLE,
		.assert_type = EVENT_ASSERTED,
	},
	{
		.is_init = false,
		.gpio_num = FM_HSC_TIMER_ALT_N,
		.event_type = HSC_OCP,
		.assert_type = EVENT_ASSERTED,
	},
	{
		.is_init = false,
		.gpio_num = IRQ_UV_DETECT_N,
		.event_type = P12V_STBY_UV,
		.assert_type = EVENT_ASSERTED,
	},
	{
		.is_init = false,
		.gpio_num = CPU_SMERR_BIC_N,
		.event_type = SYS_MANAGEMENT_ERROR,
		.assert_type = EVENT_ASSERTED,
	},
};

void init_event_work()
{
	for (int index = 0; index < ARRAY_SIZE(event_work_items); ++index) {
		if (event_work_items[index].is_init != true) {
			k_work_init_delayable(&event_work_items[index].add_sel_work,
					      addsel_work_handler);

			event_work_items[index].is_init = true;
		}
	}
}

void addsel_work_handler(struct k_work *work_item)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work_item);
	add_sel_info *work_info = CONTAINER_OF(dwork, add_sel_info, add_sel_work);
	struct pldm_addsel_data msg = { 0 };
	msg.event_type = work_info->event_type;
	msg.assert_type = work_info->assert_type;

	if (send_event_log_to_bmc(msg) != PLDM_SUCCESS) {
		LOG_ERR("Failed to send event log, event type: 0x%x, assert type: 0x%x",
			work_info->event_type, work_info->assert_type);
	};
}

static add_sel_info *find_event_work_items(uint8_t gpio_num)
{
	for (int index = 0; index < ARRAY_SIZE(event_work_items); ++index) {
		if (event_work_items[index].gpio_num == gpio_num) {
			return &event_work_items[index];
		}
	}

	return NULL;
}

/*
 *	TODO: I3C hub is supplied by DC power currently. When DC off, it can't be initialized.
 *  	  Therefore, BIC needs to implement a workaround to reinitialize the I3C hub during DC on.
 *        This part will be removed after the hardware design be modified.
 */
void reinit_i3c_hub()
{
	// i3c master initial
	I3C_MSG i3c_msg = { 0 };
	i3c_msg.bus = I3C_BUS_HUB;
	i3c_msg.target_addr = I3C_ADDR_HUB;

	const int rstdaa_count = 2;
	int ret = 0;

	for (int i = 0; i < rstdaa_count; i++) {
		ret = i3c_brocast_ccc(&i3c_msg, I3C_CCC_RSTDAA, I3C_BROADCAST_ADDR);
		if (ret != 0) {
			printf("Error to reset daa. count = %d\n", i);
		}
	}

	ret = i3c_brocast_ccc(&i3c_msg, I3C_CCC_SETAASA, I3C_BROADCAST_ADDR);
	if (ret != 0) {
		printf("Error to set daa\n");
	}

	i3c_attach(&i3c_msg);

	// Initialize I3C HUB
	if (!rg3mxxb12_i3c_mode_only_init(&i3c_msg, LDO_VOLT, 0xF0)) {
		printk("failed to initialize 1ou rg3mxxb12\n");
	}

	// Set FF/WF's EID
	send_cmd_to_dev_handler(NULL);
}

void set_ffwf_eid()
{
	// Set FF/WF's EID
	send_cmd_to_dev_handler(NULL);
}

void switch_i3c_dimm_mux_to_cpu()
{
	switch_i3c_dimm_mux(I3C_MUX_CPU_TO_DIMM);
}

static void PROC_FAIL_handler(struct k_work *work)
{
	/* if have not received kcs and post code, add FRB3 event log. */
	if ((get_kcs_ok() == false) && (get_4byte_postcode_ok() == false)) {
		LOG_ERR("FRB3 event assert");
		struct pldm_addsel_data msg = { 0 };
		msg.event_type = FRB3_TIMER_EXPIRE;
		msg.assert_type = EVENT_ASSERTED;
		if (PLDM_SUCCESS != send_event_log_to_bmc(msg)) {
			LOG_ERR("Failed to assert FRE3 event log.");
		};
	} else {
		/* Notification */
		LOG_INF("FRB3 checked pass");
	}
}

K_WORK_DELAYABLE_DEFINE(set_DC_on_5s_work, set_DC_on_delayed_status);
K_WORK_DEFINE(reinit_i3c_work, reinit_i3c_hub);
K_WORK_DEFINE(set_ffwf_eid_work, set_ffwf_eid);
K_WORK_DEFINE(switch_i3c_dimm_work, switch_i3c_dimm_mux_to_cpu);
K_WORK_DELAYABLE_DEFINE(PROC_FAIL_work, PROC_FAIL_handler);
K_WORK_DELAYABLE_DEFINE(ABORT_FRB2_WDT_THREAD, abort_frb2_wdt_thread);
K_WORK_DELAYABLE_DEFINE(read_pmic_critical_work, read_pmic_error_when_dc_off);

#define DC_ON_5_SECOND 5
#define PROC_FAIL_START_DELAY_SECOND 10
#define VR_EVENT_DELAY_MS 10
// The PMIC needs a total of 200ms from CAMP signal assertion to complete the write operation
#define READ_PMIC_CRITICAL_ERROR_MS 200
void ISR_DC_ON()
{
	set_DC_status(PWRGD_CPU_LVC3);

	bool dc_status = get_DC_status();

	if (dc_status) {
		k_work_schedule(&set_DC_on_5s_work, K_SECONDS(DC_ON_5_SECOND));
		k_work_submit(&set_ffwf_eid_work);
		k_work_submit(&switch_i3c_dimm_work);
		k_work_schedule_for_queue(&plat_work_q, &PROC_FAIL_work,
					  K_SECONDS(PROC_FAIL_START_DELAY_SECOND));
	} else {
		if (k_work_cancel_delayable(&PROC_FAIL_work) != 0) {
			LOG_ERR("Failed to cancel proc_fail delay work.");
		}
		reset_kcs_ok();
		reset_4byte_postcode_ok();

		set_DC_on_delayed_status();
		// Read PMIC error when DC off
		k_work_schedule(&read_pmic_critical_work, K_MSEC(READ_PMIC_CRITICAL_ERROR_MS));
	}
}

void ISR_POST_COMPLETE()
{
	set_post_status(FM_BIOS_POST_CMPLT_BIC_N);

	pal_check_sbrmi_command_code_length();

	if (get_post_status()) {
		set_tsi_threshold();
		disable_mailbox_completion_alert();
		enable_alert_signal();
		read_cpuid();
		LOG_INF("Post complete event assert");
		hw_event_register[12]++;

		add_sel_info *event_item = find_event_work_items(FM_BIOS_POST_CMPLT_BIC_N);
		if (event_item == NULL) {
			LOG_ERR("Fail to find event items, gpio num: 0x%x",
				FM_BIOS_POST_CMPLT_BIC_N);
			return;
		}

		k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work, K_NO_WAIT);
	} else {
		if (get_DC_status()) { // Host is reset
			k_work_submit(&switch_i3c_dimm_work);
		}
	}
}

void ISR_BMC_READY()
{
	sync_bmc_ready_pin();
}

void ISR_WF_BIC_READY()
{
	k_work_submit(&set_ffwf_eid_work);
}

static void SLP3_handler()
{
	if ((gpio_get(FM_CPU_BIC_SLP_S3_N) == GPIO_HIGH) &&
	    (gpio_get(PWRGD_CPU_LVC3) == GPIO_LOW)) {
		// add sel to bmc
		struct pldm_addsel_data msg = { 0 };
		msg.event_type = POWER_ON_SEQUENCE_FAIL;
		msg.assert_type = EVENT_ASSERTED;
		if (PLDM_SUCCESS != send_event_log_to_bmc(msg)) {
			LOG_ERR("Failed to send blade fail power-on sequence assert event log.");
		};
		hw_event_register[0]++;
	}
}

K_WORK_DELAYABLE_DEFINE(SLP3_work, SLP3_handler);

#define DETECT_VR_WDT_DELAY_S 10
void ISR_SLP3()
{
	if (gpio_get(FM_CPU_BIC_SLP_S3_N) == GPIO_HIGH) {
		LOG_INF("slp3");
		k_work_schedule_for_queue(&plat_work_q, &SLP3_work,
					  K_SECONDS(DETECT_VR_WDT_DELAY_S));
	} else {
		if (k_work_cancel_delayable(&SLP3_work) != 0) {
			LOG_ERR("Failed to cancel delayable work.");
		}
	}
}

void ISR_DBP_PRSNT()
{
	/* HDT_PRSENT_N --> Event trigger and save log when signal falling and also
    check RST_RSMRST_BMC_N / normal power. */
	if ((gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) && (get_DC_status())) {
		add_sel_info *event_item = find_event_work_items(FM_DBP_PRESENT_N);
		if (event_item == NULL) {
			LOG_ERR("Fail to find event items, gpio num: 0x%x, gpio status: 0x%x",
				FM_DBP_PRESENT_N, gpio_get(FM_DBP_PRESENT_N));
			return;
		}

		if ((gpio_get(FM_DBP_PRESENT_N) == GPIO_HIGH)) {
			LOG_INF("ISR_DBP_PRSNT deassert");
			event_item->assert_type = EVENT_DEASSERTED;
		} else {
			LOG_INF("ISR_DBP_PRSNT assert");
			hw_event_register[1]++;
			event_item->assert_type = EVENT_ASSERTED;
		}

		k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work, K_NO_WAIT);
	}
}

void ISR_MB_THROTTLE()
{
	/* FAST_PROCHOT_N glitch workaround
	 * FAST_PROCHOT_N has a glitch and causes BIC to record MB_throttle deassertion SEL.
	 * Ignore this by checking whether MB_throttle is asserted before recording the deassertion.
	 */
	static bool is_mb_throttle_assert = false;
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH && get_DC_status()) {
		add_sel_info *event_item = find_event_work_items(FAST_PROCHOT_N);
		if (event_item == NULL) {
			LOG_ERR("Fail to find event items, gpio num: 0x%x, gpio status: 0x%x",
				FAST_PROCHOT_N, gpio_get(FAST_PROCHOT_N));
			return;
		}

		if ((gpio_get(FAST_PROCHOT_N) == GPIO_HIGH) && (is_mb_throttle_assert == true)) {
			LOG_INF("ISR_MB_THROTTLE deassert");
			is_mb_throttle_assert = false;
			event_item->assert_type = EVENT_DEASSERTED;
			k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work,
						  K_NO_WAIT);
		} else if ((gpio_get(FAST_PROCHOT_N) == GPIO_LOW) &&
			   (is_mb_throttle_assert == false)) {
			LOG_INF("ISR_MB_THROTTLE assert");
			hw_event_register[2]++;
			is_mb_throttle_assert = true;
			event_item->assert_type = EVENT_ASSERTED;
			k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work,
						  K_NO_WAIT);
		}
	}
}

void ISR_SOC_THMALTRIP()
{
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		LOG_INF("ISR_SOC_THMALTRIP assert");
		hw_event_register[3]++;
		add_sel_info *event_item = find_event_work_items(FM_CPU_BIC_THERMTRIP_N);
		if (event_item == NULL) {
			LOG_ERR("Fail to find event items, gpio num: 0x%x", FM_CPU_BIC_THERMTRIP_N);
			return;
		}

		k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work, K_NO_WAIT);
	}
}

void ISR_SYS_THROTTLE()
{
	/* Same as MB_THROTTLE, glitch of FAST_PROCHOT_N will affect FM_CPU_BIC_PROCHOT_LVT3_N.
	 * Ignore the fake event by checking whether SYS_throttle is asserted before recording the deassertion.
	 */
	static bool is_sys_throttle_assert = false;
	if ((gpio_get(RST_CPU_RESET_BIC_N) == GPIO_HIGH) &&
	    (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH)) {
		add_sel_info *event_item = find_event_work_items(FM_CPU_BIC_PROCHOT_LVT3_N);
		if (event_item == NULL) {
			LOG_ERR("Fail to find event items, gpio num: 0x%x, gpio status: 0x%x",
				FM_CPU_BIC_PROCHOT_LVT3_N, gpio_get(FM_CPU_BIC_PROCHOT_LVT3_N));
			return;
		}

		if ((gpio_get(FM_CPU_BIC_PROCHOT_LVT3_N) == GPIO_HIGH) &&
		    (is_sys_throttle_assert == true)) {
			LOG_INF("ISR_SYS_THROTTLE deassert");
			is_sys_throttle_assert = false;
			event_item->assert_type = EVENT_DEASSERTED;
			k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work,
						  K_NO_WAIT);
		} else if ((gpio_get(FM_CPU_BIC_PROCHOT_LVT3_N) == GPIO_LOW) &&
			   (is_sys_throttle_assert == false)) {
			LOG_INF("ISR_SYS_THROTTLE assert");
			hw_event_register[4]++;
			is_sys_throttle_assert = true;
			event_item->assert_type = EVENT_ASSERTED;
			k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work,
						  K_NO_WAIT);
		}
	}
}

void ISR_HSC_OC()
{
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		add_sel_info *event_item = find_event_work_items(FM_HSC_TIMER_ALT_N);
		if (event_item == NULL) {
			LOG_ERR("Fail to find event items, gpio num: 0x%x, gpio status: 0x%x",
				FM_HSC_TIMER_ALT_N, gpio_get(FM_HSC_TIMER_ALT_N));
			return;
		}

		if (gpio_get(FM_HSC_TIMER_ALT_N) == GPIO_HIGH) {
			LOG_INF("ISR_HSC_OC deassert");
			event_item->assert_type = EVENT_DEASSERTED;
		} else {
			LOG_INF("ISR_HSC_OC assert");
			hw_event_register[5]++;
			event_item->assert_type = EVENT_ASSERTED;
		}

		k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work, K_NO_WAIT);
	}
}

void init_vr_event_work()
{
	for (int index = 0; index < ARRAY_SIZE(vr_event_work_item); ++index) {
		if (vr_event_work_item[index].is_init != true) {
			k_work_init_delayable(&vr_event_work_item[index].add_sel_work,
					      process_vr_pmalert_ocp_sel);

			vr_event_work_item[index].is_init = true;
		}
	}
}

void process_vr_pmalert_ocp_sel(struct k_work *work_item)
{
	int ret = -1;
	uint8_t retry = 5;

	struct k_work_delayable *dwork = k_work_delayable_from_work(work_item);
	add_vr_sel_info *work_info = CONTAINER_OF(dwork, add_vr_sel_info, add_sel_work);

	I2C_MSG msg = { 0 };
	msg.bus = I2C_BUS4;
	msg.target_addr = work_info->vr_addr;

	for (int page = 0; page < work_info->page_cnt; ++page) {

		set_vr_monitor_status(false);
		// wait 10ms for vr monitor stop
		k_msleep(10);

		/* set page for power rail */
		msg.tx_len = 2;
		msg.data[0] = PMBUS_PAGE;
		msg.data[1] = page;
		if (i2c_master_write(&msg, retry)) {
			LOG_ERR("process_vr_pmalert_ocp_sel, set page fail");
			continue;
		}

		/* read STATUS_WORD */
		msg.tx_len = 1;
		msg.rx_len = 2;
		msg.data[0] = PMBUS_STATUS_WORD;
		ret = i2c_master_read(&msg, retry);
		if (ret != 0) {
			LOG_ERR("Get vr status word fail, bus: 0x%x, addr: 0x%x", msg.bus,
				msg.target_addr);
			continue;
		}

		uint8_t status_word_lsb = msg.data[0];
		uint8_t status_word_msb = msg.data[1];

		/* add a filter to avoid false alert from TI VR */
		uint8_t vr_dev = VR_DEVICE_UNKNOWN;
		if (plat_pldm_sensor_get_vr_dev(&vr_dev) != GET_VR_DEV_SUCCESS) {
			LOG_ERR("Unable to change the VR device due to its unknown status.");
			continue;
		}

		/* 1. check if trigger reason is IOUT fault */
		if (((status_word_msb & VR_IOUT_FAULT_MASK) >> 6) == 1) {
			/* 2. IOUT fault, check if OC Warning is set */
			msg.tx_len = 1;
			msg.rx_len = 1;
			msg.data[0] = PMBUS_STATUS_IOUT;
			ret = i2c_master_read(&msg, retry);
			if (ret != 0) {
				LOG_ERR("Get vr status iout fail, bus: 0x%x, addr: 0x%x",
					msg.bus, msg.target_addr);
				continue;
			}
			if (((msg.data[0] & VR_TPS_OCW_MASK) >> 5) == 1) {
				/* only OC Warn is triggered, skip to prevent false event */
				continue;
			}
		}

		set_vr_monitor_status(true);

		struct pldm_addsel_data sel_msg = { 0 };

		switch (work_info->gpio_num) {
		case PVDDCR_CPU0_OCP_N:
			sel_msg.event_type = VR_FAULT;
			sel_msg.event_data_1 = PVDDCR_CPU0 + page;
			break;
		case PVDDCR_CPU1_OCP_N:
			sel_msg.event_type = VR_FAULT;
			sel_msg.event_data_1 = PVDDCR_CPU1 + page;
			break;
		case PVDDCR_CPU0_PMALERT_N:
			sel_msg.event_type = PMALERT_ASSERT;
			sel_msg.event_data_1 = PVDDCR_CPU0 + page;
			break;
		case PVDDCR_CPU1_PMALERT_N:
			sel_msg.event_type = PMALERT_ASSERT;
			sel_msg.event_data_1 = PVDDCR_CPU1 + page;
			break;
		case PVDD11_S3_PMALERT_N:
			sel_msg.event_type = PMALERT_ASSERT;
			sel_msg.event_data_1 = PVDD11_S3;
			break;
		default:
			break;
		}

		if (gpio_get(work_info->gpio_num) == GPIO_HIGH) {
			if (work_info->is_asserted & BIT(page)) {
				sel_msg.assert_type = EVENT_DEASSERTED;
				work_info->is_asserted &= ~BIT(page);
			} else {
				/* It's not asserted in previous status */
				continue;
			}
		} else {
			sel_msg.assert_type = EVENT_ASSERTED;

			if (((status_word_lsb & VR_FAULT_STATUS_LSB_MASK) == 0) &&
			    ((status_word_msb & VR_FAULT_STATUS_MSB_MASK) == 0)) {
				/* No OV/OC/UV/OT event happened, check next rail */
				continue;
			} else {
				work_info->is_asserted |= BIT(page);
			}
		}

		sel_msg.event_data_2 = status_word_msb;
		sel_msg.event_data_3 = status_word_lsb;

		if (PLDM_SUCCESS != send_event_log_to_bmc(sel_msg)) {
			LOG_ERR("Failed to assert VR event log.");
		};
	}
}

void ISR_PVDDCR_CPU0_OCP()
{
	if (get_DC_status() == true) {
		LOG_ERR("PVDDCR CPU0 OCP event triggered");
		hw_event_register[7]++;
		k_work_schedule_for_queue(&plat_work_q, &vr_event_work_item[0].add_sel_work,
					  K_MSEC(VR_EVENT_DELAY_MS));
	}
}

void ISR_PVDDCR_CPU1_OCP()
{
	if (get_DC_status() == true) {
		LOG_ERR("PVDDCR CPU1 OCP event triggered");
		hw_event_register[6]++;
		k_work_schedule_for_queue(&plat_work_q, &vr_event_work_item[1].add_sel_work,
					  K_MSEC(VR_EVENT_DELAY_MS));
	}
}

void ISR_PVDD11_S3_PMALERT()
{
	if (get_DC_status() == true) {
		LOG_ERR("PVDD11 S3 PMALERT event triggered");
		hw_event_register[8]++;
		k_work_schedule_for_queue(&plat_work_q, &vr_event_work_item[4].add_sel_work,
					  K_MSEC(VR_EVENT_DELAY_MS));
	}
}

void ISR_UV_DETECT()
{
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		add_sel_info *event_item = find_event_work_items(IRQ_UV_DETECT_N);
		if (event_item == NULL) {
			LOG_ERR("Fail to find event items, gpio num: 0x%x, gpio status: 0x%x",
				IRQ_UV_DETECT_N, gpio_get(IRQ_UV_DETECT_N));
			return;
		}

		if (gpio_get(IRQ_UV_DETECT_N) == GPIO_HIGH) {
			LOG_INF("ISR_UV_DETECT deassert");
			event_item->assert_type = EVENT_DEASSERTED;
		} else {
			LOG_INF("ISR_UV_DETECT assert");
			hw_event_register[9]++;
			event_item->assert_type = EVENT_ASSERTED;
		}

		k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work, K_NO_WAIT);
	}
}

void IST_PLTRST()
{
	LOG_INF("IST_PLTRST assert");
	hw_event_register[10]++;

	add_sel_info *event_item = find_event_work_items(RST_PLTRST_BIC_N);
	if (event_item == NULL) {
		LOG_ERR("Fail to find event items, gpio num: 0x%x", RST_PLTRST_BIC_N);
		return;
	}

	// Reset WDT
	k_work_schedule_for_queue(&plat_work_q, &ABORT_FRB2_WDT_THREAD, K_NO_WAIT);

	k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work, K_NO_WAIT);
}

static void APML_ALERT_handler(struct k_work *work)
{
	uint8_t status = 0;
	if (apml_read_byte(apml_get_bus(), SB_RMI_ADDR, SBRMI_STATUS, &status))
		LOG_ERR("Failed to read RMI status.");

	if ((status & 0x02) && (apml_write_byte(apml_get_bus(), SB_RMI_ADDR, SBRMI_STATUS, 0x02)))
		LOG_ERR("Failed to clear SwAlertSts.");

	pldm_msg translated_msg = { 0 };
	uint8_t bmc_bus = I2C_BUS_BMC;
	uint8_t bmc_interface = pal_get_bmc_interface();

	if (bmc_interface == BMC_INTERFACE_I3C) {
		bmc_bus = I3C_BUS_BMC;
		translated_msg.ext_params.type = MCTP_MEDIUM_TYPE_TARGET_I3C;
		translated_msg.ext_params.i3c_ext_params.addr = I3C_STATIC_ADDR_BMC;
		translated_msg.ext_params.ep = MCTP_EID_BMC;
	} else {
		bmc_bus = I2C_BUS_BMC;
		translated_msg.ext_params.type = MCTP_MEDIUM_TYPE_SMBUS;
		translated_msg.ext_params.smbus_ext_params.addr = I2C_ADDR_BMC;
		translated_msg.ext_params.ep = MCTP_EID_BMC;
	}

	translated_msg.hdr.pldm_type = PLDM_TYPE_OEM;
	translated_msg.hdr.cmd = PLDM_OEM_WRITE_FILE_IO;
	translated_msg.hdr.rq = 1;

	struct pldm_oem_write_file_io_req *ptr = (struct pldm_oem_write_file_io_req *)malloc(
		sizeof(struct pldm_oem_write_file_io_req) + CPUID_SIZE);

	if (!ptr) {
		LOG_ERR("Failed to allocate memory for APML alert event.");
		return;
	}

	ptr->cmd_code = APML_ALERT;
	ptr->data_length = CPUID_SIZE;
	const uint8_t *cpuid_ptr = get_cpuid();
	memcpy(ptr->messages, cpuid_ptr, CPUID_SIZE);

	translated_msg.buf = (uint8_t *)ptr;
	translated_msg.len = sizeof(struct pldm_oem_write_file_io_req) + CPUID_SIZE;

	uint8_t resp_len = sizeof(struct pldm_oem_write_file_io_resp);
	uint8_t rbuf[resp_len];

	if (!mctp_pldm_read(find_mctp_by_bus(bmc_bus), &translated_msg, rbuf, resp_len)) {
		LOG_ERR("APML alert mctp_pldm_read fail");
		goto exit;
	}

	struct pldm_oem_write_file_io_resp *resp = (struct pldm_oem_write_file_io_resp *)rbuf;
	if (resp->completion_code != PLDM_SUCCESS) {
		LOG_ERR("Check reponse completion code fail %x", resp->completion_code);
		goto exit;
	}

exit:
	SAFE_FREE(ptr);

	return;
}

K_WORK_DELAYABLE_DEFINE(APML_ALERT_work, APML_ALERT_handler);
void ISR_APML_ALERT()
{
	hw_event_register[11]++;
	LOG_INF("APML_ALERT detected");
	k_work_schedule_for_queue(&plat_work_q, &APML_ALERT_work, K_NO_WAIT);
}

void ISR_PVDDCR_CPU0_PMALERT()
{
	if (get_DC_status() == true) {
		LOG_ERR("PVDDCR CPU0 PMALERT event triggered");
		k_work_schedule_for_queue(&plat_work_q, &vr_event_work_item[2].add_sel_work,
					  K_MSEC(VR_EVENT_DELAY_MS));
	}
}

void ISR_PVDDCR_CPU1_PMALERT()
{
	if (get_DC_status() == true) {
		LOG_ERR("PVDDCR CPU1 PMALERT event triggered");
		k_work_schedule_for_queue(&plat_work_q, &vr_event_work_item[3].add_sel_work,
					  K_MSEC(VR_EVENT_DELAY_MS));
	}
}

void ISR_CPU_SMERR_BIC()
{
	if (get_DC_status() == true) {
		add_sel_info *event_item = find_event_work_items(CPU_SMERR_BIC_N);
		if (event_item == NULL) {
			LOG_ERR("Fail to find event items, gpio num: 0x%x", CPU_SMERR_BIC_N);
			return;
		}

		if ((gpio_get(CPU_SMERR_BIC_N) == GPIO_LOW)) {
			LOG_INF("ISR_CPU_SMERR_BIC assert");
			event_item->assert_type = EVENT_ASSERTED;
		}
		k_work_schedule_for_queue(&plat_work_q, &event_item->add_sel_work, K_NO_WAIT);
	}
}
