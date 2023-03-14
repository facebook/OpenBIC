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
#include "plat_def.h"
#include "plat_pmic.h"
#include "plat_apml.h"
#include "util_worker.h"

LOG_MODULE_REGISTER(plat_isr);

void ISR_POST_COMPLETE()
{
	set_post_status(FM_BIOS_POST_CMPLT_BIC_N);
	if (get_post_status()) {
		set_tsi_threshold();
		read_cpuid();
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
K_WORK_DELAYABLE_DEFINE(PROC_FAIL_work, PROC_FAIL_handler);
K_WORK_DELAYABLE_DEFINE(read_pmic_critical_work, read_pmic_error_when_dc_off);
#define DC_ON_5_SECOND 5
#define PROC_FAIL_START_DELAY_SECOND 10
#define READ_PMIC_CRITICAL_ERROR_MS 100
void ISR_DC_ON()
{
	set_DC_status(PWRGD_CPU_LVC3);
	if (get_DC_status() == true) {
		reset_pcc_buffer();
		k_work_schedule(&set_DC_on_5s_work, K_SECONDS(DC_ON_5_SECOND));
		k_work_schedule_for_queue(&plat_work_q, &PROC_FAIL_work,
					  K_SECONDS(PROC_FAIL_START_DELAY_SECOND));
	} else {
		if (k_work_cancel_delayable(&PROC_FAIL_work) != 0) {
			LOG_ERR("Failed to cancel proc_fail delay work.");
		}
		reset_kcs_ok();
		reset_4byte_postcode_ok();

		if (k_work_cancel_delayable(&set_DC_on_5s_work) != 0) {
			LOG_ERR("Failed to cancel set dc on delay work.");
		}
		set_DC_on_delayed_status();

		k_work_schedule(&read_pmic_critical_work, K_MSEC(READ_PMIC_CRITICAL_ERROR_MS));
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
#define DETECT_VR_WDT_DELAY_S 10
void ISR_SLP3()
{
	if (gpio_get(FM_CPU_BIC_SLP_S3_N) == GPIO_HIGH) {
		LOG_INF("slp3");
		k_work_schedule_for_queue(&plat_work_q, &SLP3_work,
					  K_SECONDS(DETECT_VR_WDT_DELAY_S));
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
		sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
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
		if (gpio_get(PWRGD_CPU_LVC3) == GPIO_LOW) {
			return;
		} else {
			if ((gpio_get(IRQ_HSC_ALERT1_N) == GPIO_HIGH) &&
			    (is_hsc_throttle_assert == true)) {
				sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
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
	/* FAST_PROCHOT_N glitch workaround
	 * FAST_PROCHOT_N has a glitch and causes BIC to record MB_throttle deassertion SEL.
	 * Ignore this by checking whether MB_throttle is asserted before recording the deassertion.
	 */
	static bool is_mb_throttle_assert = false;
	common_addsel_msg_t sel_msg;
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH && get_DC_status()) {
		if ((gpio_get(FAST_PROCHOT_N) == GPIO_HIGH) && (is_mb_throttle_assert == true)) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
			is_mb_throttle_assert = false;
		} else if ((gpio_get(FAST_PROCHOT_N) == GPIO_LOW) &&
			   (is_mb_throttle_assert == false)) {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			is_mb_throttle_assert = true;
		} else {
			return;
		}
		sel_msg.InF_target = BMC_IPMB;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_FIRMWAREASSERT;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!common_add_sel_evt_record(&sel_msg)) {
			LOG_ERR("Failed to add MB Throttle sel.");
		}
	}
}

void ISR_SOC_THMALTRIP()
{
	common_addsel_msg_t sel_msg;
	if (gpio_get(RST_CPU_RESET_BIC_N) == GPIO_HIGH) {
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
	/* Same as MB_THROTTLE, glitch of FAST_PROCHOT_N will affect FM_CPU_BIC_PROCHOT_LVT3_N.
	 * Ignore the fake event by checking whether SYS_throttle is asserted before recording the deassertion.
	 */
	static bool is_sys_throttle_assert = false;
	common_addsel_msg_t sel_msg;
	if ((gpio_get(RST_CPU_RESET_BIC_N) == GPIO_HIGH) &&
	    (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH)) {
		if ((gpio_get(FM_CPU_BIC_PROCHOT_LVT3_N) == GPIO_HIGH) &&
		    (is_sys_throttle_assert == true)) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
			is_sys_throttle_assert = false;
		} else if ((gpio_get(FM_CPU_BIC_PROCHOT_LVT3_N) == GPIO_LOW) &&
			   (is_sys_throttle_assert == false)) {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			is_sys_throttle_assert = true;
		} else {
			return;
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
			LOG_ERR("Failed to add HSC OC sel.");
		}
	}
}

static void add_vr_ocp_sel(uint8_t gpio_num, uint8_t vr_num)
{
	static uint8_t is_vr_ocp_assert = 0;
	common_addsel_msg_t sel_msg;
	if ((gpio_get(gpio_num) == GPIO_HIGH) && (is_vr_ocp_assert & BIT(vr_num))) {
		sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
		is_vr_ocp_assert &= ~BIT(vr_num);
	} else if ((gpio_get(gpio_num) == GPIO_LOW) && !(is_vr_ocp_assert & BIT(vr_num))) {
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		is_vr_ocp_assert |= BIT(vr_num);
	} else {
		return;
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

void ISR_UV_DETECT()
{
	common_addsel_msg_t sel_msg;
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		if (gpio_get(IRQ_UV_DETECT_N) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
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

void IST_PLTRST()
{
	reset_tsi_status();
}

#define FATAL_ERROR_DELAY_MSECOND 500
typedef struct {
	struct k_work_delayable work;
	uint8_t ras_status;
} fatal_error_work_info;

static void send_apml_alert(struct k_work *work)
{
	fatal_error_work_info *work_info = CONTAINER_OF(work, fatal_error_work_info, work);
	if (get_DC_status()) {
		LOG_INF("Send apml alert to bmc.");
		send_apml_alert_to_bmc(work_info->ras_status);
	}
	SAFE_FREE(work_info);
}

void ISR_APML_ALERT()
{
	uint8_t ras_status;
	if (apml_read_byte(APML_BUS, SB_RMI_ADDR, SBRMI_RAS_STATUS, &ras_status)) {
		LOG_ERR("Failed to read RAS status.");
		return;
	}

	if (ras_status & 0x0F) {
		LOG_INF("Fatal error happened, ras_status 0x%x.", ras_status);
		fatal_error_happened();

		if ((ras_status & 0x01) &&
		    (apml_write_byte(APML_BUS, SB_RMI_ADDR, SBRMI_RAS_STATUS, 0x01)))
			LOG_ERR("Failed to clear fatal error.");

		uint8_t status;
		if (apml_read_byte(APML_BUS, SB_RMI_ADDR, SBRMI_STATUS, &status))
			LOG_ERR("Failed to read RMI status.");

		if ((status & 0x02) && (apml_write_byte(APML_BUS, SB_RMI_ADDR, SBRMI_STATUS, 0x02)))
			LOG_ERR("Failed to clear SwAlertSts.");

		fatal_error_work_info *delay_work = malloc(sizeof(fatal_error_work_info));
		if (delay_work == NULL) {
			LOG_ERR("Failed to allocate delay_job.");
			return;
		}
		memset(delay_work, 0, sizeof(fatal_error_work_info));

		delay_work->ras_status = ras_status;
		k_work_init_delayable(&(delay_work->work), send_apml_alert);
		k_work_schedule(&(delay_work->work), K_MSEC(FATAL_ERROR_DELAY_MSECOND));
	}
}
