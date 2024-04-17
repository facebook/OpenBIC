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
#include "plat_mctp.h"
#include "power_status.h"
#include "ipmi.h"
#include "pldm.h"
#include "ssif.h"
#include "sbmr.h"
#include "util_worker.h"
#include "libutil.h"
#include "libipmi.h"
#include "logging/log.h"

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

void ISR_E1S_ALERT()
{
	isr_dbg_print(INA230_E1S_ALERT_L);

	common_addsel_msg_t sel_msg;
	if (gpio_get(RUN_POWER_PG) == GPIO_HIGH) {
		if (gpio_get(INA230_E1S_ALERT_L) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		}

		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_OEM;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_EVENT_OFFSET_SYS_E1S_ALERT;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			LOG_ERR("System Throttle addsel fail");
		}
	}
}

void ISR_RTC_ALERT()
{
	isr_dbg_print(I2C_2_CPU_ALERT_R_L);
}

void ISR_GPIOB7()
{
	isr_dbg_print(FPGA_CPU_BOOT_DONE);
}

void ISR_GPIOA5()
{
	isr_dbg_print(FPGA_WATCH_DOG_TIMER0_L);
}

void ISR_GPIOA6()
{
	isr_dbg_print(FPGA_WATCH_DOG_TIMER1_L);
}

void ISR_GPIOB0()
{
	isr_dbg_print(FPGA_WATCH_DOG_TIMER2_L);
}

void ISR_HSC_OC()
{
	isr_dbg_print(FM_HSC_TIMER);

	common_addsel_msg_t sel_msg;
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

void ISR_GPIOC1()
{
	isr_dbg_print(IRQ_I2C_IO_LVC_STBY_ALRT_L);
}

void ISR_GPIOC3()
{
	isr_dbg_print(BIC_I2C_0_FPGA_ALERT_L);
}

void ISR_GPIOC4()
{
	isr_dbg_print(BIC_I2C_1_FPGA_ALERT_L);
}

void ISR_GPIOD0()
{
	isr_dbg_print(PWRBTN_L);
}

static void PROC_FAIL_handler(struct k_work *work)
{
	/* if have not received ssif and post code, add FRB3 event log. */
	if ((get_ssif_ok() == false) && (sbmr_get_9byte_postcode_ok() == false)) {
		common_addsel_msg_t sel_msg;
		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.sensor_number = SENSOR_NUM_PROC_FAIL;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.event_data1 = IPMI_EVENT_OFFSET_PROCESSOR_FRB3;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			LOG_ERR("Failed to assert FRE3 event log.");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(set_DC_on_5s_work, set_DC_on_delayed_status);
K_WORK_DELAYABLE_DEFINE(PROC_FAIL_work, PROC_FAIL_handler);
#define DC_ON_5_SECOND 5
#define PROC_FAIL_START_DELAY_SECOND 10
void ISR_PWRGD_CPU()
{
	isr_dbg_print(RUN_POWER_PG);
	set_CPU_power_status(RUN_POWER_PG);
	set_DC_status(RUN_POWER_PG); // Grace don't have DC status pin to BIC
	send_gpio_interrupt(RUN_POWER_PG);
	set_post_complete(false); //temporary set cause of postcomplete gpio not ready

	if (CPU_power_good() == true) {
		reset_sbmr_postcode_buffer();
		k_work_schedule(&set_DC_on_5s_work, K_SECONDS(DC_ON_5_SECOND));
		k_work_schedule_for_queue(&plat_work_q, &PROC_FAIL_work,
					  K_SECONDS(PROC_FAIL_START_DELAY_SECOND));
	} else {
		if (k_work_cancel_delayable(&PROC_FAIL_work) != 0) {
			LOG_ERR("Failed to cancel proc_fail delay work.");
		}
		reset_ssif_ok();
		sbmr_reset_9byte_postcode_ok();

		if (k_work_cancel_delayable(&set_DC_on_5s_work) != 0) {
			LOG_ERR("Failed to cancel set dc on delay work.");
		}
		set_DC_on_delayed_status();
	}
}

void ISR_GPIOE2()
{
	isr_dbg_print(SPI_BMC_FPGA_INT_L);
}

void ISR_HSC_THROTTLE()
{
	isr_dbg_print(IRQ_HSC_ALERT1_L);

	common_addsel_msg_t sel_msg;
	static bool is_hsc_throttle_assert = false; // Flag for filt out fake alert
	if (gpio_get(RUN_POWER_PG) == GPIO_LOW) {
		return;
	} else {
		if ((gpio_get(IRQ_HSC_ALERT1_L) == GPIO_HIGH) && (is_hsc_throttle_assert == true)) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
			is_hsc_throttle_assert = false;
		} else if ((gpio_get(IRQ_HSC_ALERT1_L) == GPIO_LOW) &&
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
			LOG_ERR("Failed to add HSC Throttle sel.");
		}
	}
}

void ISR_GPIOE4()
{
	isr_dbg_print(I2C_SENSOR_LVC_ALERT_L);
}

void ISR_GPIOE5()
{
	isr_dbg_print(INA_CRIT_ALERT1_L);
}

void ISR_GPIOE6()
{
	isr_dbg_print(RUN_POWER_EN);
}

void ISR_GPIOE7()
{
	isr_dbg_print(SPI_HOST_TPM_RST_L);
}

void ISR_CPU_HIGHTEMP()
{
	isr_dbg_print(THERM_WARN_CPU1_L_3V3);

	common_addsel_msg_t sel_msg;
	if (gpio_get(RUN_POWER_PG) == GPIO_HIGH) {
		if (gpio_get(THERM_WARN_CPU1_L_3V3) == GPIO_HIGH) {
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
			LOG_ERR("System Throttle addsel fail");
		}
	}
}

void ISR_CPU_OVERTEMP()
{
	isr_dbg_print(THERM_OVERT_CPU1_L_3V3);

	common_addsel_msg_t sel_msg;
	static bool is_thermal_trip_assert = 0;
	if (gpio_get(THERM_OVERT_CPU1_L_3V3) == GPIO_LOW) {
		if ((get_post_status() == true) && (is_thermal_trip_assert == false)) {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			is_thermal_trip_assert = true;
		}
	} else if (gpio_get(THERM_OVERT_CPU1_L_3V3) && (is_thermal_trip_assert == true)) {
		sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
		is_thermal_trip_assert = false;
	} else {
		return;
	}

	sel_msg.InF_target = PLDM;
	sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
	sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
	sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_THERMAL_TRIP;
	sel_msg.event_data2 = 0xFF;
	sel_msg.event_data3 = 0xFF;
	if (!mctp_add_sel_to_ipmi(&sel_msg)) {
		LOG_ERR("Thermal trip addsel fail");
	}
}

void ISR_CPU_FAULT_ALERT()
{
	isr_dbg_print(RUN_POWER_FAULT_L);
	send_gpio_interrupt(RUN_POWER_FAULT_L);

	common_addsel_msg_t sel_msg;
	if (gpio_get(RUN_POWER_PG) == GPIO_HIGH) {
		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.sensor_number = SENSOR_NUM_CPU_FAULT;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		sel_msg.event_data1 = 0xFF;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			LOG_ERR("CPU fault addsel fail");
		}
	}
}

void ISR_GPIOF3()
{
	isr_dbg_print(SENSOR_AIR0_THERM_L);
}

void ISR_GPIOF4()
{
	isr_dbg_print(SENSOR_AIR1_THERM_L);
}

void ISR_GPIOF6()
{
	isr_dbg_print(THERM_BB_OVERT_L);
}

void ISR_GPIOF7()
{
	isr_dbg_print(THERM_BB_WARN_L);
}

void ISR_MB_THROTTLE()
{
	isr_dbg_print(FAST_PROCHOT_L);

	/* FAST_PROCHOT_L glitch workaround
	 * FAST_PROCHOT_L has a glitch and causes BIC to record MB_throttle deassertion SEL.
	 * Ignore this by checking whether MB_throttle is asserted before recording the deassertion.
	 */
	static bool is_mb_throttle_assert = false;
	common_addsel_msg_t sel_msg;
	if (get_DC_status()) {
		if ((gpio_get(FAST_PROCHOT_L) == GPIO_HIGH) && (is_mb_throttle_assert == true)) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
			is_mb_throttle_assert = false;
		} else if ((gpio_get(FAST_PROCHOT_L) == GPIO_LOW) &&
			   (is_mb_throttle_assert == false)) {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			is_mb_throttle_assert = true;
		} else {
			return;
		}
		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_FIRMWAREASSERT;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			LOG_ERR("Failed to add MB Throttle sel.");
		}
	}
}

void ISR_GPIOH0()
{
	isr_dbg_print(SENSOR_AIR0_ALERT_L);
}

void ISR_GPIOH1()
{
	isr_dbg_print(SENSOR_AIR1_ALERT_L);
}

void ISR_SYS_THROTTLE()
{
	isr_dbg_print(CPU_BIC_PROCHOT_L);

	/* Same as MB_THROTTLE, glitch of FAST_PROCHOT_L will affect CPU_BIC_PROCHOT_L.
	 * Ignore the fake event by checking whether SYS_throttle is asserted before recording the deassertion.
	 */
	static bool is_sys_throttle_assert = false;
	common_addsel_msg_t sel_msg;
	if (gpio_get(RUN_POWER_PG) == GPIO_HIGH) {
		if ((gpio_get(CPU_BIC_PROCHOT_L) == GPIO_HIGH) &&
		    (is_sys_throttle_assert == true)) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
			is_sys_throttle_assert = false;
		} else if ((gpio_get(CPU_BIC_PROCHOT_L) == GPIO_LOW) &&
			   (is_sys_throttle_assert == false)) {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			is_sys_throttle_assert = true;
		} else {
			return;
		}
		sel_msg.InF_target = PLDM;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_THROTTLE;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!mctp_add_sel_to_ipmi(&sel_msg)) {
			LOG_ERR("Failed to add System Throttle sel.");
		}
	}
}
