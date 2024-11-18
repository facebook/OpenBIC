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
#include "plat_class.h"
#include "plat_power_status.h"
#include "plat_i2c.h"
#include "plat_fru.h"
#include "power_status.h"
#include "ipmi.h"
#include "pldm.h"
#include "ssif.h"
#include "sbmr.h"
#include "pmbus.h"
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

	struct ipmi_storage_add_sel_req add_sel_msg = { 0 };
	if (gpio_get(RUN_POWER_PG) == GPIO_HIGH) {
		if (gpio_get(INA230_E1S_ALERT_L) == GPIO_HIGH) {
			add_sel_msg.event.event_dir_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
		} else {
			add_sel_msg.event.event_dir_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		}

		add_sel_msg.event.sensor_type = IPMI_OEM_SENSOR_TYPE_OEM;
		add_sel_msg.event.sensor_num = SENSOR_NUM_SYSTEM_STATUS;
		add_sel_msg.event.event_data[0] = IPMI_EVENT_OFFSET_SYS_E1S_ALERT;
		add_sel_msg.event.event_data[1] = 0xFF;
		add_sel_msg.event.event_data[2] = 0xFF;
		if (!mctp_add_sel_to_ipmi(&add_sel_msg, ADD_COMMON_SEL)) {
			LOG_ERR("Failed to add E1S alert sel.");
		}
	}
}

void ISR_RTC_ALERT()
{
	isr_dbg_print(I2C_2_CPU_ALERT_R_L);
}

#define SATMC_POLL_INTERVAL 4000 // ms
static bool is_satmc_poll_thread_running = false;
static void satmc_access_poll_handler()
{
	while (1) {
		if (CPU_power_good() == false)
			break;
		/* keep polling till post end cause CPU may reset itself during boot up */
		if (get_post_status() == true)
			break;
		LOG_INF("Patting SatMC...");
		satmc_status_update();
		k_msleep(SATMC_POLL_INTERVAL);
	}
	is_satmc_poll_thread_running = false;
}

#define SATMC_POLL_THREAD_STACK_SIZE 1024
struct k_thread satmc_poll_thread;
K_KERNEL_STACK_MEMBER(satmc_poll_thread_stack, SATMC_POLL_THREAD_STACK_SIZE);
void start_satmc_access_poll()
{
	if (is_satmc_poll_thread_running) {
		LOG_WRN("SatMC poll thread is already running!");
		return;
	}
	is_satmc_poll_thread_running = true;

	k_thread_create(&satmc_poll_thread, satmc_poll_thread_stack,
			K_THREAD_STACK_SIZEOF(satmc_poll_thread_stack), satmc_access_poll_handler,
			NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&satmc_poll_thread, "satmc_poll_thread");
}

void check_host_reboot_status()
{
	if (get_DC_status() == true) {
		LOG_WRN("Host reboot detected!");
		start_satmc_access_poll();
	}
}

#define POST_DOWN_2_SECOND 2
K_WORK_DELAYABLE_DEFINE(check_host_reboot_work, check_host_reboot_status);
void ISR_POST_COMPLETE()
{
	isr_dbg_print(FPGA_CPU_BOOT_DONE_L);

	if (get_board_revision() >= SYS_BOARD_PVT) {
		handle_post_status(GPIO_LOW, false);

		if (get_post_status())
			handle_post_action();
		else {
			reset_post_end_work_status();
			sbmr_reset_9byte_postcode_ok();
			reset_ssif_ok();
			set_satmc_status(false);
			retimer_addr_loss();
			k_work_schedule(&check_host_reboot_work, K_SECONDS(POST_DOWN_2_SECOND));
		}
	}
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

void ISR_RTC_CLR()
{
	isr_dbg_print(RTC_CLR_L);

	LOG_INF("RTC clear triggered, record to EEPROM");
	access_rtc_clr_flag(RTC_CLR_ASSERT);
}

void ISR_HSC_OC()
{
	isr_dbg_print(FM_HSC_TIMER);

	struct ipmi_storage_add_sel_req add_sel_msg = { 0 };
	if (gpio_get(FM_HSC_TIMER) == GPIO_HIGH) {
		add_sel_msg.event.event_dir_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
	} else {
		add_sel_msg.event.event_dir_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
	}

	add_sel_msg.event.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
	add_sel_msg.event.sensor_num = SENSOR_NUM_SYSTEM_STATUS;
	add_sel_msg.event.event_data[0] = IPMI_OEM_EVENT_OFFSET_SYS_HSCTIMER;
	add_sel_msg.event.event_data[1] = 0xFF;
	add_sel_msg.event.event_data[2] = 0xFF;
	if (!mctp_add_sel_to_ipmi(&add_sel_msg, ADD_COMMON_SEL)) {
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
	if ((get_ssif_ok() == false) || (sbmr_get_uefi_status() == false)) {
		struct ipmi_storage_add_sel_req add_sel_msg = { 0 };
		add_sel_msg.event.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		add_sel_msg.event.sensor_num = SENSOR_NUM_PROC_FAIL;
		add_sel_msg.event.event_dir_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		add_sel_msg.event.event_data[0] = IPMI_EVENT_OFFSET_PROCESSOR_FRB3;
		add_sel_msg.event.event_data[1] = 0xFF;
		add_sel_msg.event.event_data[2] = 0xFF;
		if (!mctp_add_sel_to_ipmi(&add_sel_msg, ADD_COMMON_SEL)) {
			LOG_ERR("Failed to assert FRE3 event log.");
		}
	}
}

#define CPLD_CPU_POWER_SEQ_STATE 0x11
#define CPU_SHDN_STATE 0x07

#define IPMI_SYS_EVENT_OFFSET_CPU_THERM_TRIP_OR_VR_HOT 0x16

static void read_cpu_power_seq_status(struct k_work *work)
{
	if (CPU_power_good() == true)
		return;

	/* CHECK1 - If cpu abnormal off, add SEL, it might trigger from CPU or VR. */
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 3;
	i2c_msg.bus = I2C_BUS1;
	i2c_msg.target_addr = (CPLD_I2C_ADDR >> 1);
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = CPLD_CPU_POWER_SEQ_STATE;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read CPU power seq state from CPLD.");
		return;
	}

	LOG_INF("CPU power seq state: 0x%x", i2c_msg.data[0]);
	if (i2c_msg.data[0] != CPU_SHDN_STATE)
		return;

	LOG_WRN("CPU abnormal off, add CPU thermal trip or VR hot sel.");
	struct ipmi_storage_add_sel_req add_sel_msg = { 0 };
	add_sel_msg.event.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
	add_sel_msg.event.sensor_num = SENSOR_NUM_SYSTEM_STATUS;
	add_sel_msg.event.event_dir_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
	add_sel_msg.event.event_data[0] = IPMI_SYS_EVENT_OFFSET_CPU_THERM_TRIP_OR_VR_HOT;
	add_sel_msg.event.event_data[1] = 0xFF;
	add_sel_msg.event.event_data[2] = 0xFF;
	if (!mctp_add_sel_to_ipmi(&add_sel_msg, ADD_COMMON_SEL)) {
		LOG_ERR("Failed to add CPU thermal trip or VR hot sel.");
	}

	/* CHECK2 - Try to read primary CPUVDD whether CPU shutdown by vr OCP */
	if (get_board_revision() < SYS_BOARD_PVT) // VR run with standby power only >= PVT
		return;

	if (get_oth_module() != OTH_MODULE_PRIMARY)
		return;

	gpio_set(FM_VR_FW_PROGRAM_L, GPIO_LOW); // In case of CPU suddenly turns on
	gpio_set(BIC_CPLD_VRD_MUX_SEL, GPIO_LOW);

	i2c_msg.bus = I2C_BUS3;
	i2c_msg.target_addr = (CPUVDD_I2C_ADDR >> 1);
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = PMBUS_PAGE;
	i2c_msg.data[1] = PMBUS_PAGE_0;
	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to switch CPUVDD page to 0.");
		goto exit;
	}

	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = 1;
	i2c_msg.data[0] = PMBUS_STATUS_BYTE;
	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read CPUVDD status byte.");
		goto exit;
	}

	LOG_INF("CPUVDD byte status: 0x%x", i2c_msg.data[0]);
	if (!(i2c_msg.data[0] & BIT(4)))
		goto exit;

	LOG_WRN("CPUVDD OCP, add OCP sel.");
	memset(&add_sel_msg, 0, sizeof(add_sel_msg));
	add_sel_msg.event.sensor_type = IPMI_OEM_SENSOR_TYPE_VR;
	add_sel_msg.event.sensor_num = SENSOR_NUM_VR_OCP;
	add_sel_msg.event.event_dir_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
	add_sel_msg.event.event_data[0] = IPMI_EVENT_CPUVDD_OCP_ASSERT;
	add_sel_msg.event.event_data[1] = 0xFF;
	add_sel_msg.event.event_data[2] = 0xFF;
	if (!mctp_add_sel_to_ipmi(&add_sel_msg, ADD_COMMON_SEL)) {
		LOG_ERR("Failed to add OCP sel.");
		goto exit;
	} else {
		i2c_msg.tx_len = 1;
		i2c_msg.data[0] = PMBUS_CLEAR_FAULTS;
		if (i2c_master_write(&i2c_msg, retry)) {
			LOG_ERR("Failed to clear CPUVDD fault.");
			goto exit;
		}
	}

exit:
	gpio_set(BIC_CPLD_VRD_MUX_SEL, GPIO_HIGH);
	gpio_set(FM_VR_FW_PROGRAM_L, GPIO_HIGH);
}

K_WORK_DELAYABLE_DEFINE(read_cpu_power_seq_status_work, read_cpu_power_seq_status);
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

	if (CPU_power_good() == true) {
		reset_sbmr_postcode_buffer();
		k_work_schedule_for_queue(&plat_work_q, &set_DC_on_5s_work,
					  K_SECONDS(DC_ON_5_SECOND));
		k_work_schedule_for_queue(&plat_work_q, &PROC_FAIL_work,
					  K_SECONDS(PROC_FAIL_START_DELAY_SECOND));
		start_satmc_access_poll();
	} else {
		set_satmc_status(false);
		retimer_addr_loss();

		/* Pull high virtual bios complete pin */
		handle_post_status(GPIO_HIGH, true);
		reset_post_end_work_status();
		handle_tda38741_work_around();
		if (k_work_cancel_delayable(&PROC_FAIL_work) != 0) {
			LOG_ERR("Failed to cancel proc_fail delay work.");
		}
		reset_ssif_ok();
		sbmr_reset_9byte_postcode_ok();

		if (k_work_cancel_delayable(&set_DC_on_5s_work) != 0) {
			LOG_ERR("Failed to cancel set dc on delay work.");
		}
		set_DC_on_delayed_status();
		k_work_schedule(&read_cpu_power_seq_status_work, K_MSEC(500));
	}
}

void ISR_GPIOE2()
{
	isr_dbg_print(SPI_BMC_FPGA_INT_L);
}

void ISR_HSC_THROTTLE()
{
	isr_dbg_print(IRQ_HSC_ALERT1_L);

	struct ipmi_storage_add_sel_req add_sel_msg = { 0 };
	static bool is_hsc_throttle_assert = false; // Flag for filt out fake alert
	if (gpio_get(RUN_POWER_PG) == GPIO_LOW) {
		return;
	} else {
		if ((gpio_get(IRQ_HSC_ALERT1_L) == GPIO_HIGH) && (is_hsc_throttle_assert == true)) {
			add_sel_msg.event.event_dir_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
			is_hsc_throttle_assert = false;
		} else if ((gpio_get(IRQ_HSC_ALERT1_L) == GPIO_LOW) &&
			   (is_hsc_throttle_assert == false)) {
			add_sel_msg.event.event_dir_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			is_hsc_throttle_assert = true;
		} else { // Fake alert
			return;
		}

		add_sel_msg.event.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		add_sel_msg.event.sensor_num = SENSOR_NUM_SYSTEM_STATUS;
		add_sel_msg.event.event_data[0] = IPMI_OEM_EVENT_OFFSET_SYS_PMBUSALERT;
		add_sel_msg.event.event_data[1] = 0xFF;
		add_sel_msg.event.event_data[2] = 0xFF;
		if (!mctp_add_sel_to_ipmi(&add_sel_msg, ADD_COMMON_SEL)) {
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

	if (gpio_get(RUN_POWER_PG) == GPIO_HIGH) {
		struct ipmi_storage_add_sel_req add_sel_msg = { 0 };
		if (gpio_get(THERM_WARN_CPU1_L_3V3) == GPIO_HIGH) {
			add_sel_msg.event.event_dir_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
		} else {
			add_sel_msg.event.event_dir_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		}

		add_sel_msg.event.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		add_sel_msg.event.sensor_num = SENSOR_NUM_SYSTEM_STATUS;
		add_sel_msg.event.event_data[0] = IPMI_OEM_EVENT_OFFSET_SYS_FMTHROTTLE;
		add_sel_msg.event.event_data[1] = 0xFF;
		add_sel_msg.event.event_data[2] = 0xFF;
		if (!mctp_add_sel_to_ipmi(&add_sel_msg, ADD_COMMON_SEL)) {
			LOG_ERR("Failed to add FM throttle sel.");
		}
	}
}

void ISR_CPU_OVERTEMP()
{
	isr_dbg_print(THERM_OVERT_CPU1_L_3V3);

	if (gpio_get(RUN_POWER_PG) == GPIO_HIGH) {
		struct ipmi_storage_add_sel_req add_sel_msg = { 0 };
		static bool is_thermal_trip_assert = 0;
		if (gpio_get(THERM_OVERT_CPU1_L_3V3) == GPIO_LOW) {
			if ((get_post_status() == true) && (is_thermal_trip_assert == false)) {
				add_sel_msg.event.event_dir_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
				is_thermal_trip_assert = true;
			}
		} else if (gpio_get(THERM_OVERT_CPU1_L_3V3) && (is_thermal_trip_assert == true)) {
			add_sel_msg.event.event_dir_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
			is_thermal_trip_assert = false;
		} else {
			return;
		}

		add_sel_msg.event.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		add_sel_msg.event.sensor_num = SENSOR_NUM_SYSTEM_STATUS;
		add_sel_msg.event.event_data[0] = IPMI_OEM_EVENT_OFFSET_SYS_THERMAL_TRIP;
		add_sel_msg.event.event_data[1] = 0xFF;
		add_sel_msg.event.event_data[2] = 0xFF;
		if (!mctp_add_sel_to_ipmi(&add_sel_msg, ADD_COMMON_SEL)) {
			LOG_ERR("Thermal trip addsel fail");
		}
	}
}

void ISR_CPU_FAULT_ALERT()
{
	isr_dbg_print(RUN_POWER_FAULT_L);
	send_gpio_interrupt(RUN_POWER_FAULT_L);

	struct ipmi_storage_add_sel_req add_sel_msg = { 0 };
	if (gpio_get(RUN_POWER_PG) == GPIO_HIGH) {
		add_sel_msg.event.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		add_sel_msg.event.sensor_num = SENSOR_NUM_CPU_FAULT;
		add_sel_msg.event.event_dir_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
		add_sel_msg.event.event_data[0] = 0xFF;
		add_sel_msg.event.event_data[1] = 0xFF;
		add_sel_msg.event.event_data[2] = 0xFF;
		if (!mctp_add_sel_to_ipmi(&add_sel_msg, ADD_COMMON_SEL)) {
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
	struct ipmi_storage_add_sel_req add_sel_msg = { 0 };
	if (get_DC_status()) {
		if ((gpio_get(FAST_PROCHOT_L) == GPIO_HIGH) && (is_mb_throttle_assert == true)) {
			add_sel_msg.event.event_dir_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
			is_mb_throttle_assert = false;
		} else if ((gpio_get(FAST_PROCHOT_L) == GPIO_LOW) &&
			   (is_mb_throttle_assert == false)) {
			add_sel_msg.event.event_dir_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			is_mb_throttle_assert = true;
		} else {
			return;
		}
		add_sel_msg.event.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		add_sel_msg.event.sensor_num = SENSOR_NUM_SYSTEM_STATUS;
		add_sel_msg.event.event_data[0] = IPMI_OEM_EVENT_OFFSET_SYS_FIRMWAREASSERT;
		add_sel_msg.event.event_data[1] = 0xFF;
		add_sel_msg.event.event_data[2] = 0xFF;
		if (!mctp_add_sel_to_ipmi(&add_sel_msg, ADD_COMMON_SEL)) {
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
	struct ipmi_storage_add_sel_req add_sel_msg = { 0 };
	if (gpio_get(RUN_POWER_PG) == GPIO_HIGH) {
		if ((gpio_get(CPU_BIC_PROCHOT_L) == GPIO_HIGH) &&
		    (is_sys_throttle_assert == true)) {
			add_sel_msg.event.event_dir_type = IPMI_OEM_EVENT_TYPE_DEASSERT;
			is_sys_throttle_assert = false;
		} else if ((gpio_get(CPU_BIC_PROCHOT_L) == GPIO_LOW) &&
			   (is_sys_throttle_assert == false)) {
			add_sel_msg.event.event_dir_type = IPMI_EVENT_TYPE_SENSOR_SPECIFIC;
			is_sys_throttle_assert = true;
		} else {
			return;
		}
		add_sel_msg.event.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		add_sel_msg.event.sensor_num = SENSOR_NUM_SYSTEM_STATUS;
		add_sel_msg.event.event_data[0] = IPMI_OEM_EVENT_OFFSET_SYS_THROTTLE;
		add_sel_msg.event.event_data[1] = 0xFF;
		add_sel_msg.event.event_data[2] = 0xFF;
		if (!mctp_add_sel_to_ipmi(&add_sel_msg, ADD_COMMON_SEL)) {
			LOG_ERR("Failed to add System Throttle sel.");
		}
	}
}
