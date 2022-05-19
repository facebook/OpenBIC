#include "plat_isr.h"

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
	addsel_msg_t sel_msg;
	if ((gpio_get(FM_SLPS3_PLD_N) == GPIO_HIGH) && (gpio_get(PWRGD_SYS_PWROK) == GPIO_LOW)) {
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_VRWATCHDOG;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printf("VR watchdog timeout addsel fail\n");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(SLP3_work, SLP3_handler);
void ISR_SLP3()
{
	if (gpio_get(FM_SLPS3_PLD_N) == GPIO_HIGH) {
		printf("slp3\n");
		k_work_schedule(&SLP3_work, K_MSEC(10000));
		return;
	}
	if (k_work_cancel_delayable(&SLP3_work) != 0) {
		printf("[%s] Failed to cancel delayable work\n", __func__);
	}
}

void ISR_POST_COMPLETE()
{
	set_post_status(FM_BIOS_POST_CMPLT_BMC_N);

	if (gpio_get(FM_BIOS_POST_CMPLT_BMC_N) == GPIO_LOW) { // Post complete
		if (get_me_mode() == ME_INIT_MODE) {
			init_me_firmware();
		}
	}
}

K_WORK_DELAYABLE_DEFINE(set_DC_on_5s_work, set_DC_on_delayed_status);
K_WORK_DELAYABLE_DEFINE(set_DC_off_10s_work, set_DC_off_delayed_status);
#define DC_ON_5_SECOND 5
#define DC_OFF_10_SECOND 10
void ISR_DC_ON()
{
	set_DC_status(PWRGD_SYS_PWROK);

	if (get_DC_status() == true) {
		k_work_schedule(&set_DC_on_5s_work, K_SECONDS(DC_ON_5_SECOND));

		if (k_work_cancel_delayable(&set_DC_off_10s_work) != 0) {
			printf("Cancel set dc off delay work fail\n");
		}
		set_DC_off_delayed_status();
	} else {
		set_DC_on_delayed_status();
		k_work_schedule(&set_DC_off_10s_work, K_SECONDS(DC_OFF_10_SECOND));

		if ((gpio_get(FM_SLPS3_PLD_N) == GPIO_HIGH) &&
		    (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH)) {
			addsel_msg_t sel_msg;
			sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_OEM_C3;
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
			sel_msg.sensor_number = SENSOR_NUM_POWER_ERROR;
			sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PWROK_FAIL;
			sel_msg.event_data2 = 0xFF;
			sel_msg.event_data3 = 0xFF;
			if (!add_sel_evt_record(&sel_msg)) {
				printf("System PWROK failure addsel fail\n");
			}
		}
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
		addsel_msg_t sel_msg;
		bool ret = false;

		memset(&sel_msg, 0, sizeof(addsel_msg_t));

		sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.sensor_number = SENSOR_NUM_PROC_FAIL;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		sel_msg.event_data1 = IPMI_EVENT_OFFSET_PROCESSOR_FRB3;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		ret = add_sel_evt_record(&sel_msg);
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
		k_work_schedule(&PROC_FAIL_work, K_SECONDS(PROC_FAIL_START_DELAY_SECOND));
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
		addsel_msg_t sel_msg;
		bool ret = false;

		memset(&sel_msg, 0, sizeof(addsel_msg_t));

		sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.sensor_number = SENSOR_NUM_CATERR;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		/* MCERR: one pulse, IERR: keep low */
		if (gpio_get(FM_CATERR_LVT3_N) == GPIO_HIGH) {
			sel_msg.event_data1 = IPMI_EVENT_OFFSET_PROCESSOR_MCERR;
		} else {
			sel_msg.event_data1 = IPMI_EVENT_OFFSET_PROCESSOR_IERR;
		}
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		ret = add_sel_evt_record(&sel_msg);
		if (!ret) {
			printf("Fail to assert CatErr event log.\n");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(CAT_ERR_work, CAT_ERR_handler);
#define CATERR_START_DELAY_SECOND 2
void ISR_CATERR()
{
	if ((gpio_get(RST_PLTRST_BUF_N) == GPIO_HIGH)) {
		if (k_work_cancel_delayable(&CAT_ERR_work) != 0) {
			printf("Cancel caterr delay work fail\n");
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

void ISR_FM_THROTTLE()
{
	addsel_msg_t sel_msg;
	if (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH) {
		if (gpio_get(FM_THROTTLE_R_N) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		}
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_FMTHROTTLE;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printf("FM Throttle addsel fail\n");
		}
	}
}

void ISR_HSC_THROTTLE()
{
	addsel_msg_t sel_msg;
	static bool is_hsc_throttle_assert = false; // Flag for filt out fake alert
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		if ((gpio_get(PWRGD_SYS_PWROK) == GPIO_LOW) &&
		    (get_DC_off_delayed_status() == false)) {
			return;
		} else {
			if ((gpio_get(IRQ_SML1_PMBUS_ALERT_N) == GPIO_HIGH) &&
			    (is_hsc_throttle_assert == true)) {
				sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
				is_hsc_throttle_assert = false;
			} else if ((gpio_get(IRQ_SML1_PMBUS_ALERT_N) == GPIO_LOW) &&
				   (is_hsc_throttle_assert == false)) {
				sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
				is_hsc_throttle_assert = true;
			} else { // Fake alert
				return;
			}

			sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
			sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
			sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PMBUSALERT;
			sel_msg.event_data2 = 0xFF;
			sel_msg.event_data3 = 0xFF;
			if (!add_sel_evt_record(&sel_msg)) {
				printf("HSC Throttle addsel fail\n");
			}
		}
	}
}

void ISR_MB_THROTTLE()
{
	addsel_msg_t sel_msg;
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		if (gpio_get(FAST_PROCHOT_N) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		}
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_FIRMWAREASSERT;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printf("MB Throttle addsel fail\n");
		}
	}
}

void ISR_SOC_THMALTRIP()
{
	addsel_msg_t sel_msg;
	if (gpio_get(RST_PLTRST_PLD_N) == GPIO_HIGH) {
		if (gpio_get(H_CPU_MEMTRIP_LVC3_N) ==
		    GPIO_HIGH) { // Reference pin for memory thermal trip event
			sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_THERMAL_TRIP;
		} else {
			sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_MEMORY_THERMALTRIP;
		}
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			if (sel_msg.event_data1 == IPMI_OEM_EVENT_OFFSET_SYS_THERMAL_TRIP) {
				printf("SOC Thermal trip addsel fail\n");
			} else {
				printf("Memory Thermal trip addsel fail\n");
			}
		}
	}
}

void ISR_SYS_THROTTLE()
{
	addsel_msg_t sel_msg;
	if ((gpio_get(RST_PLTRST_PLD_N) == GPIO_HIGH) && (gpio_get(PWRGD_SYS_PWROK) == GPIO_HIGH)) {
		if (gpio_get(FM_CPU_BIC_PROCHOT_LVT3_N) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		}
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_THROTTLE;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printf("System Throttle addsel fail\n");
		}
	}
}

void ISR_PCH_THMALTRIP()
{
	addsel_msg_t sel_msg;
	static bool is_pch_assert = 0;
	if (gpio_get(FM_PCHHOT_N) == GPIO_LOW) {
		if ((gpio_get(RST_PLTRST_PLD_N) == GPIO_HIGH) && (get_post_status() == true) &&
		    (is_pch_assert == false)) {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
			is_pch_assert = true;
		}
	} else if (gpio_get(FM_PCHHOT_N) && (is_pch_assert == true)) {
		sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		is_pch_assert = false;
	} else {
		return;
	}
	sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
	sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
	sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PCHHOT;
	sel_msg.event_data2 = 0xFF;
	sel_msg.event_data3 = 0xFF;
	if (!add_sel_evt_record(&sel_msg)) {
		printf("PCH Thermal trip addsel fail\n");
	}
}

void ISR_HSC_OC()
{
	addsel_msg_t sel_msg;
	if (gpio_get(RST_RSMRST_BMC_N) == GPIO_HIGH) {
		if (gpio_get(FM_HSC_TIMER) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		}
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.sensor_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_SYS_HSCTIMER;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printf("HSC OC addsel fail\n");
		}
	}
}

void ISR_CPU_MEMHOT()
{
	addsel_msg_t sel_msg;
	if ((gpio_get(RST_PLTRST_PLD_N) == GPIO_HIGH) && (gpio_get(PWRGD_SYS_PWROK) == GPIO_HIGH)) {
		if (gpio_get(H_CPU_MEMHOT_OUT_LVC3_N) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		}
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_CPU_DIMM_HOT;
		sel_msg.sensor_number = SENSOR_NUM_CPUDIMM_HOT;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_DIMM_HOT;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printf("CPU MEM HOT addsel fail\n");
		}
	}
}

void ISR_CPUVR_HOT()
{
	addsel_msg_t sel_msg;
	if ((gpio_get(RST_PLTRST_PLD_N) == GPIO_HIGH) && (gpio_get(PWRGD_SYS_PWROK) == GPIO_HIGH)) {
		if (gpio_get(IRQ_CPU0_VRHOT_N) == GPIO_HIGH) {
			sel_msg.event_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		}
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_CPU_DIMM_VR_HOT;
		sel_msg.sensor_number = SENSOR_NUM_VR_HOT;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_CPU_VR_HOT;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printf("CPU VR HOT addsel fail\n");
		}
	}
}

void ISR_PCH_PWRGD()
{
	addsel_msg_t sel_msg;
	if (gpio_get(FM_SLPS3_PLD_N) == GPIO_HIGH) {
		sel_msg.sensor_type = IPMI_OEM_SENSOR_TYPE_OEM_C3;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		sel_msg.sensor_number = SENSOR_NUM_POWER_ERROR;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_PCH_PWROK_FAIL;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printf("PCH PWROK failure addsel fail\n");
		}
	}
}

void ISR_RMCA()
{
	if ((gpio_get(RST_PLTRST_BUF_N) == GPIO_HIGH) || (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH)) {
		addsel_msg_t sel_msg;
		sel_msg.sensor_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.event_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		sel_msg.sensor_number = SENSOR_NUM_CATERR;
		sel_msg.event_data1 = IPMI_OEM_EVENT_OFFSET_MEM_RMCA;
		sel_msg.event_data2 = 0xFF;
		sel_msg.event_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printf("RMCA addsel fail\n");
		}
	}
}
