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

static void SLP3_handler()
{
	addsel_msg_t sel_msg;
	if (gpio_get(FM_SLPS3_PLD_N) && (gpio_get(PWRGD_SYS_PWROK) == 0)) {
		sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		sel_msg.snr_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_SYS_VRWATCHDOG;
		sel_msg.evt_data2 = 0xFF;
		sel_msg.evt_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printk("VR watchdog timeout addsel fail\n");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(SLP3_work, SLP3_handler);
void ISR_SLP3()
{
	if (gpio_get(FM_SLPS3_PLD_N)) {
		printk("slp3\n");
		k_work_schedule(&SLP3_work, K_MSEC(10000));
	} else {
		if (k_work_cancel_delayable(&SLP3_work) != 0) {
			printk("Cancel SLP3 delay work fail\n");
		}
	}
}

void ISR_POST_COMPLETE()
{
	set_post_status(FM_BIOS_POST_CMPLT_BMC_N);
}

K_WORK_DELAYABLE_DEFINE(set_DC_on_5s_work, set_DC_on_delayed_status);
K_WORK_DELAYABLE_DEFINE(set_DC_off_10s_work, set_DC_off_delayed_status);
#define DC_ON_5_SECOND 5
#define DC_OFF_10_SECOND 10
void ISR_DC_ON()
{
	set_DC_status(PWRGD_SYS_PWROK);

	if (get_DC_status() == 1) {
		k_work_schedule(&set_DC_on_5s_work, K_SECONDS(DC_ON_5_SECOND));

		if (k_work_cancel_delayable(&set_DC_off_10s_work) != 0) {
			printk("Cancel set dc off delay work fail\n");
		}
		set_DC_off_delayed_status();
	} else {
		set_DC_on_delayed_status();
		clear_unaccessible_sensor_cache();
		k_work_schedule(&set_DC_off_10s_work, K_SECONDS(DC_OFF_10_SECOND));

		snoop_abort_thread();

		if (gpio_get(FM_SLPS3_PLD_N) && gpio_get(RST_RSMRST_BMC_N)) {
			addsel_msg_t sel_msg;
			sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_OEM_C3;
			sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
			sel_msg.snr_number = SENSOR_NUM_POWER_ERROR;
			sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PWROK_FAIL;
			sel_msg.evt_data2 = 0xFF;
			sel_msg.evt_data3 = 0xFF;
			if (!add_sel_evt_record(&sel_msg)) {
				printk("System PWROK failure addsel fail\n");
			}
		}
	}
}

void ISR_BMC_PRDY()
{
	send_gpio_interrupt(H_BMC_PRDY_BUF_N);
}

static void PROC_FAIL_handler()
{
	/* if have not received kcs and post code, add FRB3 event log. */
	if ((get_kcs_ok() == 0) && (get_postcode_ok() == 0)) {
		addsel_msg_t sel_msg;
		bool ret = 0;

		memset(&sel_msg, 0, sizeof(addsel_msg_t));

		sel_msg.snr_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.snr_number = SENSOR_NUM_PROC_FAIL;
		sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		sel_msg.evt_data1 = IPMI_EVENT_OFFSET_PROCESSOR_FRB3;
		sel_msg.evt_data2 = 0xFF;
		sel_msg.evt_data3 = 0xFF;
		ret = add_sel_evt_record(&sel_msg);
		if (!ret) {
			printk("Fail to assert FRE3 event log.\n");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(PROC_FAIL_work, PROC_FAIL_handler);
#define PROC_FAIL_START_DELAY_SECOND 10
void ISR_PWRGD_CPU()
{
	if (gpio_get(PWRGD_CPU_LVC3) == 1) {
		/* start thread proc_fail_handler after 10 seconds */
		k_work_schedule(&PROC_FAIL_work, K_SECONDS(PROC_FAIL_START_DELAY_SECOND));
	} else {
		if (k_work_cancel_delayable(&PROC_FAIL_work) != 0) {
			printk("Cancel proc_fail delay work fail\n");
		}
		reset_kcs_ok();
		reset_postcode_ok();
	}
	send_gpio_interrupt(PWRGD_CPU_LVC3);
}

static void CAT_ERR_handler()
{
	if ((gpio_get(RST_PLTRST_BUF_N) == 1) || (gpio_get(PWRGD_SYS_PWROK) == 1)) {
		addsel_msg_t sel_msg;
		bool ret = 0;

		memset(&sel_msg, 0, sizeof(addsel_msg_t));

		sel_msg.snr_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.snr_number = SENSOR_NUM_CATERR;
		sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		/* MCERR: one pulse, IERR: keep low */
		if (gpio_get(FM_CPU_RMCA_CATERR_LVT3_N) == 1) {
			sel_msg.evt_data1 = IPMI_EVENT_OFFSET_PROCESSOR_MCERR;
		} else {
			sel_msg.evt_data1 = IPMI_EVENT_OFFSET_PROCESSOR_IERR;
		}
		sel_msg.evt_data2 = 0xFF;
		sel_msg.evt_data3 = 0xFF;
		ret = add_sel_evt_record(&sel_msg);
		if (!ret) {
			printk("Fail to assert CatErr event log.\n");
		}
	}
}

K_WORK_DELAYABLE_DEFINE(CAT_ERR_work, CAT_ERR_handler);
#define CATERR_START_DELAY_SECOND 2
void ISR_CATERR()
{
	if ((gpio_get(RST_PLTRST_BUF_N) == 1)) {
		if (k_work_cancel_delayable(&CAT_ERR_work) != 0) {
			printk("Cancel caterr delay work fail\n");
		}
		/* start thread CatErr_handler after 2 seconds */
		k_work_schedule(&CAT_ERR_work, K_SECONDS(CATERR_START_DELAY_SECOND));
	}
}

void ISR_PLTRST()
{
	if (gpio_get(RST_PLTRST_BUF_N) == 1) {
		snoop_start_thread();
		init_send_postcode_thread();
	}

	send_gpio_interrupt(RST_PLTRST_BUF_N);
}

void ISR_DBP_PRSNT()
{
	send_gpio_interrupt(FM_DBP_PRESENT_N);
}

void ISR_FM_THROTTLE()
{
	addsel_msg_t sel_msg;
	if (gpio_get(PWRGD_CPU_LVC3)) {
		if (gpio_get(FM_THROTTLE_R_N)) {
			sel_msg.evt_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		}
		sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.snr_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_SYS_FMTHROTTLE;
		sel_msg.evt_data2 = 0xFF;
		sel_msg.evt_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printk("FM Throttle addsel fail\n");
		}
	}
}

void ISR_HSC_THROTTLE()
{
	addsel_msg_t sel_msg;
	if (gpio_get(RST_RSMRST_BMC_N)) {
		if ((gpio_get(PWRGD_SYS_PWROK) == 0 && get_DC_off_delayed_status() == 0)) {
			return;
		} else {
			if (gpio_get(IRQ_SML1_PMBUS_ALERT_N)) {
				sel_msg.evt_type = IPMI_OEM_EVENT_TYPE_DEASSART;
			} else {
				sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
			}
			sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
			sel_msg.snr_number = SENSOR_NUM_SYSTEM_STATUS;
			sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PMBUSALERT;
			sel_msg.evt_data2 = 0xFF;
			sel_msg.evt_data3 = 0xFF;
			if (!add_sel_evt_record(&sel_msg)) {
				printk("HSC Throttle addsel fail\n");
			}
		}
	}
}

void ISR_MB_THROTTLE()
{
	addsel_msg_t sel_msg;
	if (gpio_get(RST_RSMRST_BMC_N)) {
		if (gpio_get(FAST_PROCHOT_N)) {
			sel_msg.evt_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		}
		sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.snr_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_SYS_FIRMWAREASSERT;
		sel_msg.evt_data2 = 0xFF;
		sel_msg.evt_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printk("MB Throttle addsel fail\n");
		}
	}
}

void ISR_SOC_THMALTRIP()
{
	addsel_msg_t sel_msg;
	if (gpio_get(RST_PLTRST_PLD_N)) {
		sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.snr_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_SYS_THERMAL_TRIP;
		sel_msg.evt_data2 = 0xFF;
		sel_msg.evt_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printk("SOC Thermal trip addsel fail\n");
		}
	}
}

void ISR_SYS_THROTTLE()
{
	addsel_msg_t sel_msg;
	if (gpio_get(RST_PLTRST_PLD_N) && gpio_get(PWRGD_SYS_PWROK)) {
		if (gpio_get(FM_CPU_BIC_PROCHOT_LVT3_N)) {
			sel_msg.evt_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		}
		sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.snr_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_SYS_THROTTLE;
		sel_msg.evt_data2 = 0xFF;
		sel_msg.evt_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printk("System Throttle addsel fail\n");
		}
	}
}

void ISR_PCH_THMALTRIP()
{
	addsel_msg_t sel_msg;
	static bool is_PCH_assert = 0;
	if (gpio_get(FM_PCHHOT_N) == 0) {
		if (gpio_get(RST_PLTRST_PLD_N) && get_post_status() && (is_PCH_assert == 0)) {
			sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
			is_PCH_assert = 1;
		}
	} else if (gpio_get(FM_PCHHOT_N) && (is_PCH_assert == 1)) {
		sel_msg.evt_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		is_PCH_assert = 0;
	} else {
		return;
	}
	sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
	sel_msg.snr_number = SENSOR_NUM_SYSTEM_STATUS;
	sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_SYS_PCHHOT;
	sel_msg.evt_data2 = 0xFF;
	sel_msg.evt_data3 = 0xFF;
	if (!add_sel_evt_record(&sel_msg)) {
		printk("PCH Thermal trip addsel fail\n");
	}
}

void ISR_HSC_OC()
{
	addsel_msg_t sel_msg;
	if (gpio_get(RST_RSMRST_BMC_N)) {
		if (gpio_get(FM_HSC_TIMER)) {
			sel_msg.evt_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		}
		sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_SYS_STA;
		sel_msg.snr_number = SENSOR_NUM_SYSTEM_STATUS;
		sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_SYS_HSCTIMER;
		sel_msg.evt_data2 = 0xFF;
		sel_msg.evt_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printk("HSC OC addsel fail\n");
		}
	}
}

void ISR_CPU_MEMHOT()
{
	addsel_msg_t sel_msg;
	if (gpio_get(RST_PLTRST_PLD_N) && gpio_get(PWRGD_SYS_PWROK)) {
		if (gpio_get(H_CPU_MEMHOT_OUT_LVC3_N)) {
			sel_msg.evt_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		}
		sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_CPU_DIMM_HOT;
		sel_msg.snr_number = SENSOR_NUM_CPUDIMM_HOT;
		sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_DIMM_HOT;
		sel_msg.evt_data2 = 0xFF;
		sel_msg.evt_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printk("CPU MEM HOT addsel fail\n");
		}
	}
}

void ISR_CPUVR_HOT()
{
	addsel_msg_t sel_msg;
	if (gpio_get(RST_PLTRST_PLD_N) && gpio_get(PWRGD_SYS_PWROK)) {
		if (gpio_get(IRQ_CPU0_VRHOT_N)) {
			sel_msg.evt_type = IPMI_OEM_EVENT_TYPE_DEASSART;
		} else {
			sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		}
		sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_CPU_DIMM_VR_HOT;
		sel_msg.snr_number = SENSOR_NUM_VR_HOT;
		sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_CPU_VR_HOT;
		sel_msg.evt_data2 = 0xFF;
		sel_msg.evt_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printk("CPU VR HOT addsel fail\n");
		}
	}
}

void ISR_PCH_PWRGD()
{
	addsel_msg_t sel_msg;
	if (gpio_get(FM_SLPS3_PLD_N)) {
		sel_msg.snr_type = IPMI_OEM_SENSOR_TYPE_OEM_C3;
		sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		sel_msg.snr_number = SENSOR_NUM_POWER_ERROR;
		sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_PCH_PWROK_FAIL;
		sel_msg.evt_data2 = 0xFF;
		sel_msg.evt_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printk("PCH PWROK failure addsel fail\n");
		}
	}
}

void ISR_RMCA()
{
	if ((gpio_get(RST_PLTRST_BUF_N) == GPIO_HIGH) || (gpio_get(PWRGD_CPU_LVC3) == GPIO_HIGH)) {
		addsel_msg_t sel_msg;
		sel_msg.snr_type = IPMI_SENSOR_TYPE_PROCESSOR;
		sel_msg.evt_type = IPMI_EVENT_TYPE_SENSOR_SPEC;
		sel_msg.snr_number = SENSOR_NUM_CATERR;
		sel_msg.evt_data1 = IPMI_OEM_EVENT_OFFSET_MEM_RMCA;
		sel_msg.evt_data2 = 0xFF;
		sel_msg.evt_data3 = 0xFF;
		if (!add_sel_evt_record(&sel_msg)) {
			printk("RMCA addsel fail\n");
		}
	}
}
