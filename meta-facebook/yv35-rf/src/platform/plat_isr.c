#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include "plat_power_seq.h"
#include "power_status.h"
#include "plat_gpio.h"
#include "plat_isr.h"

#define POWER_SEQ_CTRL_STACK_SIZE 1000
#define DC_ON_5_SECOND 5

K_WORK_DELAYABLE_DEFINE(set_DC_on_5s_work, set_DC_on_delayed_status);

K_THREAD_STACK_DEFINE(power_thread, POWER_SEQ_CTRL_STACK_SIZE);
struct k_thread power_thread_handler;
k_tid_t power_tid;

void control_power_sequence()
{
	if (gpio_get(FM_POWER_EN) == POWER_ON) { // CraterLake DC on
		if (gpio_get(PWRGD_CARD_PWROK) == POWER_OFF) {
			// If power on sequence not finished or not started , abort power off thread before creating power on thread
			abort_power_thread();
			init_power_on_thread();
		} else {
			// If the last stage of power on sequence already power on , no need to recheck power on sequence
			// Update the flag of power on sequence number
			set_power_on_seq(NUMBER_OF_POWER_ON_SEQ);
		}
	} else { // CraterLake DC off
		if (gpio_get(CLK_100M_OSC_EN) == HIGH_ACTIVE) {
			// If power off sequence not finished or not started , abort power on thread before creating power off thread
			abort_power_thread();
			init_power_off_thread();
		} else {
			// If the last stage of power off sequence already power off , no need to recheck power off sequence
			// Update the flag of power off sequence number
			set_power_off_seq(NUMBER_OF_POWER_OFF_SEQ);
		}
	}
}

void init_power_on_thread()
{
	// Avoid re-create thread by checking thread status and thread id
	if (power_tid != NULL && strcmp(k_thread_state_str(power_tid), "dead") != 0) {
		return;
	}
	power_tid = k_thread_create(&power_thread_handler, power_thread,
				    K_THREAD_STACK_SIZEOF(power_thread), control_power_on_sequence,
				    NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&power_thread_handler, "power_on_sequence_thread");
}

void init_power_off_thread()
{
	// Avoid re-create thread by checking thread status and thread id
	if (power_tid != NULL && strcmp(k_thread_state_str(power_tid), "dead") != 0) {
		return;
	}
	power_tid = k_thread_create(&power_thread_handler, power_thread,
				    K_THREAD_STACK_SIZEOF(power_thread), control_power_off_sequence,
				    NULL, NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&power_thread_handler, "power_off_sequence_thread");
}

void abort_power_thread()
{
	if (power_tid != NULL && strcmp(k_thread_state_str(power_tid), "dead") != 0) {
		k_thread_abort(power_tid);
	}
}

void check_power_abnormal(uint8_t power_good_gpio_num, uint8_t control_power_gpio_num)
{
	if (gpio_get(power_good_gpio_num) == POWER_OFF) {
		// If the control power pin is high when the power good pin is falling
		// This means that the behavior of turning off is not expected
		if (gpio_get(control_power_gpio_num) == CONTROL_ON) {
			// Sequentially turn off the power
			abort_power_thread();
			init_power_off_thread();
		}
	}
}

void ISR_MB_DC_STATE()
{
	set_MB_DC_status(FM_POWER_EN);
	control_power_sequence();
}

void ISR_DC_STATE()
{
	set_DC_status(PWRGD_CARD_PWROK);

	// Set a access flag after DC on 5 secs
	if (get_DC_status() == true) {
		k_work_schedule(&set_DC_on_5s_work, K_SECONDS(DC_ON_5_SECOND));

	} else {
		if (k_work_cancel_delayable(&set_DC_on_5s_work) != 0) {
			printf("Cancel set dc off delay work fail\n");
		}
		set_DC_on_delayed_status();
	}
}

void ISR_MB_RST()
{
	if (gpio_get(RST_MB_N) == HIGH_ACTIVE) {
		// Enable ASIC reset pin
		gpio_set(ASIC_PERST0_N, HIGH_ACTIVE);
	} else {
		// Disable ASIC reset pin
		gpio_set(ASIC_PERST0_N, HIGH_INACTIVE);
	}
}

void ISR_P0V8_ASICA_POWER_GOOD_LOST()
{
	check_power_abnormal(P0V8_ASICA_PWRGD, FM_P0V8_ASICA_EN);
}

void ISR_P0V8_ASICD_POWER_GOOD_LOST()
{
	check_power_abnormal(P0V8_ASICD_PWRGD, FM_P0V8_ASICD_EN);
}

void ISR_P0V9_ASICA_POWER_GOOD_LOST()
{
	check_power_abnormal(P0V9_ASICA_PWRGD, FM_P0V9_ASICA_EN);
}

void ISR_P1V8_ASIC_POWER_GOOD_LOST()
{
	check_power_abnormal(P1V8_ASIC_PG_R, P1V8_ASIC_EN_R);
}

void ISR_PVPP_AB_POWER_GOOD_LOST()
{
	check_power_abnormal(PVPP_AB_PG_R, PVPP_AB_EN_R);
}

void ISR_PVPP_CD_POWER_GOOD_LOST()
{
	check_power_abnormal(PVPP_CD_PG_R, PVPP_CD_EN_R);
}

void ISR_PVDDQ_AB_POWER_GOOD_LOST()
{
	check_power_abnormal(PWRGD_PVDDQ_AB, FM_PVDDQ_AB_EN);
}

void ISR_PVDDQ_CD_POWER_GOOD_LOST()
{
	check_power_abnormal(PWRGD_PVDDQ_CD, FM_PVDDQ_CD_EN);
}

void ISR_PVTT_AB_POWER_GOOD_LOST()
{
	check_power_abnormal(PVTT_AB_PG_R, PVTT_AB_EN_R);
}

void ISR_PVTT_CD_POWER_GOOD_LOST()
{
	check_power_abnormal(PVTT_CD_PG_R, PVTT_CD_EN_R);
}
