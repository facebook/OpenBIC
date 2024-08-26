#include "adm1272.h"
#include "hal_gpio.h"
#include "sensor.h"
#include "plat_isr.h"
#include "plat_gpio.h"
#include "plat_pwm.h"
#include "plat_hwmon.h"
#include "plat_sensor_table.h"
#include "plat_led.h"
#include "plat_log.h"
#include "plat_threshold.h"
#include "plat_status.h"

void deassert_all_rpu_ready_pin(void)
{
	gpio_set(BIC_RPU_READY0, 0);
	gpio_set(BIC_RPU_READY1, 0);
	gpio_set(BIC_RPU_READY2, 0);
	gpio_set(BIC_RPU_READY3, 0);
}

void set_all_rpu_ready_pin_normal(void)
{
	gpio_set(BIC_RPU_READY0, 1);
	gpio_set(BIC_RPU_READY1, 1);
	gpio_set(BIC_RPU_READY2, 1);
	gpio_set(BIC_RPU_READY3, 1);
}

/* TO DO: 
	1. turn off pump
	2. change hx fan to min speed
	3. store to non volatile memory
*/
void emergency_button_action()
{
	if (gpio_get(CDU_PWR_BTN)) {
		// pump recovery
		if (pump_status_recovery())
			set_pwm_group(PWM_GROUP_E_PUMP, 60);
	} else {
		ctl_all_pwm_dev(0);
	}

	if (rpu_ready_recovery())
		set_all_rpu_ready_pin_normal();
	else
		deassert_all_rpu_ready_pin();
}

void fault_leak_action()
{
	ctl_all_pwm_dev(0);
	deassert_all_rpu_ready_pin();
	gpio_set(RPU_LEAK_ALERT_N, 0);
}

void it_leak_handler(uint8_t idx)
{
	uint8_t sen_num = (idx == IT_LEAK_E_0) ? SENSOR_NUM_IT_LEAK_0_GPIO :
			  (idx == IT_LEAK_E_1) ? SENSOR_NUM_IT_LEAK_1_GPIO :
			  (idx == IT_LEAK_E_2) ? SENSOR_NUM_IT_LEAK_2_GPIO :
			  (idx == IT_LEAK_E_3) ? SENSOR_NUM_IT_LEAK_3_GPIO :
						 0xFF;

	fault_leak_action();
	error_log_event(sen_num, IS_ABNORMAL_VAL);

	switch (idx) {
	case IT_LEAK_E_0:
		set_sticky_sensor_status(STICKY_ITRACK_CHASSIS0_LEAKAGE, 1);
		set_leak_status(AALC_STATUS_IT_LEAK_0, 1);
		break;
	case IT_LEAK_E_1:
		set_sticky_sensor_status(STICKY_ITRACK_CHASSIS1_LEAKAGE, 1);
		set_leak_status(AALC_STATUS_IT_LEAK_1, 1);
		break;
	case IT_LEAK_E_2:
		set_sticky_sensor_status(STICKY_ITRACK_CHASSIS2_LEAKAGE, 1);
		set_leak_status(AALC_STATUS_IT_LEAK_2, 1);
		break;
	case IT_LEAK_E_3:
		set_sticky_sensor_status(STICKY_ITRACK_CHASSIS3_LEAKAGE, 1);
		set_leak_status(AALC_STATUS_IT_LEAK_3, 1);
		break;
	}

	fault_led_control();
}

#define IT_LEAK_ALERT_HANDLER(idx)                                                                 \
	void it_leak_handle_##idx(struct k_work *work)                                             \
	{                                                                                          \
		it_leak_handler(IT_LEAK_E_##idx);                                                  \
	}                                                                                          \
	K_WORK_DEFINE(it_leak_work_##idx, it_leak_handle_##idx);                                   \
	void it_leak_action_##idx(void)                                                            \
	{                                                                                          \
		k_work_submit(&it_leak_work_##idx);                                                \
	}

IT_LEAK_ALERT_HANDLER(0);
IT_LEAK_ALERT_HANDLER(1);
IT_LEAK_ALERT_HANDLER(2);
IT_LEAK_ALERT_HANDLER(3);

void aalc_leak_behavior(uint8_t sensor_num)
{
	fault_leak_action();
	error_log_event(sensor_num, IS_ABNORMAL_VAL);

	uint8_t led_leak = (sensor_num == SENSOR_NUM_BPB_CDU_COOLANT_LEAKAGE_VOLT_V) ?
				   AALC_STATUS_CDU_LEAKAGE :
			   (sensor_num == SENSOR_NUM_BPB_RACK_COOLANT_LEAKAGE_VOLT_V) ?
				   AALC_STATUS_RACK_LEAKAGE :
				   AALC_STATUS_LEAK_E_MAX;
	set_leak_status(led_leak, 1);
	set_sticky_sensor_status((sensor_num == SENSOR_NUM_BPB_CDU_COOLANT_LEAKAGE_VOLT_V) ?
					 STICKY_RPU_INTERNAL_LEAKAGE_ABNORMAL :
					 STICKY_HEX_RACK_PAN_LEAKAGE,
				 1);
	if (get_led_status(LED_IDX_E_LEAK) != LED_START_BLINK)
		led_ctrl(LED_IDX_E_LEAK, LED_START_BLINK);
	fault_led_control();
	gpio_set(RPU_LEAK_ALERT_N, 0);
}