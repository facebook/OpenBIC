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

void deassert_all_rpu_ready_pin(void)
{
	gpio_set(BIC_RPU_READY0, 0);
	gpio_set(BIC_RPU_READY1, 0);
	gpio_set(BIC_RPU_READY2, 0);
	gpio_set(BIC_RPU_READY3, 0);
}

/* TO DO: 
	1. turn off pump
	2. change hx fan to min speed
	3. store to non volatile memory
*/
void fault_leak_action()
{
	set_pwm_group(PWM_GROUP_E_PUMP, 0);
	//ctl_all_pwm_dev(0);
	deassert_all_rpu_ready_pin();
	gpio_set(RPU_LEAK_ALERT_N, 0);
}

void it_leak_handler(uint8_t idx)
{
	uint8_t sen_num = (idx == IT_LEAK_E_0) ?
				  SENSOR_NUM_IT_LEAK_0_GPIO :
				  (idx == IT_LEAK_E_1) ?
				  SENSOR_NUM_IT_LEAK_1_GPIO :
				  (idx == IT_LEAK_E_2) ?
				  SENSOR_NUM_IT_LEAK_2_GPIO :
				  (idx == IT_LEAK_E_3) ? SENSOR_NUM_IT_LEAK_3_GPIO : 0xFF;

	fault_leak_action();
	error_log_event(sen_num, IS_ABNORMAL_VAL);

	led_ctrl(LED_IDX_E_FAULT, LED_TURN_ON);
	if (get_led_status(LED_IDX_E_LEAK) == LED_START_BLINK)
		led_ctrl(LED_IDX_E_LEAK, LED_STOP_BLINK);
	led_ctrl(LED_IDX_E_LEAK, LED_TURN_OFF);
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
