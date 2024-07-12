#include "adm1272.h"
#include "hal_gpio.h"
#include "sensor.h"
#include "plat_isr.h"
#include "plat_gpio.h"
#include "plat_pwm.h"
#include "plat_hwmon.h"
#include "plat_sensor_table.h"
#include "plat_led.h"

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
#define IT_LEAK_ALERT_HANDLER(idx)                                                                 \
	void it_leak_alert_##idx(void)                                                             \
	{                                                                                          \
		deassert_all_rpu_ready_pin();                                                      \
		gpio_set(RPU_LEAK_ALERT_N, 0);                                                     \
	}

IT_LEAK_ALERT_HANDLER(0);
IT_LEAK_ALERT_HANDLER(1);
IT_LEAK_ALERT_HANDLER(2);
IT_LEAK_ALERT_HANDLER(3);

void fault_leak_action()
{
	//set_all_pump_power(false);
	//ctl_all_pwm_dev(0);
	deassert_all_rpu_ready_pin();
	gpio_set(RPU_LEAK_ALERT_N, 0);
}

void it_leak_action()
{
	fault_leak_action();
	led_ctrl(LED_IDX_E_FAULT, LED_TURN_ON);
	if (get_led_status(LED_IDX_E_LEAK) != LED_START_BLINK)
		led_ctrl(LED_IDX_E_LEAK, LED_START_BLINK);
}

K_WORK_DEFINE(it_leak_work, it_leak_action);

void ISR_IT_LEAK()
{
	k_work_submit(&it_leak_work);
}
