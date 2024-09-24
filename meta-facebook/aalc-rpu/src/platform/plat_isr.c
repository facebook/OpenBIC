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
#include "plat_fru.h"

LOG_MODULE_REGISTER(plat_isr);

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
		set_status_flag(STATUS_FLAG_FAILURE, PUMP_FAIL_EMERGENCY_BUTTON, 0);
		if (rpu_ready_recovery())
			set_all_rpu_ready_pin_normal();
	} else {
		set_status_flag(STATUS_FLAG_FAILURE, PUMP_FAIL_EMERGENCY_BUTTON, 1);
		deassert_all_rpu_ready_pin();
	}
}

void fault_leak_action()
{
	set_status_flag(STATUS_FLAG_FAILURE, PUMP_FAIL_LEAK, 1);
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
		set_status_flag(STATUS_FLAG_LEAK, AALC_STATUS_IT_LEAK_0, 1);
		break;
	case IT_LEAK_E_1:
		set_sticky_sensor_status(STICKY_ITRACK_CHASSIS1_LEAKAGE, 1);
		set_status_flag(STATUS_FLAG_LEAK, AALC_STATUS_IT_LEAK_1, 1);
		break;
	case IT_LEAK_E_2:
		set_sticky_sensor_status(STICKY_ITRACK_CHASSIS2_LEAKAGE, 1);
		set_status_flag(STATUS_FLAG_LEAK, AALC_STATUS_IT_LEAK_2, 1);
		break;
	case IT_LEAK_E_3:
		set_sticky_sensor_status(STICKY_ITRACK_CHASSIS3_LEAKAGE, 1);
		set_status_flag(STATUS_FLAG_LEAK, AALC_STATUS_IT_LEAK_3, 1);
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
	set_status_flag(STATUS_FLAG_LEAK, led_leak, 1);
	set_sticky_sensor_status((sensor_num == SENSOR_NUM_BPB_CDU_COOLANT_LEAKAGE_VOLT_V) ?
					 STICKY_RPU_INTERNAL_LEAKAGE_ABNORMAL :
					 STICKY_HEX_RACK_PAN_LEAKAGE,
				 1);
	if (get_led_status(LED_IDX_E_LEAK) != LED_START_BLINK)
		led_ctrl(LED_IDX_E_LEAK, LED_START_BLINK);
	fault_led_control();
	gpio_set(RPU_LEAK_ALERT_N, 0);
}

bool plat_gpio_immediate_int_cb(uint8_t gpio_num)
{
	bool ret = false;

	if (gpio_num == PWRGD_P48V_BRI) {
		ret = true;
	}

	return ret;
}

void shutdown_save_uptime_action()
{
	uint8_t pre_time[EEPROM_UPTIME_SIZE] = { 0 };
	if (!plat_eeprom_read(EEPROM_UPTIME_OFFSET, pre_time, EEPROM_UPTIME_SIZE))
		LOG_ERR("read uptime fail!");

	uint32_t old_uptime =
		(pre_time[3] << 24) | (pre_time[2] << 16) | (pre_time[1] << 8) | pre_time[0];
	//LOG_INF("old uptime: %d\n", old_uptime);

	uint32_t get_uptime_mins = (k_uptime_get() / 60000); // mins(60 *1000 ms)
	//LOG_INF("uptime: %d\n", get_uptime_mins);

	get_uptime_mins += old_uptime;

	uint8_t temp[EEPROM_UPTIME_SIZE] = { 0 };
	memcpy(temp, &get_uptime_mins, EEPROM_UPTIME_SIZE);

	if (!plat_eeprom_write(EEPROM_UPTIME_OFFSET, temp, EEPROM_UPTIME_SIZE))
		LOG_ERR("write uptime fail!");
}