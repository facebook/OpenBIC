#include <stdio.h>
#include <drivers/sensor.h>
#include <drivers/pwm.h>
#include "fan.h"

// init fan device list
#undef DT_DRV_COMPAT
#if DT_HAS_COMPAT_STATUS_OKAY(aspeed_tach)
#define DT_DRV_COMPAT aspeed_tach
#endif

#define FAN_LABELS(node_id) { .device_label = DT_LABEL(node_id) },
#define FAN_NODE_LABELS(n) DT_FOREACH_CHILD(DT_DRV_INST(n), FAN_LABELS)
#define FAN_INIT_MACRO() DT_INST_FOREACH_STATUS_OKAY(FAN_NODE_LABELS)
static struct fan_handle {
	char *device_label;
} fan_list[] = { FAN_INIT_MACRO() };

static uint8_t ctrl_fan_mode;

static int pwm_record[2][4] = { { 0, 0, 0, 0 }, //[0][x] - slot1 BMC
				{ 0, 0, 0, 0 } }; // [1][x] - slot3 BMC

void fan_mode_init()
{
	ctrl_fan_mode = FAN_AUTO_MODE;
	return;
}

int pal_get_fan_ctrl_mode(uint8_t *ctrl_mode)
{
	if (ctrl_mode == NULL) {
		printf("Failed to get fan control state due to parameter is NULL.\n");
		return -1;
	}

	*ctrl_mode = ctrl_fan_mode;
	return 0;
}

void pal_set_fan_ctrl_mode(uint8_t ctrl_mode)
{
	ctrl_fan_mode = ctrl_mode;
	return;
}

int pal_get_fan_rpm(uint8_t fan_id, uint16_t *rpm)
{
	const struct device *fan_dev;
	struct sensor_value sensor_value;
	int ret = 0;

	if (rpm == NULL) {
		return -1;
	}

	fan_dev = device_get_binding(fan_list[fan_id].device_label);
	if (fan_dev == NULL) {
		printf("FAN%d device not found\n", fan_id);
		return -ENODEV;
	}

	ret = sensor_sample_fetch(fan_dev);
	if (ret < 0) {
		printf("Failed to read FAN%d due to sensor_sample_fetch failed, ret: %d\n", fan_id,
		       ret);
		return ret;
	}

	ret = sensor_channel_get(fan_dev, SENSOR_CHAN_RPM, &sensor_value);
	if (ret < 0) {
		printf("Failed to read FAN%d due to sensor_channel_get failed, ret: %d\n", fan_id,
		       ret);
		return ret;
	}

	*rpm = sensor_value.val1;
	return ret;
}

int pal_get_fan_duty(uint8_t pwm_id, uint8_t *duty, uint8_t slot_index)
{
	int ret = 0;

	if (slot_index == 0x01) {
		*duty = pwm_record[0][pwm_id];
	} else if (slot_index == 0x03) {
		*duty = pwm_record[1][pwm_id];
	} else {
		printf("Invalid slot index: %d\n", slot_index);
		ret = -1;
	}

	return ret;
}

int pal_set_fan_duty(uint8_t pwm_id, uint8_t duty, uint8_t slot_index)
{
	const struct device *pwm_dev;
	uint8_t final_duty = 0;
	int ret = 0;

	pwm_dev = device_get_binding(PWM_DEVICE_NAME);
	if (pwm_dev == NULL) {
		printf("PWM device not found\n");
		return -1;
	}

	if (ctrl_fan_mode == FAN_AUTO_MODE) {
		// Auto mode need to compare slot1 and slot3, and set the higher one
		if (slot_index == 0x01) {
			if (duty >= pwm_record[1][pwm_id]) {
				final_duty = duty;
			} else {
				final_duty = pwm_record[1][pwm_id];
			}

		} else if (slot_index == 0x03) {
			if (duty >= pwm_record[0][pwm_id]) {
				final_duty = duty;
			} else {
				final_duty = pwm_record[0][pwm_id];
			}
		} else {
			printf("Invalid slot index: %d\n", slot_index);
			return -1;
		}

	} else if (ctrl_fan_mode == FAN_MANUAL_MODE) {
		final_duty = duty;
	}

	ret = pwm_pin_set_cycles(pwm_dev, pwm_id, MAX_FAN_DUTY, final_duty, 0);
	// If set pwm successful, update pwm table
	if (ret == 0) {
		if (slot_index == 0x01) {
			pwm_record[0][pwm_id] = duty;

		} else if (slot_index == 0x03) {
			pwm_record[1][pwm_id] = duty;
		}
	}

	return ret;
}
