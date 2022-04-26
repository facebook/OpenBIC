#ifndef PLAT_FAN_H
#define PLAT_FAN_H

#define MAX_FAN_DUTY_VALUE 100
#define DEFAULT_FAN_DUTY_VALUE 70
#define MAX_FAN_PWM_INDEX_COUNT 4

#define INDEX_ALL_PWM 0xFF

#define PWM_DEVICE_NAME "PWM"

enum fan_mode_ctrl_cmd {
	FAN_SET_MANUAL_MODE = 0x00,
	FAN_SET_AUTO_MODE = 0x01,
	FAN_GET_MODE = 0x02,
};

enum fan_mode {
	FAN_MANUAL_MODE = 0x00,
	FAN_AUTO_MODE = 0x01,
};

void init_fan_mode();
void init_fan_duty();
int pal_get_fan_ctrl_mode(uint8_t *ctrl_mode);
void pal_set_fan_ctrl_mode(uint8_t ctrl_mode);
int pal_get_fan_rpm(uint8_t fan_id, uint16_t *rpm);
int pal_get_fan_duty(uint8_t pwm_id, uint8_t *duty, uint8_t slot_index);
int pal_set_fan_duty(uint8_t pwm_id, uint8_t duty, uint8_t slot_index);

#endif
