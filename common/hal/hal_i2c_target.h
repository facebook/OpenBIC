#ifndef HAL_I2C_TARGET_H
#define HAL_I2C_TARGET_H

#include <drivers/i2c.h>
#include "hal_i2c.h"

#define MAX_I2C_TARGET_BUFF 512
#define MAX_TARGET_NUM 16
#define I2C_DEVICE_PREFIX "I2C_"
#define I2C_CONTROLLER_NAME_GET(inst) I2C_DEVICE_PREFIX #inst

struct __attribute__((__packed__)) i2c_msg_package {
	uint16_t msg_length;
	uint8_t msg[MAX_I2C_TARGET_BUFF];
};

struct i2c_target_data {
	uint8_t i2c_bus; /* i2c bus number */
	const struct device *i2c_controller; /* i2c controller for one target bus */
	struct i2c_slave_config config; /* i2c target relative config */
	uint16_t max_msg_count; /* max message count that target could handle */
	uint32_t buffer_idx; /* index point to array that store message */
	struct i2c_msg_package current_msg; /* store message relative stuff */
	struct k_msgq z_msgq_id; /* message queue of Zephyr api */
};

struct _i2c_target_config {
	uint8_t address;
	uint32_t i2c_msg_count;
};

struct i2c_target_device {
	struct i2c_target_data data;
	bool is_init;
	bool is_register;
};

/* Retern value set for i2c target status */
enum i2c_target_error_status {
	I2C_TARGET_HAS_NO_ERR,
	I2C_TARGET_BUS_INVALID,
	I2C_TARGET_NOT_INIT,
	I2C_TARGET_NOT_REGISTER = 0x04,
	I2C_TARGET_CONTROLLER_ERR =
		0x08, /* Might heapen if bus controler is not enable in device tree */
};

/* Retern value set for i2c target api status */
enum i2c_target_api_error_status {
	I2C_TARGET_API_NO_ERR,
	I2C_TARGET_API_INPUT_ERR,
	I2C_TARGET_API_LOCK_ERR,
	I2C_TARGET_API_MEMORY_ERR,
	I2C_TARGET_API_MSGQ_ERR,
	I2C_TARGET_API_BUS_GET_FAIL,
	I2C_TARGET_API_UNKNOWN_ERR = 0xFF
};

/* Mode of "i2c_target_control" */
enum i2c_target_api_control_mode {
	I2C_CONTROL_UNREGISTER,
	I2C_CONTROL_REGISTER,
	I2C_CONTROL_MAX = 0xFF
};

extern const bool I2C_TARGET_ENABLE_TABLE[MAX_TARGET_NUM];
extern const struct _i2c_target_config I2C_TARGET_CONFIG_TABLE[MAX_TARGET_NUM];

uint8_t i2c_target_status_get(uint8_t bus_num);
uint8_t i2c_target_status_print(uint8_t bus_num);
uint8_t i2c_target_cfg_get(uint8_t bus_num, struct _i2c_target_config *cfg);
uint8_t i2c_target_read(uint8_t bus_num, uint8_t *buff, uint16_t buff_len, uint16_t *msg_len);
int i2c_target_control(uint8_t bus_num, struct _i2c_target_config *cfg,
		       enum i2c_target_api_control_mode mode);

void util_init_I2C_target(void);

#endif
