#ifndef HAL_I2C_SLAVE_H
#define HAL_I2C_SLAVE_H

#include <drivers/i2c.h>
#include "hal_i2c.h"

#define DEBUG_MODE 0

#define MAX_I2C_SLAVE_BUFF 512
#define MAX_SLAVE_NUM 16
#define I2C_DEVICE_PREFIX "I2C_"
#define I2C_CONTROLLER_NAME_GET(inst) I2C_DEVICE_PREFIX #inst

struct __attribute__((__packed__)) i2c_msg_package {
    uint16_t msg_length;
    uint8_t msg[MAX_I2C_SLAVE_BUFF];
};

struct i2c_slave_data {
    uint8_t i2c_bus;                        /* i2c bus number */
    const struct device *i2c_controller;    /* i2c controller for one slave bus */
    struct i2c_slave_config config;         /* i2c slave relative config */
    uint16_t max_msg_count;                 /* max message count that slave could handle */
    uint32_t buffer_idx;                    /* index point to array that store message */
    struct i2c_msg_package current_msg;     /* store message relative stuff */
    struct k_msgq z_msgq_id;                /* message queue of Zephyr api */
};

struct _i2c_slave_config {
    uint8_t address;
    uint32_t i2c_msg_count;
};

struct i2c_slave_device
{
    struct i2c_slave_data data;
    bool is_init;
    bool is_register;
};

/* Retern value set for i2c slave status */
enum i2c_slave_error_status{
    I2C_SLAVE_HAS_NO_ERR,
    I2C_SLAVE_BUS_INVALID,
    I2C_SLAVE_NOT_INIT,
    I2C_SLAVE_NOT_REGISTER = 0x04,
    I2C_SLAVE_CONTROLLER_ERR = 0x08, /* Might heapen if bus controler is not enable in device tree */
};

/* Retern value set for i2c slave api status */
enum i2c_slave_api_error_status{
    I2C_SLAVE_API_NO_ERR,
    I2C_SLAVE_API_INPUT_ERR,
    I2C_SLAVE_API_LOCK_ERR,
    I2C_SLAVE_API_MEMORY_ERR,
    I2C_SLAVE_API_MSGQ_ERR,
    I2C_SLAVE_API_BUS_GET_FAIL,
    I2C_SLAVE_API_UNKNOWN_ERR = 0xFF
};

/* Mode of "i2c_slave_control" */
enum i2c_slave_api_control_mode{
    I2C_CONTROL_UNREGISTER,
    I2C_CONTROL_REGISTER,
    I2C_CONTROL_MAX = 0xFF
};

uint8_t i2c_slave_status_get(uint8_t bus_num);
uint8_t i2c_slave_status_print(uint8_t bus_num);
uint8_t i2c_slave_cfg_get(uint8_t bus_num, struct _i2c_slave_config *cfg);
uint8_t i2c_slave_read(uint8_t bus_num, uint8_t *buff, uint16_t buff_len, uint16_t *msg_len);
int i2c_slave_control(uint8_t bus_num, struct _i2c_slave_config *cfg, enum i2c_slave_api_control_mode mode);

void util_init_I2C_slave(void);

#endif