#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <drivers/i2c.h>
#include <drivers/i2c/slave/ipmb.h>

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c0), okay)
#define DEV_I2C_0
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay)
#define DEV_I2C_1
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c2), okay)
#define DEV_I2C_2
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c3), okay)
#define DEV_I2C_3
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c4), okay)
#define DEV_I2C_4
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c5), okay)
#define DEV_I2C_5
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c6), okay)
#define DEV_I2C_6
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c7), okay)
#define DEV_I2C_7
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c8), okay)
#define DEV_I2C_8
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c9), okay)
#define DEV_I2C_9
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c10), okay)
#define DEV_I2C_10
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c11), okay)
#define DEV_I2C_11
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c12), okay)
#define DEV_I2C_12
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c13), okay)
#define DEV_I2C_13
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c14), okay)
#define DEV_I2C_14
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c15), okay)
#define DEV_I2C_15
#endif

#define DEV_I2C(n) DEV_I2C_##n

#define I2C_BUFF_SIZE 256
#define DEBUG_I2C 0

typedef struct _I2C_MSG_ {
	uint8_t bus;
	uint8_t target_addr;
	uint8_t rx_len;
	uint8_t tx_len;
	uint8_t data[I2C_BUFF_SIZE];
	struct k_mutex lock;
} I2C_MSG;

void i2c_freq_set(uint8_t i2c_bus, uint8_t i2c_speed_mode);
int i2c_master_read(I2C_MSG *msg, uint8_t retry);
int i2c_master_write(I2C_MSG *msg, uint8_t retry);
void i2c_scan(uint8_t bus, uint8_t *target_addr, uint8_t *target_addr_len);
void util_init_I2C(void);
int check_i2c_bus_valid(uint8_t bus);

#endif
