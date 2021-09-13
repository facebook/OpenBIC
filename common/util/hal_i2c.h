#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <drivers/i2c.h>
#include <drivers/i2c/slave/ipmb.h>

extern const uint8_t i2c_bus_to_index[];

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

#define DEV_I2C(n) DEV_I2C_##n

#define I2C_BUFF_SIZE 256
#define MAX_I2C_BUS_NUM 16
#define DEBUG_I2C 0

enum {
  smc_i2c0,
  smc_i2c1,
  smc_i2c2,
  smc_i2c3,
  smc_i2c4,
  smc_i2c5,
  smc_i2c6,
  smc_i2c7,
  smc_i2c8,
  smc_i2c9,
};

typedef struct _I2C_MSG_ {
  uint8_t slave_addr;
  uint8_t bus;
  uint8_t rx_len;
  uint8_t tx_len;
  uint8_t data[I2C_BUFF_SIZE];
  struct k_mutex lock;
} I2C_MSG;

int i2c_master_read(I2C_MSG *msg, uint8_t retry);
int i2c_master_write(I2C_MSG *msg, uint8_t retry);
void util_init_I2C(void);

#endif
