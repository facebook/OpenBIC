/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <drivers/i2c.h>
#include <drivers/i2c/slave/ipmb.h>

#if defined(CONFIG_I2C_ASPEED)
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

#elif (CONFIG_I2C_NPCM4XX)
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1a), okay)
#define DEV_I2C_0
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1b), okay)
#define DEV_I2C_0
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c2a), okay)
#define DEV_I2C_1
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c3a), okay)
#define DEV_I2C_2
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c4a), okay)
#define DEV_I2C_3
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c4b), okay)
#define DEV_I2C_3
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c5a), okay)
#define DEV_I2C_4
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c6a), okay)
#define DEV_I2C_5
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c6b), okay)
#define DEV_I2C_5
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c7a), okay)
#define DEV_I2C_6
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c8a), okay)
#define DEV_I2C_7
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c9a), okay)
#define DEV_I2C_8
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c10a), okay)
#define DEV_I2C_9
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c11a), okay)
#define DEV_I2C_10
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c12a), okay)
#define DEV_I2C_11
#endif

#else /* defined(CONFIG_GPIO_ASPEED) */
#endif /* defined(CONFIG_GPIO_ASPEED) */

#define DEV_I2C(n) DEV_I2C_##n

#define I2C_BUFF_SIZE 256
#define MUTEX_LOCK_ENABLE true
#define MUTEX_LOCK_DISENABLE false

enum I2C_TRANSFER_TYPE {
	I2C_READ,
	I2C_WRITE,
};

typedef struct _I2C_MSG_ {
	uint8_t bus;
	uint8_t target_addr;
	uint8_t rx_len;
	uint8_t tx_len;
	uint8_t data[I2C_BUFF_SIZE];
	struct k_mutex lock;
} I2C_MSG;

int i2c_freq_set(uint8_t i2c_bus, uint8_t i2c_speed_mode, uint8_t en_slave);
int i2c_addr_set(uint8_t i2c_bus, uint8_t i2c_addr);
int i2c_master_read(I2C_MSG *msg, uint8_t retry);
int i2c_master_read_without_mutex(I2C_MSG *msg, uint8_t retry);
int i2c_master_write(I2C_MSG *msg, uint8_t retry);
int i2c_master_write_without_mutex(I2C_MSG *msg, uint8_t retry);
void i2c_scan(uint8_t bus, uint8_t *target_addr, uint8_t *target_addr_len);
void util_init_I2C(void);
int check_i2c_bus_valid(uint8_t bus);

#endif
