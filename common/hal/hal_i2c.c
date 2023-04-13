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

#include <zephyr.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cmsis_os2.h"
#include "hal_i2c.h"
#include "timer.h"
#include "plat_i2c.h"
#include "libutil.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(hal_i2c);

#define AST_1030_I2C_BASE 0x7e7b0080
#define AST_1030_I2C_REG_LEN 0x80
#define AST_1030_SLAVE_EN BIT(1)

static const struct device *dev_i2c[I2C_BUS_MAX_NUM];

struct k_mutex i2c_mutex[I2C_BUS_MAX_NUM];

int i2c_freq_set(uint8_t i2c_bus, uint8_t i2c_speed_mode, uint8_t en_slave)
{
	if (check_i2c_bus_valid(i2c_bus) < 0) {
		LOG_ERR("i2c bus %d is invalid", i2c_bus);
		return -1;
	}

	uint32_t dev_config_raw;
	dev_config_raw = I2C_MODE_MASTER | I2C_SPEED_SET(i2c_speed_mode);

	if (i2c_configure(dev_i2c[i2c_bus], dev_config_raw)) {
		LOG_ERR("i2c freq set failed");
		return -1;
	}

	if (en_slave) {
		uint32_t *addr = (uint32_t *)(AST_1030_I2C_BASE + (i2c_bus * AST_1030_I2C_REG_LEN));
		*addr |= AST_1030_SLAVE_EN;
	}

	return 0;
}

int i2c_master_read(I2C_MSG *msg, uint8_t retry)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -1);

	LOG_DBG("bus %d, addr %x, rxlen %d, txlen %d", msg->bus, msg->target_addr, msg->rx_len,
		msg->tx_len);
	LOG_HEXDUMP_DBG(msg->data, msg->tx_len, "txbuf");

	if (check_i2c_bus_valid(msg->bus) < 0) {
		LOG_ERR("i2c bus %d is invalid", msg->bus);
		return -1;
	}

	if (msg->rx_len == 0) {
		LOG_ERR("rx_len = 0");
		return EMSGSIZE;
	}

	if (msg->tx_len > I2C_BUFF_SIZE) {
		LOG_ERR("tx_len %d is over limit %d", msg->tx_len, I2C_BUFF_SIZE);
		return -1;
	}

	int status;
	status = k_mutex_lock(&i2c_mutex[msg->bus], K_MSEC(1000));
	if (status) {
		LOG_ERR("I2C %d master read get mutex timeout with ret %d", msg->bus, status);
		return ENOLCK;
	}

	int ret = -1;
	uint8_t *txbuf = NULL, *rxbuf = NULL;
	txbuf = (uint8_t *)malloc(I2C_BUFF_SIZE * sizeof(uint8_t));
	if (!txbuf) {
		LOG_ERR("Failed to malloc txbuf");
		goto exit;
	}
	rxbuf = (uint8_t *)malloc(I2C_BUFF_SIZE * sizeof(uint8_t));
	if (!rxbuf) {
		LOG_ERR("Failed to malloc rxbuf");
		goto exit;
	}
	memcpy(txbuf, &msg->data[0], msg->tx_len);

	uint8_t i;
	for (i = 0; i <= retry; i++) {
		if (msg->tx_len > 0) {
			ret = i2c_write_read(dev_i2c[msg->bus], msg->target_addr, txbuf,
					     msg->tx_len, rxbuf, msg->rx_len);
		} else {
			ret = i2c_read(dev_i2c[msg->bus], rxbuf, msg->rx_len, msg->target_addr);
		}
		if (ret == 0) { // i2c write read success
			memcpy(&msg->data[0], rxbuf, msg->rx_len);
			LOG_HEXDUMP_DBG(msg->data, msg->rx_len, "rxbuf");
			break;
		}
	}

	if (i > retry)
		LOG_ERR("I2C %d master read retry reach max with ret %d", msg->bus, ret);

exit:
	SAFE_FREE(txbuf);
	SAFE_FREE(rxbuf);

	status = k_mutex_unlock(&i2c_mutex[msg->bus]);
	if (status)
		LOG_ERR("I2C %d master read release mutex fail with ret %d", msg->bus, status);

	return ret;
}

int i2c_master_write(I2C_MSG *msg, uint8_t retry)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -1);

	LOG_DBG("bus %d, addr %x, txlen %d", msg->bus, msg->target_addr, msg->tx_len);
	LOG_HEXDUMP_DBG(msg->data, msg->tx_len, "txbuf");

	if (check_i2c_bus_valid(msg->bus) < 0) {
		LOG_ERR("i2c bus %d is invalid", msg->bus);
		return -1;
	}

	if (msg->tx_len > I2C_BUFF_SIZE) {
		LOG_ERR("tx_len %d is over limit %d", msg->tx_len, I2C_BUFF_SIZE);
		return -1;
	}

	int status;
	status = k_mutex_lock(&i2c_mutex[msg->bus], K_MSEC(1000));
	if (status) {
		LOG_ERR("I2C %d master write get mutex timeout with ret %d", msg->bus, status);
		return ENOLCK;
	}

	int ret = -1;
	uint8_t *txbuf = NULL;
	txbuf = (uint8_t *)malloc(I2C_BUFF_SIZE * sizeof(uint8_t));
	if (!txbuf) {
		LOG_ERR("Failed to malloc txbuf");
		goto exit;
	}
	memcpy(txbuf, &msg->data[0], msg->tx_len);

	uint8_t i;
	for (i = 0; i <= retry; i++) {
		ret = i2c_write(dev_i2c[msg->bus], txbuf, msg->tx_len, msg->target_addr);
		if (ret == 0) // i2c write success
			break;
	}

	if (i > retry)
		LOG_ERR("I2C %d master write retry reach max with ret %d", msg->bus, ret);

exit:
	SAFE_FREE(txbuf);

	status = k_mutex_unlock(&i2c_mutex[msg->bus]);
	if (status)
		LOG_ERR("I2C %d master write release mutex fail with ret %d", msg->bus, status);

	return ret;
}

int i2c_master_read_without_mutex(I2C_MSG *msg, uint8_t retry)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -1);

	LOG_DBG("bus %d, addr %x, rxlen %d, txlen %d", msg->bus, msg->target_addr, msg->rx_len,
		msg->tx_len);
	LOG_HEXDUMP_DBG(msg->data, msg->tx_len, "txbuf");

	if (check_i2c_bus_valid(msg->bus) < 0) {
		LOG_ERR("i2c bus %d is invalid", msg->bus);
		return -1;
	}

	if (msg->rx_len == 0) {
		LOG_ERR("rx_len = 0");
		return EMSGSIZE;
	}

	if (msg->tx_len > I2C_BUFF_SIZE) {
		LOG_ERR("tx_len %d is over limit %d", msg->tx_len, I2C_BUFF_SIZE);
		return -1;
	}

	int ret = -1;
	uint8_t *txbuf = NULL, *rxbuf = NULL;
	txbuf = (uint8_t *)malloc(I2C_BUFF_SIZE * sizeof(uint8_t));
	if (!txbuf) {
		LOG_ERR("Failed to malloc txbuf");
		goto exit;
	}
	rxbuf = (uint8_t *)malloc(I2C_BUFF_SIZE * sizeof(uint8_t));
	if (!rxbuf) {
		LOG_ERR("Failed to malloc rxbuf");
		goto exit;
	}
	memcpy(txbuf, &msg->data[0], msg->tx_len);

	uint8_t i;
	for (i = 0; i <= retry; i++) {
		if (msg->tx_len > 0) {
			ret = i2c_write_read(dev_i2c[msg->bus], msg->target_addr, txbuf,
					     msg->tx_len, rxbuf, msg->rx_len);
		} else {
			ret = i2c_read(dev_i2c[msg->bus], rxbuf, msg->rx_len, msg->target_addr);
		}
		if (ret == 0) { // i2c write read success
			memcpy(&msg->data[0], rxbuf, msg->rx_len);
			LOG_HEXDUMP_DBG(msg->data, msg->rx_len, "rxbuf");
			break;
		}
	}

	if (i > retry)
		LOG_ERR("I2C %d master read retry reach max with ret %d", msg->bus, ret);

exit:
	SAFE_FREE(txbuf);
	SAFE_FREE(rxbuf);

	return ret;
}

int i2c_master_write_without_mutex(I2C_MSG *msg, uint8_t retry)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -1);

	LOG_DBG("bus %d, addr %x, txlen %d", msg->bus, msg->target_addr, msg->tx_len);
	LOG_HEXDUMP_DBG(msg->data, msg->tx_len, "txbuf");

	if (check_i2c_bus_valid(msg->bus) < 0) {
		LOG_ERR("i2c bus %d is invalid", msg->bus);
		return -1;
	}

	if (msg->tx_len > I2C_BUFF_SIZE) {
		LOG_ERR("tx_len %d is over limit %d", msg->tx_len, I2C_BUFF_SIZE);
		return -1;
	}

	int ret = -1;
	uint8_t *txbuf = NULL;
	txbuf = (uint8_t *)malloc(I2C_BUFF_SIZE * sizeof(uint8_t));
	if (!txbuf) {
		LOG_ERR("Failed to malloc txbuf");
		goto exit;
	}
	memcpy(txbuf, &msg->data[0], msg->tx_len);

	uint8_t i;
	for (i = 0; i <= retry; i++) {
		ret = i2c_write(dev_i2c[msg->bus], txbuf, msg->tx_len, msg->target_addr);
		if (ret == 0) // i2c write success
			break;
	}

	if (i > retry)
		LOG_ERR("I2C %d master write retry reach max with ret %d", msg->bus, ret);

exit:
	SAFE_FREE(txbuf);

	return ret;
}

void i2c_scan(uint8_t bus, uint8_t *target_addr, uint8_t *target_addr_len)
{
	CHECK_NULL_ARG(target_addr);
	CHECK_NULL_ARG(target_addr_len);

	uint8_t first = 0x04, last = 0x77;
	*target_addr_len = 0;

	if (check_i2c_bus_valid(bus) < 0) {
		LOG_ERR("i2c bus %d is invalid", bus);
		return;
	}

	for (uint8_t i = 0; i <= last; i += 16) {
		for (uint8_t j = 0; j < 16; j++) {
			if (i + j < first || i + j > last) {
				continue;
			}

			struct i2c_msg msgs[1];
			uint8_t dst;

			/* Send the address to read from */
			msgs[0].buf = &dst;
			msgs[0].len = 0U;
			msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
			if (i2c_transfer(dev_i2c[bus], &msgs[0], 1, i + j) == 0) {
				target_addr[*target_addr_len] = (i + j) << 1;
				(*target_addr_len)++;
			}
		}
	}
}

void util_init_I2C(void)
{
	int status;

#ifdef DEV_I2C_0
	dev_i2c[0] = device_get_binding("I2C_0");
	status = k_mutex_init(&i2c_mutex[0]);
	if (status)
		LOG_ERR("i2c0 mutex init fail");
#endif
#ifdef DEV_I2C_1
	dev_i2c[1] = device_get_binding("I2C_1");
	status = k_mutex_init(&i2c_mutex[1]);
	if (status)
		LOG_ERR("i2c1 mutex init fail");
#endif
#ifdef DEV_I2C_2
	dev_i2c[2] = device_get_binding("I2C_2");
	status = k_mutex_init(&i2c_mutex[2]);
	if (status)
		LOG_ERR("i2c2 mutex init fail");
#endif
#ifdef DEV_I2C_3
	dev_i2c[3] = device_get_binding("I2C_3");
	status = k_mutex_init(&i2c_mutex[3]);
	if (status)
		LOG_ERR("i2c3 mutex init fail");
#endif
#ifdef DEV_I2C_4
	dev_i2c[4] = device_get_binding("I2C_4");
	status = k_mutex_init(&i2c_mutex[4]);
	if (status)
		LOG_ERR("i2c4 mutex init fail");
#endif
#ifdef DEV_I2C_5
	dev_i2c[5] = device_get_binding("I2C_5");
	status = k_mutex_init(&i2c_mutex[5]);
	if (status)
		LOG_ERR("i2c5 mutex init fail");
#endif
#ifdef DEV_I2C_6
	dev_i2c[6] = device_get_binding("I2C_6");
	status = k_mutex_init(&i2c_mutex[6]);
	if (status)
		LOG_ERR("i2c6 mutex init fail");
#endif
#ifdef DEV_I2C_7
	dev_i2c[7] = device_get_binding("I2C_7");
	status = k_mutex_init(&i2c_mutex[7]);
	if (status)
		LOG_ERR("i2c7 mutex init fail");
#endif
#ifdef DEV_I2C_8
	dev_i2c[8] = device_get_binding("I2C_8");
	status = k_mutex_init(&i2c_mutex[8]);
	if (status)
		LOG_ERR("i2c8 mutex init fail");
#endif
#ifdef DEV_I2C_9
	dev_i2c[9] = device_get_binding("I2C_9");
	status = k_mutex_init(&i2c_mutex[9]);
	if (status)
		LOG_ERR("i2c9 mutex init fail");
#endif
#ifdef DEV_I2C_10
	dev_i2c[10] = device_get_binding("I2C_10");
	status = k_mutex_init(&i2c_mutex[10]);
	if (status)
		LOG_ERR("i2c10 mutex init fail");
#endif
#ifdef DEV_I2C_11
	dev_i2c[11] = device_get_binding("I2C_11");
	status = k_mutex_init(&i2c_mutex[11]);
	if (status)
		LOG_ERR("i2c11 mutex init fail");
#endif
#ifdef DEV_I2C_12
	dev_i2c[12] = device_get_binding("I2C_12");
	status = k_mutex_init(&i2c_mutex[12]);
	if (status)
		LOG_ERR("i2c12 mutex init fail");
#endif
#ifdef DEV_I2C_13
	dev_i2c[13] = device_get_binding("I2C_13");
	status = k_mutex_init(&i2c_mutex[13]);
	if (status)
		LOG_ERR("i2c13 mutex init fail");
#endif
#ifdef DEV_I2C_14
	dev_i2c[14] = device_get_binding("I2C_14");
	status = k_mutex_init(&i2c_mutex[14]);
	if (status)
		LOG_ERR("i2c14 mutex init fail");
#endif
#ifdef DEV_I2C_15
	dev_i2c[15] = device_get_binding("I2C_15");
	status = k_mutex_init(&i2c_mutex[15]);
	if (status)
		LOG_ERR("i2c15 mutex init fail");
#endif
}

int check_i2c_bus_valid(uint8_t bus)
{
	if (dev_i2c[bus] == NULL) {
		return -1;
	}
	return 0;
}
