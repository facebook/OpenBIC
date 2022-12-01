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

#include "hal_i3c.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr.h>
#include <logging/log.h>
#include "libutil.h"

LOG_MODULE_REGISTER(hal_i3c);

static const struct device *dev_i3c[I3C_MAX_NUM];
static const struct device *dev_i3c_smq[I3C_MAX_NUM];
static struct i3c_dev_desc i3c_desc_table[I3C_MAX_NUM];
static int i3c_desc_count = 0;
static struct k_mutex mutex_write[I3C_MAX_NUM];
static struct k_mutex mutex_read[I3C_MAX_NUM];

int i3c_slave_mqueue_read(const struct device *dev, uint8_t *dest, int budget);
int i3c_slave_mqueue_write(const struct device *dev, uint8_t *src, int size);

static struct i3c_dev_desc *find_matching_desc(const struct device *dev, uint8_t desc_addr)
{
	CHECK_NULL_ARG_WITH_RETURN(dev, NULL);

	struct i3c_dev_desc *desc = NULL;
	int i = 0;

	for (i = 0; i < I3C_MAX_NUM; i++) {
		desc = &i3c_desc_table[i];
		if ((desc->master_dev == dev) && (desc->info.dynamic_addr == desc_addr)) {
			return desc;
		}
	}

	return NULL;
}

/**
 * @brief api to read i3c message from target message queue
 * 
 * @param msg i3c message structure
 * @return ret: return the data size
 */
int i3c_smq_read(I3C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -EINVAL);

	if (!dev_i3c[msg->bus]) {
		return -ENODEV;
	}

	int ret = k_mutex_lock(&mutex_read[msg->bus], K_FOREVER);
	if (ret) {
		printf("[%s] Failed to lock the mutex(%d)\n", __func__, ret);
		return -1;
	}

	msg->rx_len =
		i3c_slave_mqueue_read(dev_i3c_smq[msg->bus], &msg->data[0], I3C_MAX_DATA_SIZE);
	if (msg->rx_len == 0) {
		k_mutex_unlock(&mutex_read[msg->bus]);
		return -ENODATA;
	}

	k_mutex_unlock(&mutex_read[msg->bus]);
	return msg->rx_len;
}

/**
 * @brief api to write i3c message to target message queue
 * 
 * @param msg i3c message structure
 * @return 0: api to write i3c message to target message queue
 */
int i3c_smq_write(I3C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -EINVAL);

	int ret;
	ret = k_mutex_lock(&mutex_write[msg->bus], K_FOREVER);
	if (ret) {
		printf("[%s] Failed to lock the mutex(%d)\n", __func__, ret);
		return -1;
	}

	if (!dev_i3c[msg->bus]) {
		LOG_ERR("[%s] bus%u did not define\n", __func__, msg->bus);
		k_mutex_unlock(&mutex_write[msg->bus]);
		return -ENODEV;
	}

	ret = i3c_slave_mqueue_write(dev_i3c_smq[msg->bus], &msg->data[0], msg->tx_len);
	if (ret < 0) {
		LOG_ERR("[%s] i3c wrtie failed, ret = %d",__func__, ret);
	}
	k_mutex_unlock(&mutex_write[msg->bus]);

	return ret;
}

int i3c_attach(I3C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -EINVAL);

	int ret = 0;
	struct i3c_dev_desc *desc;

	if (!dev_i3c[msg->bus]) {
		LOG_ERR("Failed to attach address 0x%x due to undefined bus%u", msg->target_addr,
			msg->bus);
		return -ENODEV;
	}

	desc = &i3c_desc_table[i3c_desc_count];
	desc->info.assigned_dynamic_addr = msg->target_addr;
	desc->info.static_addr = desc->info.assigned_dynamic_addr;
	desc->info.i2c_mode = 0;

	ret = i3c_master_attach_device(dev_i3c[msg->bus], desc);
	if (ret == 0) {
		i3c_desc_count++;
	} else {
		LOG_ERR("Failed to attach address 0x%x, ret: %d", msg->target_addr, ret);
	}

	return -ret;
}

int i3c_brocast_ccc(I3C_MSG *msg, uint8_t ccc_id, uint8_t ccc_addr)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -EINVAL);

	if (!dev_i3c[msg->bus]) {
		LOG_ERR("Failed to broadcast ccc 0x%x to addresses 0x%x due to undefined bus%u",
			ccc_id, ccc_addr, msg->bus);
		return -ENODEV;
	}

	int ret = 0;
	struct i3c_ccc_cmd ccc;

	memset(&ccc, 0, sizeof(struct i3c_ccc_cmd));
	ccc.addr = ccc_addr;
	ccc.id = ccc_id;
	ccc.rnw = I3C_WRITE_CMD;
	ccc.payload.data = &msg->data;
	ccc.payload.length = 0;

	ret = i3c_master_send_ccc(dev_i3c[msg->bus], &ccc);
	if (ret != 0) {
		LOG_ERR("Failed to broadcast ccc 0x%x to addresses 0x%x due to undefined bus%u",
			ccc_id, ccc_addr, ret);
	}

	return -ret;
}

int i3c_transfer(I3C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -EINVAL);

	if (!dev_i3c[msg->bus]) {
		LOG_ERR("Failed to transfer messages to address 0x%x due to undefined bus%u",
			msg->target_addr, msg->bus);
		return -ENODEV;
	}

	int ret = 0;
	struct i3c_dev_desc *desc;
	struct i3c_priv_xfer xfer_data[I3C_MAX_BUFFER_COUNT];

	desc = find_matching_desc(dev_i3c[msg->bus], msg->target_addr);
	if (desc == NULL) {
		LOG_ERR("Failed to transfer messages to address 0x%x due to unknown address",
			msg->target_addr);
		return -ENODEV;
	}

	xfer_data[0].rnw = I3C_WRITE_CMD;
	xfer_data[0].len = msg->tx_len;
	xfer_data[0].data.out = &msg->data;

	xfer_data[1].rnw = I3C_READ_CMD;
	xfer_data[1].len = msg->rx_len;
	xfer_data[1].data.in = &msg->data;

	ret = i3c_master_priv_xfer(desc, xfer_data, 2);
	if (ret != 0) {
		LOG_ERR("Failed to transfer messages to addr 0x%x, ret: %d", msg->target_addr, ret);
	}

	return -ret;
}

void util_init_i3c(void)
{
#ifdef DEV_I3C_0
	dev_i3c[0] = device_get_binding("I3C_0");
#endif
#ifdef DEV_I3C_1
	dev_i3c[1] = device_get_binding("I3C_1");
#endif
#ifdef DEV_I3C_2
	dev_i3c[2] = device_get_binding("I3C_2");
#endif
#ifdef DEV_I3C_3
	dev_i3c[3] = device_get_binding("I3C_3");
#endif
#ifdef DEV_I3C_4
	dev_i3c[4] = device_get_binding("I3C_4");
#endif
#ifdef DEV_I3C_5
	dev_i3c[5] = device_get_binding("I3C_5");
#endif
#ifdef DEV_I3C_6
	dev_i3c[6] = device_get_binding("I3C_6");
#endif
#ifdef DEV_I3C_7
	dev_i3c[7] = device_get_binding("I3C_7");
#endif

#ifdef DEV_I3CSMQ_0
	dev_i3c_smq[0] = device_get_binding("I3C_SMQ_0");
#endif
#ifdef DEV_I3CSMQ_1
	dev_i3c_smq[1] = device_get_binding("I3C_SMQ_1");
#endif
#ifdef DEV_I3CSMQ_2
	dev_i3c_smq[2] = device_get_binding("I3C_SMQ_2");
#endif
#ifdef DEV_I3CSMQ_3
	dev_i3c_smq[3] = device_get_binding("I3C_SMQ_3");
#endif
#ifdef DEV_I3CSMQ_4
	dev_i3c_smq[4] = device_get_binding("I3C_SMQ_4");
#endif
#ifdef DEV_I3CSMQ_5
	dev_i3c_smq[5] = device_get_binding("I3C_SMQ_5");
#endif
#ifdef DEV_I3CSMQ_6
	dev_i3c_smq[6] = device_get_binding("I3C_SMQ_6");
#endif
#ifdef DEV_I3CSMQ_7
	dev_i3c_smq[7] = device_get_binding("I3C_SMQ_7");
#endif
	for (int i = 0; i <= I3C_MAX_NUM; ++i) {
		if (k_mutex_init(&mutex_read[i])) {
			LOG_ERR("Mutex %d init error.",i);
		}
		if (k_mutex_init(&mutex_write[i])) {
			LOG_ERR("Mutex %d init error.",i);
		}
	}
}
