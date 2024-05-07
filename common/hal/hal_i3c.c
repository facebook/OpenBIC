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
static i3c_ibi_dev i3c_ibi_dev_table[I3C_MAX_NUM];
static int i3c_desc_count = 0;
static struct k_mutex mutex_write[I3C_MAX_NUM];
static struct k_mutex mutex_read[I3C_MAX_NUM];

int i3c_slave_mqueue_read(const struct device *dev, uint8_t *dest, int budget);
int i3c_slave_mqueue_write(const struct device *dev, uint8_t *src, int size);

static struct i3c_dev_desc *find_matching_desc(const struct device *dev, uint8_t desc_addr,
					       int *pos)
{
	CHECK_NULL_ARG_WITH_RETURN(dev, NULL);

	struct i3c_dev_desc *desc = NULL;
	int i = 0;

	for (i = 0; i < I3C_MAX_NUM; i++) {
		desc = &i3c_desc_table[i];
		if ((desc->master_dev == dev) && (desc->info.dynamic_addr == desc_addr)) {
			if (pos == NULL) {
				return desc;
			}
			*pos = i;
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
		LOG_ERR("Failed to lock the mutex(%d)", ret);
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
		LOG_ERR("Failed to lock the mutex(%d)", ret);
		return -1;
	}

	if (!dev_i3c[msg->bus]) {
		LOG_ERR("Bus %u is not defined", msg->bus);
		k_mutex_unlock(&mutex_write[msg->bus]);
		return -ENODEV;
	}

	ret = i3c_slave_mqueue_write(dev_i3c_smq[msg->bus], &msg->data[0], msg->tx_len);
	if (ret < 0) {
		LOG_ERR("I3C wrtie failed, ret = %d", ret);
	}
	k_mutex_unlock(&mutex_write[msg->bus]);

	return ret;
}

int i3c_attach(I3C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -EINVAL);

	int ret = 0, pos = 0;
	struct i3c_dev_desc *desc = NULL;

	if (!dev_i3c[msg->bus]) {
		LOG_ERR("Failed to attach address 0x%x due to undefined bus%u", msg->target_addr,
			msg->bus);
		return -ENODEV;
	}

	desc = find_matching_desc(dev_i3c[msg->bus], msg->target_addr, &pos);
	if (desc != NULL) {
		LOG_INF("addr 0x%x already attach", msg->target_addr);
		return -ret;
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

int i3c_detach(I3C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -EINVAL);

	int ret = 0, pos = 0, i;
	struct i3c_dev_desc *desc;

	if (!dev_i3c[msg->bus]) {
		LOG_ERR("Failed to detach address 0x%x due to undefined bus%u", msg->target_addr,
			msg->bus);
		return -ENODEV;
	}

	desc = find_matching_desc(dev_i3c[msg->bus], msg->target_addr, &pos);
	if (desc == NULL) {
		LOG_ERR("Failed to detach address 0x%x due to unknown address", msg->target_addr);
		return -ENODEV;
	}

	ret = i3c_master_detach_device(dev_i3c[msg->bus], desc);
	if (ret != 0) {
		LOG_ERR("Failed to detach address 0x%x, ret: %d", msg->target_addr, ret);
		return -ret;
	}

	for (i = pos; i < i3c_desc_count; i++) {
		i3c_desc_table[i] = i3c_desc_table[i + 1];
	}
	i3c_desc_count--;

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
		LOG_ERR("Failed to broadcast ccc 0x%x to addresses 0x%x bus%u, ret%d",
                       ccc_id, ccc_addr, msg->bus, ret);
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

	int ret = 0, *pos = NULL;
	struct i3c_dev_desc *desc;
	struct i3c_priv_xfer xfer_data[I3C_MAX_BUFFER_COUNT];

	desc = find_matching_desc(dev_i3c[msg->bus], msg->target_addr, pos);
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

int i3c_spd_reg_read(I3C_MSG *msg, bool is_nvm)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -EINVAL);

	if (!dev_i3c[msg->bus]) {
		LOG_ERR("Failed to read spd 0x%x due to undefined bus%u", msg->target_addr,
			msg->bus);
		return -ENODEV;
	}

	int ret = 0, *pos = NULL;
	struct i3c_dev_desc *desc;
	uint8_t addr = (msg->data[1] << 8 | msg->data[0]) % 64;
	uint8_t block_addr = (msg->data[1] << 8 | msg->data[0]) / 64;
	uint8_t offset0 = GETBIT(block_addr, 0) << 6 | (addr & GENMASK(5, 0));
	uint8_t offset1 = (block_addr >> 1) & 0xF;
	if (is_nvm) {
		offset0 = SETBIT(offset0, 7);
	} else {
		offset0 = CLEARBIT(offset0, 7);
	}
	uint8_t offset[2] = { offset0, offset1 };

	desc = find_matching_desc(dev_i3c[msg->bus], msg->target_addr, pos);
	if (desc == NULL) {
		LOG_ERR("Failed to transfer messages to address 0x%x due to unknown address",
			msg->target_addr);
		return -ENODEV;
	}

	ret = i3c_jesd403_read(desc, offset, sizeof(offset), msg->data, msg->rx_len);
	if (ret != 0) {
		LOG_ERR("Failed to read SPD bus0x%x addr0x%x offset0x%x %x, ret: %d", msg->bus,
			msg->target_addr, offset[1], offset[0], ret);
	}

	return ret;
}

int i3c_controller_write(I3C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -EINVAL);

	int *pos = NULL;
	struct i3c_dev_desc *target;
	target = find_matching_desc(dev_i3c[msg->bus], msg->target_addr, pos);
	if (target == NULL) {
		LOG_ERR("Failed to write messages to address 0x%x due to unknown address",
			msg->target_addr);
		return -ENODEV;
	}

	struct i3c_priv_xfer xfer[1];

	xfer[0].rnw = I3C_WRITE_CMD;
	xfer[0].len = msg->tx_len;
	xfer[0].data.out = &msg->data;

	int ret = i3c_master_priv_xfer(target, xfer, 1);
	if (ret != 0) {
		LOG_ERR("Failed to write messages to bus 0x%d addr 0x%x, ret: %d", msg->bus,
			msg->target_addr, ret);
		return false;
	}

	return true;
}

int i3c_set_pid(I3C_MSG *msg, uint16_t slot_pid)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -EINVAL);

	int ret = i3c_set_pid_extra_info(dev_i3c[msg->bus], slot_pid);
	if (ret != 0) {
		LOG_ERR("Failed to set pid to bus 0x%d, ret: %d", msg->bus, ret);
		return false;
	}

	return true;
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
			LOG_ERR("Mutex %d init error.", i);
		}
		if (k_mutex_init(&mutex_write[i])) {
			LOG_ERR("Mutex %d init error.", i);
		}
	}
}

static int get_bus_id(struct i3c_dev_desc *desc)
{
	char i3c_dev_name[I3C_DEV_STR_LEN] = { 0 };
	const char delim[2] = "_";
	char *saveptr = NULL;
	char *substr = NULL;
	int bus_id;

	strncpy(i3c_dev_name, desc->master_dev->name, I3C_DEV_STR_LEN);

	substr = strtok_r(i3c_dev_name, delim, &saveptr);
	substr = strtok_r(NULL, delim, &saveptr);

	bus_id = atoi(substr);
	if (bus_id > I3C_MAX_NUM) {
		LOG_ERR("bus id out of range, bus = %d", bus_id);
		return -1;
	}

	return bus_id;
}

static int find_dev_i3c_idx(struct i3c_dev_desc *desc)
{
	int bus_id = get_bus_id(desc);
	if (bus_id < 0) {
		return -1;
	}

	int i = 0;
	struct i3c_dev_desc *desc_ptr = NULL;
	for (i = 0; i < i3c_desc_count; i++) {
		desc_ptr = &i3c_desc_table[i];

		int table_bus_id = get_bus_id(desc_ptr);
		if (table_bus_id < 0) {
			return -1;
		}

		if ((table_bus_id == bus_id) &&
		    (desc->info.dynamic_addr == desc_ptr->info.dynamic_addr)) {
			return i;
		}
	}

	LOG_ERR("Not found id in i3c table");

	return -1;
}

static struct i3c_ibi_payload *ibi_write_requested(struct i3c_dev_desc *desc)
{
	int idx = find_dev_i3c_idx(desc);
	if (idx < 0) {
		LOG_ERR("%s: find dev i3c idx failed. idx = %d", __func__, idx);
	}
	i3c_ibi_dev_table[idx].i3c_payload.max_payload_size = I3C_MAX_DATA_SIZE;
	i3c_ibi_dev_table[idx].i3c_payload.size = 0;
	i3c_ibi_dev_table[idx].i3c_payload.buf = i3c_ibi_dev_table[idx].data_rx;

	return &i3c_ibi_dev_table[idx].i3c_payload;
}

static void ibi_write_done(struct i3c_dev_desc *desc)
{
	int idx = find_dev_i3c_idx(desc);
	if (idx < 0) {
		LOG_ERR("%s: find dev i3c idx failed. idx = %d", __func__, idx);
	}
	k_sem_give(&i3c_ibi_dev_table[idx].ibi_complete);
}

static struct i3c_ibi_callbacks i3c_ibi_def_callbacks = {
	.write_requested = ibi_write_requested,
	.write_done = ibi_write_done,
};

int i3c_controller_ibi_init(I3C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -EINVAL);

	if (!dev_i3c[msg->bus]) {
		LOG_ERR("Failed to receive messages to address 0x%x due to undefined bus%u",
			msg->target_addr, msg->bus);
		return -ENODEV;
	}

	struct i3c_dev_desc *target;
	target = find_matching_desc(dev_i3c[msg->bus], msg->target_addr, NULL);
	if (target == NULL) {
		LOG_ERR("Failed to reveive messages to address 0x%x due to unknown address",
			msg->target_addr);
		return -ENODEV;
	}

	int ret = 0;

	int idx = find_dev_i3c_idx(target);
	if (idx < 0) {
		LOG_ERR("%s: find dev i3c idx failed. idx = %d", __func__, idx);
	}

	k_sem_init(&i3c_ibi_dev_table[idx].ibi_complete, 0, 1);

	ret = i3c_master_send_rstdaa(dev_i3c[msg->bus]);
	if (ret) {
		LOG_ERR("Failed to send rstdaa");
	}

	ret = i3c_master_send_rstdaa(dev_i3c[msg->bus]);
	if (ret) {
		LOG_ERR("Failed to 2nd send rstdaa");
	}

	ret = i3c_master_send_aasa(dev_i3c[msg->bus]);
	if (ret) {
		LOG_ERR("Failed to send aasa");
	}

	ret = i3c_master_send_getpid(dev_i3c[msg->bus], target->info.dynamic_addr,
				     &target->info.pid);
	if (ret) {
		LOG_ERR("Failed to get pid");
	}

	ret = i3c_master_send_getbcr(dev_i3c[msg->bus], target->info.dynamic_addr,
				     &target->info.bcr);
	if (ret) {
		LOG_ERR("Failed to get bcr");
	}

	ret = i3c_master_request_ibi(target, &i3c_ibi_def_callbacks);
	if (ret != 0) {
		LOG_ERR("Failed to request SIR, bus = %x, addr = %u", msg->target_addr, msg->bus);
	}

	ret = i3c_master_enable_ibi(target);
	if (ret != 0) {
		LOG_ERR("Failed to enable SIR, bus = %x, addr = %u", msg->target_addr, msg->bus);
	}

	return -ret;
}

int i3c_controller_ibi_read(I3C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -EINVAL);

	if (!dev_i3c[msg->bus]) {
		LOG_ERR("Failed to receive messages to address 0x%x due to undefined bus%u",
			msg->target_addr, msg->bus);
		return -ENODEV;
	}

	struct i3c_dev_desc *target;
	target = find_matching_desc(dev_i3c[msg->bus], msg->target_addr, NULL);
	if (target == NULL) {
		LOG_ERR("Failed to reveive messages to address 0x%x due to unknown address",
			msg->target_addr);
		return -ENODEV;
	}

	int idx = find_dev_i3c_idx(target);
	if (idx < 0) {
		LOG_ERR("%s: find dev i3c idx failed. idx = %d", __func__, idx);
	}

	/* master device waits for the IBI from the target */
	k_sem_take(&i3c_ibi_dev_table[idx].ibi_complete, K_FOREVER);

	/* init the flag for the next loop */
	k_sem_init(&i3c_ibi_dev_table[idx].ibi_complete, 0, 1);

	int ret = 0;

	/* check result: first byte (MDB) shall match the DT property mandatory-data-byte */
	if (IS_MDB_PENDING_READ_NOTIFY(i3c_ibi_dev_table[idx].data_rx[0])) {
		struct i3c_priv_xfer xfer;

		/* initiate a private read transfer to read the pending data */
		xfer.rnw = 1;
		xfer.len = IBI_PAYLOAD_SIZE;
		xfer.data.in = i3c_ibi_dev_table[idx].data_rx;
		k_yield();
		ret = i3c_master_priv_xfer(target, &xfer, 1);
		if (ret) {
			LOG_ERR("ibi read failed. ret = %d", ret);
		}
		msg->rx_len = xfer.len;
		memcpy(msg->data, i3c_ibi_dev_table[idx].data_rx, IBI_PAYLOAD_SIZE);
	}

	return msg->rx_len;
}

int i3c_target_set_address(I3C_MSG *msg)
{
	CHECK_NULL_ARG_WITH_RETURN(msg, -EINVAL);

	if (!dev_i3c[msg->bus]) {
		return -ENODEV;
	}

	int ret = i3c_slave_set_static_addr(dev_i3c[msg->bus], msg->target_addr);
	if (ret != 0) {
		LOG_ERR("Failed to set address 0x%x, ret: %d", msg->target_addr, ret);
		return -1;
	}

	return 0;
}
