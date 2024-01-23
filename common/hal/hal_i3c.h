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

#ifndef HAL_I3C_H
#define HAL_I3C_H

#include <zephyr.h>
#include <drivers/i3c/i3c.h>

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c0), okay)
#define DEV_I3C_0
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c1), okay)
#define DEV_I3C_1
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c2), okay)
#define DEV_I3C_2
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c3), okay)
#define DEV_I3C_3
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c4), okay)
#define DEV_I3C_4
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c5), okay)
#define DEV_I3C_5
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c6), okay)
#define DEV_I3C_6
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c7), okay)
#define DEV_I3C_7
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(i3c0_smq))
#define DEV_I3CSMQ_0
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(i3c1_smq))
#define DEV_I3CSMQ_1
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(i3c2_smq))
#define DEV_I3CSMQ_2
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(i3c3_smq))
#define DEV_I3CSMQ_3
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(i3c4_smq))
#define DEV_I3CSMQ_4
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(i3c5_smq))
#define DEV_I3CSMQ_5
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(i3c6_smq))
#define DEV_I3CSMQ_6
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(i3c7_smq))
#define DEV_I3CSMQ_7
#endif

#define DEV_I3C(n) DEV_I3C_##n
#define DEV_I3CSMQ(n) DEV_I3CSMQ_##n

#define I3C_MAX_NUM 8
#define I3C_MAX_DATA_SIZE 256
#define I3C_DEBUG 0
#define I3C_SMQ_SUCCESS 0
#define I3C_MAX_BUFFER_COUNT 2
#define IBI_PAYLOAD_SIZE 128

#define I3C_DEV_STR_LEN 8
#define I3C_BUS_STR_LEN 4

enum I3C_WRITE_READ_CMD {
	I3C_WRITE_CMD = 0,
	I3C_READ_CMD,
};

typedef struct _I3C_MSG_ {
	uint8_t bus;
	uint8_t target_addr;
	uint8_t tx_len;
	uint8_t rx_len;
	uint8_t data[I3C_MAX_DATA_SIZE];
} I3C_MSG;

typedef struct _i3c_ibi_dev {
	uint8_t data_rx[I3C_MAX_DATA_SIZE];
	struct i3c_ibi_payload i3c_payload;
	struct k_sem ibi_complete;
} i3c_ibi_dev;

void util_init_i3c(void);
int i3c_smq_read(I3C_MSG *msg);
int i3c_smq_write(I3C_MSG *msg);
int i3c_slave_mqueue_read(const struct device *dev, uint8_t *dest, int budget);
int i3c_slave_mqueue_write(const struct device *dev, uint8_t *src, int size);

int i3c_attach(I3C_MSG *msg);
int i3c_detach(I3C_MSG *msg);
int i3c_transfer(I3C_MSG *msg);
int i3c_brocast_ccc(I3C_MSG *msg, uint8_t ccc_id, uint8_t ccc_addr);
int i3c_spd_reg_read(I3C_MSG *msg, bool is_nvm);
int i3c_set_pid(I3C_MSG *msg, uint16_t slot_pid);

int i3c_controller_write(I3C_MSG *msg);
int i3c_controller_ibi_init(I3C_MSG *msg);
int i3c_controller_ibi_read(I3C_MSG *msg);
int i3c_controller_write(I3C_MSG *msg);
int i3c_target_set_address(I3C_MSG *msg);

#endif
