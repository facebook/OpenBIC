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

#include "mctp.h"

#include <stdlib.h>
#include <string.h>
#include <zephyr.h>
#include <sys/crc.h>
#include <logging/log.h>
#include "libutil.h"
#include "hal_i3c.h"

LOG_MODULE_REGISTER(mctp_i3c);

#ifndef MCTP_I3C_PEC_ENABLE
#define MCTP_I3C_PEC_ENABLE 0
#endif

static uint16_t mctp_i3c_read_smq(void *mctp_p, uint8_t *buf, uint32_t len,
				  mctp_ext_params *extra_data)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);

	/** blind return length 0 for now, the function will implement in rx commit **/
	LOG_INF("mctp_i3c_read_smq is not ready, return 0(length) blindly");
	return 0;
}

static uint16_t mctp_i3c_write_smq(void *mctp_p, uint8_t *buf, uint32_t len,
				   mctp_ext_params extra_data)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);

	if (extra_data.type != MCTP_MEDIUM_TYPE_I3C) {
		LOG_ERR("mctp medium type incorrect");
		return MCTP_ERROR;
	}

	int ret;
	I3C_MSG *i3c_msg = (I3C_MSG *)malloc(sizeof(I3C_MSG));
	if (i3c_msg == NULL) {
		LOG_ERR("Memory allocation failed.");
		return MCTP_ERROR;
	}

	mctp *mctp_instance = (mctp *)mctp_p;
	i3c_msg->bus = mctp_instance->medium_conf.i3c_conf.bus;
	/** mctp package **/
	memcpy(&i3c_msg->data[0], buf, len);
	/** +1 pec; default no pec **/
	if (MCTP_I3C_PEC_ENABLE) {
		i3c_msg->tx_len = len + 1;
		/** pec byte use 7-degree polynomial with 0 init value and false reverse **/
		i3c_msg->data[len + 1] = crc8(&i3c_msg->data[0], len, 0x07, 0x00, false);
	} else {
		i3c_msg->tx_len = len;
	}

	LOG_HEXDUMP_DBG(&i3c_msg->data[0], i3c_msg->tx_len, "mctp_i3c_write_smq msg dump");

	ret = i3c_smq_write(i3c_msg);
	SAFE_FREE(i3c_msg);

	if (ret < 0) {
		LOG_ERR("mctp_i3c_write_smq write failed");
		return MCTP_ERROR;
	}
	return MCTP_SUCCESS;
}

uint8_t mctp_i3c_init(mctp *mctp_instance, mctp_medium_conf medium_conf)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_instance, MCTP_ERROR);

	mctp_instance->medium_conf = medium_conf;
	mctp_instance->read_data = mctp_i3c_read_smq;
	mctp_instance->write_data = mctp_i3c_write_smq;

	return MCTP_SUCCESS;
}

uint8_t mctp_i3c_deinit(mctp *mctp_instance)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_instance, MCTP_ERROR);

	mctp_instance->read_data = NULL;
	mctp_instance->write_data = NULL;
	memset(&mctp_instance->medium_conf, 0, sizeof(mctp_instance->medium_conf));
	return MCTP_SUCCESS;
}
