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

#include "plat_def.h"
#ifdef ENABLE_MCTP_I3C
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

static uint16_t mctp_i3c_read(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params *extra_data)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(extra_data, MCTP_ERROR);

	mctp *mctp_inst = (mctp *)mctp_p;
	I3C_MSG i3c_msg = { 0 };

	i3c_msg.bus = mctp_inst->medium_conf.i3c_conf.bus;
	i3c_msg.target_addr = mctp_inst->medium_conf.i3c_conf.addr;

	int ret = i3c_controller_ibi_read(&i3c_msg);

	/** mctp rx keep polling, return length 0 directly if no data or invalid data **/
	if (ret <= 0) {
		return 0;
	}

	i3c_msg.rx_len = ret;
	LOG_HEXDUMP_DBG(&i3c_msg.data[0], i3c_msg.rx_len, "mctp_i3c_read_smq msg dump");

	if (MCTP_I3C_PEC_ENABLE) {
		/** pec byte use 7-degree polynomial with 0 init value and false reverse **/
		uint8_t pec = crc8(&i3c_msg.data[0], i3c_msg.rx_len - 1, 0x07, 0x00, false);
		if (pec != i3c_msg.data[i3c_msg.rx_len - 1]) {
			LOG_ERR("mctp i3c pec error: crc8 should be 0x%02x, but got 0x%02x", pec,
				i3c_msg.data[i3c_msg.rx_len - 1]);
			return 0;
		}
	}

	extra_data->type = MCTP_MEDIUM_TYPE_CONTROLLER_I3C;
	memcpy(buf, &i3c_msg.data[0], i3c_msg.rx_len);
	return i3c_msg.rx_len;
}

static uint16_t mctp_i3c_write(void *mctp_p, uint8_t *buf, uint32_t len, mctp_ext_params extra_data)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);

	if (extra_data.type != MCTP_MEDIUM_TYPE_CONTROLLER_I3C) {
		LOG_ERR("mctp medium type incorrect");
		return MCTP_ERROR;
	}

	int ret;
	I3C_MSG i3c_msg;
	mctp *mctp_instance = (mctp *)mctp_p;

	i3c_msg.bus = mctp_instance->medium_conf.i3c_conf.bus;
	i3c_msg.target_addr = mctp_instance->medium_conf.i3c_conf.addr;

	/** mctp package **/
	memcpy(&i3c_msg.data[0], buf, len);

	/** +1 pec; default no pec **/
	if (MCTP_I3C_PEC_ENABLE) {
		i3c_msg.tx_len = len + 1;
		/** pec byte use 7-degree polynomial with 0 init value and false reverse **/
		i3c_msg.data[len + 1] = crc8(&i3c_msg.data[0], len, 0x07, 0x00, false);
	} else {
		i3c_msg.tx_len = len;
	}

	LOG_HEXDUMP_DBG(&i3c_msg.data[0], i3c_msg.tx_len, "mctp_i3c_write msg dump");

	ret = i3c_controller_write(&i3c_msg);
	if (ret < 0) {
		LOG_ERR("i3c controller write failed, %d", ret);
		return MCTP_ERROR;
	}
	return MCTP_SUCCESS;
}

static uint16_t mctp_i3c_read_smq(void *mctp_p, uint8_t *buf, uint32_t len,
				  mctp_ext_params *extra_data)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(extra_data, MCTP_ERROR);

	int ret = 0;
	I3C_MSG i3c_msg;
	mctp *mctp_inst = (mctp *)mctp_p;
	i3c_msg.bus = mctp_inst->medium_conf.i3c_conf.bus;
	ret = i3c_smq_read(&i3c_msg);

	/** mctp rx keep polling, return length 0 directly if no data or invalid data **/
	if (ret <= 0) {
		return 0;
	}

	i3c_msg.rx_len = ret;
	LOG_HEXDUMP_DBG(&i3c_msg.data[0], i3c_msg.rx_len, "mctp_i3c_read_smq msg dump");

	if (MCTP_I3C_PEC_ENABLE) {
		/** pec byte use 7-degree polynomial with 0 init value and false reverse **/
		uint8_t pec = crc8(&i3c_msg.data[0], i3c_msg.rx_len - 1, 0x07, 0x00, false);
		if (pec != i3c_msg.data[i3c_msg.rx_len - 1]) {
			LOG_ERR("mctp i3c pec error: crc8 should be 0x%02x, but got 0x%02x", pec,
				i3c_msg.data[i3c_msg.rx_len - 1]);
			return 0;
		}
	}

	extra_data->type = MCTP_MEDIUM_TYPE_TARGET_I3C;
	memcpy(buf, &i3c_msg.data[0], i3c_msg.rx_len);
	return i3c_msg.rx_len;
}

static uint16_t mctp_i3c_write_smq(void *mctp_p, uint8_t *buf, uint32_t len,
				   mctp_ext_params extra_data)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);

	if (extra_data.type != MCTP_MEDIUM_TYPE_TARGET_I3C) {
		LOG_ERR("mctp medium type incorrect");
		return MCTP_ERROR;
	}

	int ret;
	I3C_MSG i3c_msg;
	mctp *mctp_instance = (mctp *)mctp_p;
	i3c_msg.bus = mctp_instance->medium_conf.i3c_conf.bus;
	/** mctp package **/
	memcpy(&i3c_msg.data[0], buf, len);
	/** +1 pec; default no pec **/
	if (MCTP_I3C_PEC_ENABLE) {
		i3c_msg.tx_len = len + 1;
		/** pec byte use 7-degree polynomial with 0 init value and false reverse **/
		i3c_msg.data[len + 1] = crc8(&i3c_msg.data[0], len, 0x07, 0x00, false);
	} else {
		i3c_msg.tx_len = len;
	}

	LOG_HEXDUMP_DBG(&i3c_msg.data[0], i3c_msg.tx_len, "mctp_i3c_write_smq msg dump");

	ret = i3c_smq_write(&i3c_msg);
	if (ret < 0) {
		LOG_ERR("mctp_i3c_write_smq write failed, %d", ret);
		return MCTP_ERROR;
	}
	return MCTP_SUCCESS;
}

uint8_t mctp_i3c_controller_init(mctp *mctp_instance, mctp_medium_conf medium_conf)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_instance, MCTP_ERROR);

	mctp_instance->medium_conf = medium_conf;
	mctp_instance->read_data = mctp_i3c_read;
	mctp_instance->write_data = mctp_i3c_write;

	// i3c master initial
	LOG_INF("Bus= 0x%x, Addr = 0x%x", medium_conf.i3c_conf.bus, medium_conf.i3c_conf.addr);
	I3C_MSG i3c_msg = { 0 };
	i3c_msg.bus = medium_conf.i3c_conf.bus;
	i3c_msg.target_addr = medium_conf.i3c_conf.addr;

	i3c_attach(&i3c_msg);

	// i3c ibi mqueue initial
	i3c_controller_ibi_init(&i3c_msg);

	return MCTP_SUCCESS;
}

uint8_t mctp_i3c_target_init(mctp *mctp_instance, mctp_medium_conf medium_conf)
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

#endif // ENABLE_MCTP_I3C
