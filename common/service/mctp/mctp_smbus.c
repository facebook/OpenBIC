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
#include "hal_i2c_target.h"
#include <logging/log.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/crc.h>
#include <sys/printk.h>
#include <zephyr.h>
#include "libutil.h"

LOG_MODULE_DECLARE(mctp, LOG_LEVEL_DBG);

#define MCTP_SMBUS_PEC_SIZE 1

#define MCTP_SMBUS_CMD_CODE 0x0F

typedef struct __attribute__((packed)) {
	uint8_t cmd_code;
	uint8_t byte_count;
	uint8_t src_addr;
} smbus_hdr;

static uint8_t cal_pec(uint8_t dest_addr, uint8_t *buf, uint32_t len, uint8_t *pec)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);
	CHECK_ARG_WITH_RETURN(!len, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(pec, MCTP_ERROR);

	uint8_t pec_buf[len];
	pec_buf[0] = dest_addr;
	memcpy(pec_buf + 1, buf, len - 1);

	LOG_HEXDUMP_DBG(pec_buf, sizeof(pec_buf), "cal_pec");

	*pec = crc8(pec_buf, sizeof(pec_buf), 0x07, 0x00, false);
	return MCTP_SUCCESS;
}

static bool is_pec_vaild(uint8_t dest_addr, uint8_t *buf, uint32_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(buf, false);
	CHECK_ARG_WITH_RETURN(!len, false);

	uint8_t pec = 0;
	if (cal_pec(dest_addr, buf, len, &pec) == MCTP_ERROR)
		return false;

	uint8_t exp_pec = buf[len - 1];

	if (pec != exp_pec) {
		LOG_WRN("pec error dest_addr %x, cal = %x, exp = %x", dest_addr, pec, exp_pec);
		LOG_HEXDUMP_WRN(buf, sizeof(len), "is_pec_vaild");
	}

	return (pec == exp_pec) ? true : false;
}

static uint8_t make_send_buf(mctp *mctp_inst, uint8_t *send_buf, uint32_t send_len,
			     uint8_t *mctp_data, uint32_t mctp_data_len, mctp_ext_params extra_data)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(send_buf, MCTP_ERROR);
	CHECK_ARG_WITH_RETURN(!send_len, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(mctp_data, MCTP_ERROR);
	CHECK_ARG_WITH_RETURN(!mctp_data_len, MCTP_ERROR);

	smbus_hdr *hdr = (smbus_hdr *)send_buf;
	hdr->cmd_code = MCTP_SMBUS_CMD_CODE;
	/* bit 0 always set */
	hdr->src_addr = mctp_inst->medium_conf.smbus_conf.addr + 1;
	/* extend 1 byte src addr */
	hdr->byte_count = 1 + mctp_data_len;
	memcpy(send_buf + sizeof(*hdr), mctp_data, mctp_data_len);

	uint8_t pec = 0;
	if (cal_pec(extra_data.smbus_ext_params.addr, send_buf, send_len, &pec) == MCTP_ERROR)
		return MCTP_ERROR;

	send_buf[send_len - 1] = pec;
	return MCTP_SUCCESS;
}

static void mctp_smbus_read_hook(uint8_t *data, uint16_t *len)
{
	CHECK_NULL_ARG(data);
	CHECK_NULL_ARG(len);

	smbus_hdr *hdr = (smbus_hdr *)data;

	if (hdr->byte_count != *len - sizeof(smbus_hdr)) {
		LOG_HEXDUMP_DBG(data, *len, "Data before move:");
		if (hdr->byte_count == 0x0F) {
			LOG_DBG("Data length (%d) move 1 byte.....", *len);
			*len -= 1;
			memcpy(data, data + 1, *len);
		} else {
			LOG_DBG("Data length (%d) move 2 bytes.....", *len);
			*len -= 2;
			memcpy(data, data + 2, *len);
		}
		LOG_HEXDUMP_DBG(data, *len, "Data after move:");
	}
	return;
}

static uint16_t mctp_smbus_read(void *mctp_p, uint8_t *buf, uint32_t len,
				mctp_ext_params *extra_data)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, 0);
	CHECK_NULL_ARG_WITH_RETURN(buf, 0);
	CHECK_ARG_WITH_RETURN(!len, 0);
	CHECK_NULL_ARG_WITH_RETURN(extra_data, 0);

	mctp *mctp_inst = (mctp *)mctp_p;

	uint8_t rdata[256] = { 0 };
	uint16_t rlen = 0;

	uint8_t ret = 0;
	ret = i2c_target_read(mctp_inst->medium_conf.smbus_conf.bus, rdata, 256, &rlen);
	if (ret) {
		LOG_ERR("i2c_target_read fail, ret %d", ret);
		return 0;
	}

	/**
   * Since the i2c driver provided by ASPEED may read redundant duplicates data,
   * use this temporary function to workaround and wait for the ASPEED to solve
   * this issue then remove it.
   */
	mctp_smbus_read_hook(rdata, &rlen);

	smbus_hdr *hdr = (smbus_hdr *)rdata;

	/**
   * Does read data include pec?
   * rlen = 1(mctp cmd code 0x0F) + 1(byte count) + N(byte count) + 1(*pec, if exist)
   * so if rlen is equal N + 3, the pec is existing.
	 */
	const uint8_t is_pec_exist = ((hdr->byte_count + 3) == rlen) ? 1 : 0;

	if (is_pec_exist) {
		if (is_pec_vaild(mctp_inst->medium_conf.smbus_conf.addr, rdata, rlen) == false)
			return 0;
	}

	if (hdr->cmd_code != MCTP_SMBUS_CMD_CODE)
		return 0;

	extra_data->type = MCTP_MEDIUM_TYPE_SMBUS;
	extra_data->smbus_ext_params.addr = hdr->src_addr - 1;

	uint8_t rt_size = rlen - sizeof(smbus_hdr) - is_pec_exist;
	memcpy(buf, rdata + sizeof(smbus_hdr), rt_size);

	return rt_size;
}

static uint16_t mctp_smbus_write(void *mctp_p, uint8_t *buf, uint32_t len,
				 mctp_ext_params extra_data)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_p, MCTP_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(buf, MCTP_ERROR);
	CHECK_ARG_WITH_RETURN(!len, MCTP_ERROR);

	mctp *mctp_inst = (mctp *)mctp_p;

	if (extra_data.type != MCTP_MEDIUM_TYPE_SMBUS)
		return MCTP_ERROR;

	LOG_HEXDUMP_DBG(buf, len, "mctp_smbus_write receive data");

	/* send len = 1(mctp cmd code 0x0F) + 1(byte count) + 1(src addr) +
            len(mctp data) + 1(*pec, if exist) */
	uint32_t send_len = len + 4;
	uint8_t send_buf[send_len];
	uint8_t rc = make_send_buf(mctp_inst, send_buf, send_len, buf, len, extra_data);
	if (rc == MCTP_ERROR) {
		LOG_WRN("make send buf failed!!");
		return MCTP_ERROR;
	}

	LOG_DBG("smbus_ext_params addr = %x", extra_data.smbus_ext_params.addr);
	LOG_HEXDUMP_DBG(send_buf, send_len, "mctp_smbus_write make header");

	int status;
	I2C_MSG i2c_msg;
	i2c_msg.bus = mctp_inst->medium_conf.smbus_conf.bus;
	i2c_msg.target_addr = extra_data.smbus_ext_params.addr >> 1;
	i2c_msg.tx_len = send_len;
	memcpy(&i2c_msg.data[0], send_buf, send_len);
	status = i2c_master_write(&i2c_msg, 5);
	if (status) {
		LOG_ERR("i2c_master_write failed, ret %d", status);
		return MCTP_ERROR;
	}

	return MCTP_SUCCESS;
}

uint8_t mctp_smbus_init(mctp *mctp_inst, mctp_medium_conf medium_conf)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, MCTP_ERROR);

	mctp_inst->medium_conf = medium_conf;
	mctp_inst->read_data = mctp_smbus_read;
	mctp_inst->write_data = mctp_smbus_write;

	return MCTP_SUCCESS;
}

uint8_t mctp_smbus_deinit(mctp *mctp_inst)
{
	CHECK_NULL_ARG_WITH_RETURN(mctp_inst, MCTP_ERROR);

	mctp_inst->read_data = NULL;
	mctp_inst->write_data = NULL;
	memset(&mctp_inst->medium_conf, 0, sizeof(mctp_inst->medium_conf));
	return MCTP_SUCCESS;
}
