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
 *
 * Board-local PLDM for Platform Monitoring and Control (DSP0248,
 * PLDM type 0x02) responder for this EVK.
 *
 * Why board-local instead of common/service/pldm/pldm_monitor.c: that
 * file is ~1500 lines wired to common/hal/hal_gpio.c (the hand-rolled
 * Aspeed/NPCM4XX register-level GPIO driver, unportable here - see
 * README.md) and the AST1030 167-pin GPIO-effecter model. None of that
 * applies to an EVK whose only monitorable hardware is the SoC die
 * temperature sensor, the SW2 button, and an on-board LED. This file
 * stands up a small, self-contained real responder for exactly those,
 * following the same pattern as plat_mctp.c / plat_ipmb.c: it provides
 * the pldm_monitor_handler_query() that pldm.c's query_tbl needs
 * (replacing the plat_stubs.c "not implemented" stub), which is also
 * what makes PLDM type 0x02 automatically appear in GetPLDMTypes /
 * GetPLDMCommands.
 *
 * Implemented so far (phase 1): GetPDRInfo (0x50), GetPDR (0x51),
 * GetSensorReading (0x11) - backed by one real numeric sensor, the
 * MCXN947 on-die temperature (LPADC TEMP40). State sensor (SW2) and
 * state effecter (LED) are added in later phases.
 */

#include <stddef.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "pldm.h"
#include "pldm_monitor.h"
#include "pdr.h"
#include "plat_pldm_monitor.h"

LOG_MODULE_REGISTER(plat_pldm_monitor);

/* ---- sensor / PDR identifiers --------------------------------------- */

#define PLAT_PLDM_TERMINUS_HANDLE 0x0000

/* one numeric sensor: the SoC die temperature */
#define SENSOR_ID_DIE_TEMP 0x0001
#define PDR_HANDLE_DIE_TEMP 0x00000001

/* DSP0248 Table 79 base units: 2 = degrees C */
#define PLDM_SENSOR_UNIT_DEGRESS_C 2
/* reading is reported in milli-degC, so applied value = raw * 10^-3 */
#define DIE_TEMP_UNIT_MODIFIER (-3)

/* DSP0248 numeric sensor data size / GetSensorReading sensor_data_size:
 * 0=uint8 1=sint8 2=uint16 3=sint16 4=uint32 5=sint32
 */
#define PLDM_SENSOR_DATA_SIZE_SINT32_ENC 5

/* ---- die-temp backing --------------------------------------------------
 * Optional: only wired if the devicetree provides a die_temp sensor
 * node (nxp,lpadc-temp40 in the board overlay). Without it the sensor
 * still exists in the PDR repository but GetSensorReading honestly
 * reports PLDM_SENSOR_UNAVAILABLE rather than fabricating a value.
 */
#if DT_NODE_EXISTS(DT_NODELABEL(die_temp))
#define HAVE_DIE_TEMP 1
static const struct device *const die_temp_dev = DEVICE_DT_GET(DT_NODELABEL(die_temp));
#else
#define HAVE_DIE_TEMP 0
static const struct device *const die_temp_dev = NULL;
#endif

static bool die_temp_primed;

/* Fetch the SoC die temperature in milli-degrees C.
 * Returns 0 on success, negative errno on failure.
 */
static int plat_read_die_temp_mdegc(int32_t *out_mdegc)
{
	if (!HAVE_DIE_TEMP || !device_is_ready(die_temp_dev)) {
		return -ENODEV;
	}

	/* The MCXN947 LPADC temp path (FSL_FEATURE_LPADC_TEMP_SENS_BUFFER_SIZE
	 * == 2, so lpadc_temp40's "drop first two samples" branch is
	 * compiled out) returns garbage on the very first conversion after
	 * init - throw one away once.
	 */
	if (!die_temp_primed) {
		(void)sensor_sample_fetch_chan(die_temp_dev, SENSOR_CHAN_DIE_TEMP);
		die_temp_primed = true;
	}

	int ret = sensor_sample_fetch_chan(die_temp_dev, SENSOR_CHAN_DIE_TEMP);

	if (ret) {
		return ret;
	}

	struct sensor_value val;

	ret = sensor_channel_get(die_temp_dev, SENSOR_CHAN_DIE_TEMP, &val);
	if (ret) {
		return ret;
	}

	*out_mdegc = val.val1 * 1000 + val.val2 / 1000;
	return 0;
}

/* ---- PDR repository --------------------------------------------------
 * Small static repository. Phase 1: a single PLDM Numeric Sensor PDR
 * (DSP0248 28.4) for the die-temp sensor. Built once at init.
 */

static PDR_numeric_sensor die_temp_pdr;
static PDR_INFO plat_pdr_info;
static uint32_t plat_pdr_record_count;
static bool plat_pdr_ready;

static void build_die_temp_pdr(void)
{
	memset(&die_temp_pdr, 0, sizeof(die_temp_pdr));

	die_temp_pdr.pdr_common_header.record_handle = PDR_HANDLE_DIE_TEMP;
	die_temp_pdr.pdr_common_header.PDR_header_version = 1;
	die_temp_pdr.pdr_common_header.PDR_type = PLDM_NUMERIC_SENSOR_PDR;
	die_temp_pdr.pdr_common_header.record_change_number = 0;
	die_temp_pdr.pdr_common_header.data_length =
		sizeof(PDR_numeric_sensor) - sizeof(PDR_common_header);

	die_temp_pdr.PLDM_terminus_handle = PLAT_PLDM_TERMINUS_HANDLE;
	die_temp_pdr.sensor_id = SENSOR_ID_DIE_TEMP;
	die_temp_pdr.entity_type = PLDM_ENTITY_DEVICE_DRIVER;
	die_temp_pdr.entity_instance_number = 1;
	die_temp_pdr.container_id = 0;
	die_temp_pdr.sensor_init = PDR_SENSOR_NO_INIT;
	die_temp_pdr.sensor_auxiliary_names_pdr = 0;

	die_temp_pdr.base_unit = PLDM_SENSOR_UNIT_DEGRESS_C;
	die_temp_pdr.unit_modifier = DIE_TEMP_UNIT_MODIFIER;
	die_temp_pdr.rate_unit = 0;
	die_temp_pdr.aux_unit = 0;
	die_temp_pdr.rel = 0;

	die_temp_pdr.is_linear = 1;
	die_temp_pdr.sensor_data_size = PLDM_SENSOR_DATA_SIZE_SINT32_ENC;
	/* applied = (resolution * raw + offset) * 10^unit_modifier */
	die_temp_pdr.resolution = 1.0f;
	die_temp_pdr.offset = 0.0f;
	die_temp_pdr.accuracy = 0;
	die_temp_pdr.hysteresis = 0;
	die_temp_pdr.supported_thresholds = 0;
	die_temp_pdr.update_interval = 1.0f;
	die_temp_pdr.range_field_format = 0; /* uint8 - unused, no thresholds */
	die_temp_pdr.range_field_support = 0;
}

static void plat_pdr_init(void)
{
	build_die_temp_pdr();
	plat_pdr_record_count = 1;

	memset(&plat_pdr_info, 0, sizeof(plat_pdr_info));
	plat_pdr_info.repository_state = PDR_STATE_AVAILABLE;
	plat_pdr_info.record_count = plat_pdr_record_count;
	plat_pdr_info.repository_size = sizeof(die_temp_pdr);
	plat_pdr_info.largest_record_size = sizeof(die_temp_pdr);
	plat_pdr_info.data_transfer_handle_timeout = 0;

	plat_pdr_ready = true;
}

/* record_handle 0 means "first record" per DSP0248 GetPDR */
static const uint8_t *plat_pdr_get_record(uint32_t record_handle, uint16_t *rec_len,
					  uint32_t *next_handle)
{
	if (record_handle == 0 || record_handle == PDR_HANDLE_DIE_TEMP) {
		*rec_len = sizeof(die_temp_pdr);
		*next_handle = 0; /* only record */
		return (const uint8_t *)&die_temp_pdr;
	}

	return NULL;
}

/* ---- command handlers --------------------------------------------------
 * Signature: pldm_cmd_proc_fn
 *   (void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t iid,
 *    uint8_t *resp, uint16_t *resp_len, void *ext_params)
 * buf/len are the payload after the pldm_hdr; resp is filled after the
 * (already-populated) response header; *resp_len starts at 1.
 */

static uint8_t plat_get_pdr_info(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t iid,
				 uint8_t *resp, uint16_t *resp_len, void *ext_params)
{
	ARG_UNUSED(mctp_inst);
	ARG_UNUSED(buf);
	ARG_UNUSED(iid);
	ARG_UNUSED(ext_params);

	struct pldm_get_pdr_info_resp *r = (struct pldm_get_pdr_info_resp *)resp;

	if (len != 0) {
		resp[0] = PLDM_ERROR_INVALID_LENGTH;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}
	if (!plat_pdr_ready) {
		resp[0] = PLDM_ERROR;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	memset(r, 0, sizeof(*r));
	r->completion_code = PLDM_SUCCESS;
	r->repository_state = plat_pdr_info.repository_state;
	r->record_count = plat_pdr_info.record_count;
	r->repository_size = plat_pdr_info.repository_size;
	r->largest_record_size = plat_pdr_info.largest_record_size;
	r->data_transfer_handle_timeout = 0;
	*resp_len = sizeof(*r);
	return PLDM_SUCCESS;
}

static uint8_t plat_get_pdr(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t iid, uint8_t *resp,
			    uint16_t *resp_len, void *ext_params)
{
	ARG_UNUSED(mctp_inst);
	ARG_UNUSED(iid);
	ARG_UNUSED(ext_params);

	struct pldm_get_pdr_req *req = (struct pldm_get_pdr_req *)buf;
	struct pldm_get_pdr_resp *r = (struct pldm_get_pdr_resp *)resp;

	if (len != sizeof(*req)) {
		resp[0] = PLDM_ERROR_INVALID_LENGTH;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}
	if (!plat_pdr_ready) {
		resp[0] = PLDM_ERROR;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	uint16_t rec_len = 0;
	uint32_t next_handle = 0;
	const uint8_t *rec = plat_pdr_get_record(req->record_handle, &rec_len, &next_handle);

	if (!rec) {
		/* DSP0248: INVALID_RECORD_HANDLE = 0x82 */
		resp[0] = 0x82;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	if (rec_len > sizeof(r->record_data)) {
		resp[0] = PLDM_ERROR;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	memset(r, 0, sizeof(*r));
	r->completion_code = PLDM_SUCCESS;
	r->next_record_handle = next_handle;
	r->next_data_transfer_handle = 0;
	r->transfer_flag = PLDM_TRANSFER_FLAG_START_AND_END;
	r->response_count = rec_len;
	memcpy(r->record_data, rec, rec_len);

	*resp_len = offsetof(struct pldm_get_pdr_resp, record_data) + rec_len;
	return PLDM_SUCCESS;
}

static uint8_t plat_get_sensor_reading(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t iid,
				       uint8_t *resp, uint16_t *resp_len, void *ext_params)
{
	ARG_UNUSED(mctp_inst);
	ARG_UNUSED(iid);
	ARG_UNUSED(ext_params);

	struct pldm_get_sensor_reading_req *req = (struct pldm_get_sensor_reading_req *)buf;
	struct pldm_get_sensor_reading_resp *r = (struct pldm_get_sensor_reading_resp *)resp;

	if (len != sizeof(*req)) {
		resp[0] = PLDM_ERROR_INVALID_LENGTH;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}
	if (req->sensor_id != SENSOR_ID_DIE_TEMP) {
		/* DSP0248: INVALID_SENSOR_ID = 0x80 */
		resp[0] = 0x80;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	int32_t mdegc = 0;
	int ret = plat_read_die_temp_mdegc(&mdegc);

	memset(r, 0, sizeof(*r) + sizeof(int32_t));
	r->completion_code = PLDM_SUCCESS;
	r->sensor_data_size = PLDM_SENSOR_DATA_SIZE_SINT32_ENC;
	r->sensor_event_message_enable = PLDM_NO_EVENT_GENERATION;
	r->present_state = PLDM_SENSOR_NORMAL;
	r->previous_state = PLDM_SENSOR_NORMAL;
	r->event_state = PLDM_SENSOR_NORMAL;

	if (ret) {
		r->sensor_operational_state = PLDM_SENSOR_UNAVAILABLE;
		mdegc = 0;
		LOG_WRN("die-temp read failed (%d), reporting UNAVAILABLE", ret);
	} else {
		r->sensor_operational_state = PLDM_SENSOR_ENABLED;
	}

	sys_put_le32((uint32_t)mdegc, r->present_reading);

	/* resp struct already has present_reading[1]; total = header fields
	 * + 4-byte sint32 reading
	 */
	*resp_len = offsetof(struct pldm_get_sensor_reading_resp, present_reading) + sizeof(int32_t);
	return PLDM_SUCCESS;
}

/* ---- dispatch ------------------------------------------------------- */

static pldm_cmd_handler plat_pldm_monitor_cmd_tbl[] = {
	{ PLDM_MONITOR_CMD_CODE_GET_SENSOR_READING, plat_get_sensor_reading },
	{ PLDM_MONITOR_CMD_CODE_GET_PDR_INFO, plat_get_pdr_info },
	{ PLDM_MONITOR_CMD_CODE_GET_PDR, plat_get_pdr },
};

uint8_t pldm_monitor_handler_query(uint8_t code, void **ret_fn)
{
	if (!ret_fn) {
		return PLDM_ERROR;
	}

	for (uint8_t i = 0; i < ARRAY_SIZE(plat_pldm_monitor_cmd_tbl); i++) {
		if (plat_pldm_monitor_cmd_tbl[i].cmd_code == code) {
			*ret_fn = (void *)plat_pldm_monitor_cmd_tbl[i].fn;
			return PLDM_SUCCESS;
		}
	}

	*ret_fn = NULL;
	return PLDM_ERROR;
}

/* ---- public: local (non-MCTP) helpers for the shell test command ---- */

int plat_pldm_monitor_read_die_temp_mdegc(int32_t *out_mdegc)
{
	return plat_read_die_temp_mdegc(out_mdegc);
}

uint32_t plat_pldm_monitor_pdr_count(void)
{
	return plat_pdr_ready ? plat_pdr_record_count : 0;
}

void plat_pldm_monitor_init(void)
{
	plat_pdr_init();

	if (HAVE_DIE_TEMP && device_is_ready(die_temp_dev)) {
		int32_t t = 0;

		if (plat_read_die_temp_mdegc(&t) == 0) {
			LOG_INF("PLDM Type 2 up: %u PDR(s); die-temp sensor id 0x%x = %d.%03d C",
				plat_pdr_record_count, SENSOR_ID_DIE_TEMP, t / 1000,
				(t < 0 ? -t : t) % 1000);
			return;
		}
	}

	LOG_INF("PLDM Type 2 up: %u PDR(s); die-temp sensor present but not readable yet",
		plat_pdr_record_count);
}
