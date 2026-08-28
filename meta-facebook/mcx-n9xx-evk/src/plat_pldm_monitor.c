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
 * file is wired to common/hal/hal_gpio.c (the Aspeed/NPCM4XX
 * register-level GPIO driver, unportable here) and the AST1030 167-pin
 * GPIO-effecter model. This file stands up a small self-contained real
 * responder for exactly the hardware this EVK has, following the same
 * pattern as plat_mctp.c / plat_ipmb.c. It also provides the
 * pldm_monitor_handler_query() pldm.c's query_tbl needs, which is what
 * makes PLDM type 0x02 appear in GetPLDMTypes / GetPLDMCommands.
 *
 * PDR repository (walked via GetPDRInfo/GetPDR):
 *   handle 1  Terminus Locator PDR   -> this terminus, MCTP EID
 *   handle 2  Numeric Sensor PDR     -> sensor 0x0001, J2 analog input (mV)
 *   handle 3  State Sensor PDR       -> sensor 0x0002, SW2 button (Presence)
 *   handle 4  State Effecter PDR     -> effecter 0x0003, on-board LED
 *
 * Commands: GetSensorReading (0x11), GetStateSensorReadings (0x21),
 * SetStateEffecterStates (0x39), GetStateEffecterStates (0x3A),
 * GetPDRInfo (0x50), GetPDR (0x51).
 */

#include <stddef.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "mctp.h"
#include "pldm.h"
#include "pldm_base.h"
#include "pldm_monitor.h"
#include "pldm_state_set.h"
#include "pdr.h"
#include "plat_gpio.h"
#include "plat_mctp.h"
#include "plat_pldm_monitor.h"

uint8_t mctp_pldm_send_msg(void *mctp_p, pldm_msg *msg);

LOG_MODULE_REGISTER(plat_pldm_monitor);

/* ---- identifiers -------------------------------------------------------- */

#define PLAT_PLDM_TERMINUS_HANDLE 0x0000

#define SENSOR_ID_VMON 0x0001
#define SENSOR_ID_SW2 0x0002
#define EFFECTER_ID_LED 0x0003

#define PDR_HANDLE_TERMINUS_LOCATOR 1
#define PDR_HANDLE_VMON 2
#define PDR_HANDLE_SW2 3
#define PDR_HANDLE_LED 4
#define PDR_RECORD_COUNT 4

/* DSP0248 Table 79 base units: 5 = Volts */
#define PLDM_SENSOR_UNIT_VOLTS 5
#define VMON_UNIT_MODIFIER (-3) /* reading is millivolts */
/* DSP0248 sensor data size enum: 5 = sint32 */
#define PLDM_SENSOR_DATA_SIZE_SINT32_ENC 5

/* LED effecter uses the OEM device-status state set (DSP0249 32768);
 * for this board: state 1 (NORMAL) = LED off, state 2 (ALERT) = LED on. */
#define LED_STATE_OFF PLDM_STATE_SET_OEM_DEVICE_STATUS_NORMAL /* 1 */
#define LED_STATE_ON PLDM_STATE_SET_OEM_DEVICE_STATUS_ALERT /* 2 */

/* ---- hardware backing ------------------------------------------------- */

/* Numeric sensor 0x0001: a single-ended LPADC voltage read on an
 * analog pin routed to the J2 header, reported in millivolts. Nothing
 * is required to be wired there - it is a genuine ADC conversion of
 * whatever that pin sits at (a floating pin reads rail-referenced
 * noise), with honest resolution/scale so the PDR's m/b/r math checks
 * out. */
#if DT_NODE_EXISTS(DT_PATH(zephyr_user)) && DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#define HAVE_VMON 1
static const struct adc_dt_spec vmon_adc = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));
#else
#define HAVE_VMON 0
static const struct adc_dt_spec vmon_adc;
#endif

#if DT_NODE_HAS_STATUS(DT_ALIAS(led1), okay)
#define HAVE_LED 1
static const struct gpio_dt_spec led_effecter = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#else
#define HAVE_LED 0
static const struct gpio_dt_spec led_effecter = { 0 };
#endif
static uint8_t led_state = LED_STATE_OFF;

static bool vmon_ready;

static int plat_read_vmon_mv(int32_t *out_mv)
{
	if (!HAVE_VMON || !vmon_ready) {
		return -ENODEV;
	}

	int32_t sample = 0;
	struct adc_sequence seq = {
		.buffer = &sample,
		.buffer_size = sizeof(sample),
	};

	int ret = adc_sequence_init_dt(&vmon_adc, &seq);

	if (ret) {
		return ret;
	}

	ret = adc_read_dt(&vmon_adc, &seq);
	if (ret) {
		return ret;
	}

	ret = adc_raw_to_millivolts_dt(&vmon_adc, &sample);
	if (ret) {
		return ret;
	}

	*out_mv = sample;
	return 0;
}

/* SW2 present state, mapped onto the DSP0249 Presence state set. */
static uint8_t plat_sw2_present_state(void)
{
	return plat_gpio_mon0_get() ? PLDM_STATE_SET_PRESENT : PLDM_STATE_SET_NOT_PRESENT;
}

static int plat_led_apply(uint8_t state)
{
	if (!HAVE_LED || !gpio_is_ready_dt(&led_effecter)) {
		return -ENODEV;
	}
	if (state != LED_STATE_OFF && state != LED_STATE_ON) {
		return -EINVAL;
	}

	int ret = gpio_pin_set_dt(&led_effecter, state == LED_STATE_ON ? 1 : 0);

	if (ret == 0) {
		led_state = state;
	}
	return ret;
}

/* ---- PDR repository -------------------------------------------------- */

/* Hand-packed PDR blobs. Numeric sensor uses the pdr.h struct; the
 * state sensor / state effecter / terminus locator layouts aren't in
 * pdr.h so they're built byte-for-byte per DSP0248. */

static uint8_t terminus_locator_pdr[sizeof(PDR_common_header) + 16];
static PDR_numeric_sensor vmon_pdr;
static uint8_t state_sensor_pdr[sizeof(PDR_common_header) + 32];
static uint8_t state_effecter_pdr[sizeof(PDR_common_header) + 32];

static PDR_INFO plat_pdr_info;
static bool plat_pdr_ready;

struct pdr_record {
	uint32_t handle;
	const uint8_t *data;
	uint16_t len;
};

static struct pdr_record pdr_table[PDR_RECORD_COUNT];

/* little-endian append helpers */
static uint8_t *put8(uint8_t *p, uint8_t v)
{
	*p++ = v;
	return p;
}
static uint8_t *put16(uint8_t *p, uint16_t v)
{
	*p++ = v & 0xFF;
	*p++ = (v >> 8) & 0xFF;
	return p;
}

static void build_common_header(uint8_t *buf, uint32_t handle, uint8_t type, uint16_t body_len)
{
	PDR_common_header *h = (PDR_common_header *)buf;

	h->record_handle = handle;
	h->PDR_header_version = 1;
	h->PDR_type = type;
	h->record_change_number = 0;
	h->data_length = body_len;
}

static void build_terminus_locator_pdr(void)
{
	uint8_t *p = terminus_locator_pdr + sizeof(PDR_common_header);
	uint8_t *body = p;

	p = put16(p, PLAT_PLDM_TERMINUS_HANDLE);
	p = put8(p, 1); /* validity: valid */
	p = put8(p, plat_pldm_get_tid());
	p = put16(p, 0); /* container id */
	p = put8(p, 0); /* terminus locator type: MCTP EID */
	p = put8(p, 1); /* terminus locator value size */
	p = put8(p, PLAT_MCTP_EID); /* the EID */

	build_common_header(terminus_locator_pdr, PDR_HANDLE_TERMINUS_LOCATOR,
			    PLDM_TERMINUS_LOCATOR_PDR, (uint16_t)(p - body));
}

static void build_vmon_pdr(void)
{
	memset(&vmon_pdr, 0, sizeof(vmon_pdr));

	vmon_pdr.pdr_common_header.record_handle = PDR_HANDLE_VMON;
	vmon_pdr.pdr_common_header.PDR_header_version = 1;
	vmon_pdr.pdr_common_header.PDR_type = PLDM_NUMERIC_SENSOR_PDR;
	vmon_pdr.pdr_common_header.record_change_number = 0;
	vmon_pdr.pdr_common_header.data_length =
		sizeof(PDR_numeric_sensor) - sizeof(PDR_common_header);

	vmon_pdr.PLDM_terminus_handle = PLAT_PLDM_TERMINUS_HANDLE;
	vmon_pdr.sensor_id = SENSOR_ID_VMON;
	vmon_pdr.entity_type = PLDM_ENTITY_DEVICE_DRIVER;
	vmon_pdr.entity_instance_number = 1;
	vmon_pdr.container_id = 0;
	vmon_pdr.sensor_init = PDR_SENSOR_NO_INIT;
	vmon_pdr.base_unit = PLDM_SENSOR_UNIT_VOLTS;
	vmon_pdr.unit_modifier = VMON_UNIT_MODIFIER;
	vmon_pdr.is_linear = 1;
	vmon_pdr.sensor_data_size = PLDM_SENSOR_DATA_SIZE_SINT32_ENC;
	vmon_pdr.resolution = 1.0f;
	vmon_pdr.offset = 0.0f;
	vmon_pdr.update_interval = 1.0f;
}

static void build_state_sensor_pdr(void)
{
	uint8_t *p = state_sensor_pdr + sizeof(PDR_common_header);
	uint8_t *body = p;

	p = put16(p, PLAT_PLDM_TERMINUS_HANDLE);
	p = put16(p, SENSOR_ID_SW2);
	p = put16(p, PLDM_ENTITY_SUB_CHASSIS); /* entity type */
	p = put16(p, 1); /* entity instance number */
	p = put16(p, 0); /* container id */
	p = put8(p, PDR_SENSOR_NO_INIT); /* sensor init */
	p = put8(p, 0); /* sensor auxiliary names PDR */
	p = put8(p, 1); /* composite sensor count */
	/* one state: Presence */
	p = put16(p, PLDM_STATE_SET_PRESENCE);
	p = put8(p, 1); /* possible states size (bytes) */
	p = put8(p, BIT(PLDM_STATE_SET_PRESENT) | BIT(PLDM_STATE_SET_NOT_PRESENT)); /* 0x06 */

	build_common_header(state_sensor_pdr, PDR_HANDLE_SW2, PLDM_STATE_SENSOR_PDR,
			    (uint16_t)(p - body));
}

static void build_state_effecter_pdr(void)
{
	uint8_t *p = state_effecter_pdr + sizeof(PDR_common_header);
	uint8_t *body = p;

	p = put16(p, PLAT_PLDM_TERMINUS_HANDLE);
	p = put16(p, EFFECTER_ID_LED);
	p = put16(p, PLDM_ENTITY_DEVICE_DRIVER); /* entity type */
	p = put16(p, 1); /* entity instance number */
	p = put16(p, 0); /* container id */
	p = put16(p, 0); /* effecter semantic id */
	p = put8(p, PDR_SENSOR_NO_INIT); /* effecter init */
	p = put8(p, 0); /* effecter description PDR */
	p = put8(p, 1); /* composite effecter count */
	/* one state: OEM device status (1=off, 2=on for this board) */
	p = put16(p, PLDM_STATE_SET_OEM_DEVICE_STATUS);
	p = put8(p, 1); /* possible states size */
	p = put8(p, BIT(LED_STATE_OFF) | BIT(LED_STATE_ON)); /* 0x06 */

	build_common_header(state_effecter_pdr, PDR_HANDLE_LED, PLDM_STATE_EFFECTER_PDR,
			    (uint16_t)(p - body));
}

static void plat_pdr_init(void)
{
	build_terminus_locator_pdr();
	build_vmon_pdr();
	build_state_sensor_pdr();
	build_state_effecter_pdr();

	pdr_table[0] = (struct pdr_record){ PDR_HANDLE_TERMINUS_LOCATOR, terminus_locator_pdr,
					   sizeof(terminus_locator_pdr) };
	pdr_table[1] = (struct pdr_record){ PDR_HANDLE_VMON, (const uint8_t *)&vmon_pdr,
					   sizeof(vmon_pdr) };
	pdr_table[2] = (struct pdr_record){ PDR_HANDLE_SW2, state_sensor_pdr,
					   sizeof(state_sensor_pdr) };
	pdr_table[3] = (struct pdr_record){ PDR_HANDLE_LED, state_effecter_pdr,
					   sizeof(state_effecter_pdr) };

	uint32_t largest = 0;

	for (int i = 0; i < PDR_RECORD_COUNT; i++) {
		if (pdr_table[i].len > largest) {
			largest = pdr_table[i].len;
		}
	}

	memset(&plat_pdr_info, 0, sizeof(plat_pdr_info));
	plat_pdr_info.repository_state = PDR_STATE_AVAILABLE;
	plat_pdr_info.record_count = PDR_RECORD_COUNT;
	plat_pdr_info.repository_size = sizeof(terminus_locator_pdr) + sizeof(vmon_pdr) +
				       sizeof(state_sensor_pdr) + sizeof(state_effecter_pdr);
	plat_pdr_info.largest_record_size = largest;

	plat_pdr_ready = true;
}

/* record_handle 0 => first record. Returns index into pdr_table or -1. */
static int pdr_index_for_handle(uint32_t handle)
{
	if (handle == 0) {
		return 0;
	}
	for (int i = 0; i < PDR_RECORD_COUNT; i++) {
		if (pdr_table[i].handle == handle) {
			return i;
		}
	}
	return -1;
}

/* ---- command handlers ------------------------------------------------ */

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

	int idx = pdr_index_for_handle(req->record_handle);

	if (idx < 0) {
		resp[0] = 0x82; /* INVALID_RECORD_HANDLE */
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	uint16_t rec_len = pdr_table[idx].len;

	if (rec_len > sizeof(r->record_data)) {
		resp[0] = PLDM_ERROR;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	uint32_t next = (idx + 1 < PDR_RECORD_COUNT) ? pdr_table[idx + 1].handle : 0;

	memset(r, 0, sizeof(*r));
	r->completion_code = PLDM_SUCCESS;
	r->next_record_handle = next;
	r->next_data_transfer_handle = 0;
	r->transfer_flag = PLDM_TRANSFER_FLAG_START_AND_END;
	r->response_count = rec_len;
	memcpy(r->record_data, pdr_table[idx].data, rec_len);

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
	if (req->sensor_id != SENSOR_ID_VMON) {
		resp[0] = 0x80; /* INVALID_SENSOR_ID */
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	int32_t mdegc = 0;
	int ret = plat_read_vmon_mv(&mdegc);

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
	} else {
		r->sensor_operational_state = PLDM_SENSOR_ENABLED;
	}

	sys_put_le32((uint32_t)mdegc, r->present_reading);
	*resp_len = offsetof(struct pldm_get_sensor_reading_resp, present_reading) + sizeof(int32_t);
	return PLDM_SUCCESS;
}

static uint8_t plat_get_state_sensor_readings(void *mctp_inst, uint8_t *buf, uint16_t len,
					     uint8_t iid, uint8_t *resp, uint16_t *resp_len,
					     void *ext_params)
{
	ARG_UNUSED(mctp_inst);
	ARG_UNUSED(iid);
	ARG_UNUSED(ext_params);

	struct pldm_get_state_sensor_reading_req *req =
		(struct pldm_get_state_sensor_reading_req *)buf;
	struct pldm_get_state_sensor_reading_resp *r =
		(struct pldm_get_state_sensor_reading_resp *)resp;

	if (len != sizeof(*req)) {
		resp[0] = PLDM_ERROR_INVALID_LENGTH;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}
	if (req->sensor_id != SENSOR_ID_SW2) {
		resp[0] = PLDM_PLATFORM_INVALID_SENSOR_ID;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	uint8_t present = plat_sw2_present_state();

	memset(r, 0, sizeof(*r));
	r->completion_code = PLDM_SUCCESS;
	r->composite_sensor_count = 1;
	r->field[0].sensor_op_state = PLDM_SENSOR_ENABLED;
	r->field[0].present_state = present;
	r->field[0].previous_state = present;
	r->field[0].event_state = present;

	*resp_len = offsetof(struct pldm_get_state_sensor_reading_resp, field) +
		    sizeof(state_sensor_reading_state_field_t);
	return PLDM_SUCCESS;
}

static uint8_t plat_set_state_effecter_states(void *mctp_inst, uint8_t *buf, uint16_t len,
					     uint8_t iid, uint8_t *resp, uint16_t *resp_len,
					     void *ext_params)
{
	ARG_UNUSED(mctp_inst);
	ARG_UNUSED(iid);
	ARG_UNUSED(ext_params);

	struct pldm_set_state_effecter_states_req *req =
		(struct pldm_set_state_effecter_states_req *)buf;

	/* wire length = 3 fixed bytes + 2 per composite field */
	if (len < PLDM_SET_STATE_EFFECTER_REQ_NO_STATE_FIELD_BYTES) {
		resp[0] = PLDM_ERROR_INVALID_LENGTH;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}
	if (req->effecter_id != EFFECTER_ID_LED) {
		resp[0] = PLDM_PLATFORM_INVALID_EFFECTER_ID;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}
	if (req->composite_effecter_count != 1 ||
	    len != PLDM_SET_STATE_EFFECTER_REQ_NO_STATE_FIELD_BYTES +
			   req->composite_effecter_count * sizeof(set_effecter_state_field_t)) {
		resp[0] = PLDM_ERROR_INVALID_LENGTH;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	uint8_t cc = PLDM_SUCCESS;

	if (req->field[0].set_request == PLDM_REQUEST_SET) {
		int ret = plat_led_apply(req->field[0].effecter_state);

		if (ret == -EINVAL) {
			cc = PLDM_PLATFORM_INVALID_STATE_VALUE;
		} else if (ret) {
			cc = PLDM_ERROR;
		}
	}

	resp[0] = cc;
	*resp_len = 1;
	return PLDM_SUCCESS;
}

static uint8_t plat_get_state_effecter_states(void *mctp_inst, uint8_t *buf, uint16_t len,
					     uint8_t iid, uint8_t *resp, uint16_t *resp_len,
					     void *ext_params)
{
	ARG_UNUSED(mctp_inst);
	ARG_UNUSED(iid);
	ARG_UNUSED(ext_params);

	struct pldm_get_state_effecter_states_req *req =
		(struct pldm_get_state_effecter_states_req *)buf;
	struct pldm_get_state_effecter_states_resp *r =
		(struct pldm_get_state_effecter_states_resp *)resp;

	if (len != sizeof(*req)) {
		resp[0] = PLDM_ERROR_INVALID_LENGTH;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}
	if (req->effecter_id != EFFECTER_ID_LED) {
		resp[0] = PLDM_PLATFORM_INVALID_EFFECTER_ID;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	memset(r, 0, sizeof(*r));
	r->completion_code = PLDM_SUCCESS;
	r->composite_effecter_count = 1;
	r->field[0].effecter_op_state = PLDM_EFFECTER_ENABLED_NOUPDATEPENDING;
	r->field[0].pending_state = led_state;
	r->field[0].present_state = led_state;

	*resp_len = PLDM_GET_STATE_EFFECTER_RESP_NO_STATE_FIELD_BYTES +
		    sizeof(get_effecter_state_field_t);
	return PLDM_SUCCESS;
}

/* ---- asynchronous platform events --------------------------------------
 * SetEventReceiver registers where to send them; an SW2 transition then
 * pushes a PLDM stateSensorEvent (PlatformEventMessage, cmd 0x0A) to
 * that receiver over the outbound MCTP path. This is the "PLDM speaks
 * out over MCTP", not just "answers requests", direction.
 */

static bool event_recv_enabled;
static uint8_t event_recv_eid;
static mctp_ext_params event_recv_ext;
static uint8_t sw2_prev_event_state = PLDM_STATE_SET_NOT_PRESENT;

static void sw2_event_work_handler(struct k_work *work);
static K_WORK_DEFINE(sw2_event_work, sw2_event_work_handler);

static int send_state_sensor_event(uint16_t sensor_id, uint8_t event_state, uint8_t prev_state)
{
	void *inst = plat_mctp_get_inst();

	if (!inst || !event_recv_enabled) {
		return -ENOTCONN;
	}

	uint8_t payload[10];
	uint8_t *p = payload;

	*p++ = 0x01; /* eventMessage format version */
	*p++ = plat_pldm_get_tid(); /* our TID */
	*p++ = PLDM_SENSOR_EVENT; /* event class 0x00 */
	/* eventData: state sensor state */
	*p++ = sensor_id & 0xFF;
	*p++ = (sensor_id >> 8) & 0xFF;
	*p++ = PLDM_STATE_SENSOR_STATE; /* sensor event class 0x01 */
	*p++ = 0; /* sensor offset (composite index) */
	*p++ = event_state;
	*p++ = prev_state;

	pldm_msg msg = { 0 };

	msg.ext_params = event_recv_ext;
	msg.hdr.rq = 1;
	msg.hdr.pldm_type = PLDM_TYPE_PLAT_MON_CTRL;
	msg.hdr.cmd = PLDM_MONITOR_CMD_CODE_PLATFORM_EVENT_MESSAGE;
	msg.buf = payload;
	msg.len = (uint16_t)(p - payload);

	uint8_t rc = mctp_pldm_send_msg(inst, &msg);

	if (rc != PLDM_SUCCESS) {
		LOG_WRN("PlatformEventMessage send failed (%d)", rc);
		return -EIO;
	}
	LOG_INF("sent PLDM stateSensorEvent: sensor 0x%04x %u->%u to EID 0x%02x", sensor_id,
		prev_state, event_state, event_recv_eid);
	return 0;
}

static void sw2_event_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	uint8_t cur = plat_sw2_present_state();

	if (cur == sw2_prev_event_state) {
		return;
	}
	(void)send_state_sensor_event(SENSOR_ID_SW2, cur, sw2_prev_event_state);
	sw2_prev_event_state = cur;
}

/* plat_gpio.c hook - runs in the system workqueue after SW2 debounce. */
void plat_gpio_mon0_changed(int level)
{
	ARG_UNUSED(level);
	k_work_submit(&sw2_event_work);
}

static uint8_t plat_set_event_receiver(void *mctp_inst, uint8_t *buf, uint16_t len, uint8_t iid,
				       uint8_t *resp, uint16_t *resp_len, void *ext_params)
{
	ARG_UNUSED(mctp_inst);
	ARG_UNUSED(iid);

	struct pldm_set_event_receiver_req *req = (struct pldm_set_event_receiver_req *)buf;

	/* heartbeat_timer is only present for ASYNC_KEEP_ALIVE */
	if (len != sizeof(*req) && len != sizeof(*req) - sizeof(req->heartbeat_timer)) {
		resp[0] = PLDM_ERROR_INVALID_LENGTH;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}
	if (req->transport_protocol_type != PLDM_TRANSPORT_PROTOCOL_TYPE_MCTP) {
		resp[0] = PLDM_PLATFORM_INVALID_PROTOCOL_TYPE;
		*resp_len = 1;
		return PLDM_SUCCESS;
	}

	event_recv_enabled =
		(req->event_message_global_enable != PLDM_EVENT_MESSAGE_GLOBAL_DISABLE);
	event_recv_eid = req->event_receiver_address_info;
	if (ext_params) {
		memcpy(&event_recv_ext, ext_params, sizeof(event_recv_ext));
	}

	LOG_INF("SetEventReceiver: %s, receiver EID 0x%02x",
		event_recv_enabled ? "enabled" : "disabled", event_recv_eid);

	resp[0] = PLDM_SUCCESS;
	*resp_len = 1;
	return PLDM_SUCCESS;
}

/* ---- dispatch ------------------------------------------------------- */

static pldm_cmd_handler plat_pldm_monitor_cmd_tbl[] = {
	{ PLDM_MONITOR_CMD_CODE_SET_EVENT_RECEIVER, plat_set_event_receiver },
	{ PLDM_MONITOR_CMD_CODE_GET_SENSOR_READING, plat_get_sensor_reading },
	{ PLDM_MONITOR_CMD_CODE_GET_STATE_SENSOR_READING, plat_get_state_sensor_readings },
	{ PLDM_MONITOR_CMD_CODE_SET_STATE_EFFECTER_STATES, plat_set_state_effecter_states },
	{ PLDM_MONITOR_CMD_CODE_GET_STATE_EFFECTER_STATES, plat_get_state_effecter_states },
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

/* ---- public: local helpers for the shell test commands ------------- */

int plat_pldm_monitor_read_vmon_mv(int32_t *out_mdegc)
{
	return plat_read_vmon_mv(out_mdegc);
}

uint8_t plat_pldm_monitor_sw2_state(void)
{
	return plat_sw2_present_state();
}

int plat_pldm_monitor_set_led(bool on)
{
	return plat_led_apply(on ? LED_STATE_ON : LED_STATE_OFF);
}

uint8_t plat_pldm_monitor_led_state(void)
{
	return led_state;
}

uint32_t plat_pldm_monitor_pdr_count(void)
{
	return plat_pdr_ready ? PDR_RECORD_COUNT : 0;
}

bool plat_pldm_monitor_event_receiver(uint8_t *eid)
{
	if (eid) {
		*eid = event_recv_eid;
	}
	return event_recv_enabled;
}

int plat_pldm_monitor_fire_sw2_event(void)
{
	uint8_t cur = plat_sw2_present_state();
	int ret = send_state_sensor_event(SENSOR_ID_SW2, cur, sw2_prev_event_state);

	if (ret == 0) {
		sw2_prev_event_state = cur;
	}
	return ret;
}

void plat_pldm_monitor_init(void)
{
	if (HAVE_VMON && adc_is_ready_dt(&vmon_adc) && adc_channel_setup_dt(&vmon_adc) == 0) {
		vmon_ready = true;
	}

	plat_pdr_init();

	if (HAVE_LED && gpio_is_ready_dt(&led_effecter)) {
		gpio_pin_configure_dt(&led_effecter, GPIO_OUTPUT_INACTIVE);
	}

	int32_t mv = 0;
	bool vmon_ok = (plat_read_vmon_mv(&mv) == 0);

	LOG_INF("PLDM Type 2 up: %d PDRs (locator/numeric/state-sensor/state-effecter); "
		"vmon(0x0001) %s, SW2 %s, LED effecter %s",
		PDR_RECORD_COUNT, vmon_ok ? "readable" : "unavailable",
		plat_sw2_present_state() == PLDM_STATE_SET_PRESENT ? "present" : "not-present",
		HAVE_LED ? "ready" : "absent");
}
