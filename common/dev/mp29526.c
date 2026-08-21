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

/*
 * =============================================================================
 * MP29526 - Dual Loop Digital Multi-Phase Controller
 * =============================================================================
 *
 * Datasheet    : MP29526 Rev. 0.1, 2/26/2026 (preliminary)
 * Register map : MP29526 Register Map Rev 1.0.0.196 / 2.0.0.233, 2026-07-28
 * Modeled on   : mp29816a.c
 *
 * Everything below is taken from the register map. The preliminary datasheet
 * is used only for behavioural context; where the two disagree the register
 * map wins, and each such case is called out inline.
 *
 * -----------------------------------------------------------------------------
 * DATASHEET vs REGISTER MAP - corrections applied
 * -----------------------------------------------------------------------------
 *  1. VOUT_MODE (20h) is a standard 1-byte PMBus mode byte, not a multi-bit
 *     "VID step select" field. Four legal values only.
 *  2. VOUT_TRIM (22h) is fixed 0.3906 mV/LSB two's complement - it is NOT
 *     scaled by the VID step, unlike mp29816a.
 *  3. VOUT_OV/UV_FAULT_LIMIT (40h/44h) are plain 16-bit absolute thresholds in
 *     VOUT_MODE format. They contain no "delta" subfield, so the mp29816a
 *     GENMASK(12,9) offset arithmetic does not apply to this part at all.
 *  4. IOUT_OC_FAULT_LIMIT (46h) and IOUT_OC_WARN_LIMIT (4Ah) are linear11,
 *     not raw DAC codes. No KCS/RIMON board constants are needed.
 *  5. READ_VIN (88h) is linear11. Datasheet Table 5's "31.25mV/LSB" does not
 *     describe the register format.
 *  6. READ_VOUT (8Bh) result field is bits[11:0].
 *  7. Line-float flag is STATUS_MFR_PROTECT (80h, page 5/6) bit[0].
 *     Datasheet p.27 says bit[1]; that is bit[1] PWM_SELF_CHECK. Map wins.
 *  8. PHASE_NUM (01h page 5/6) is a plain count 0..16, not the encoded
 *     Table 1 pattern printed in the datasheet.
 *  9. NVM store is STORE_USER (15h), page 0, write-only, zero data bytes.
 *     The MP29816 unlock dance (page1@CCh / page0@17h / page2@1Ah / page3@81h)
 *     has no counterpart here and is deliberately not carried over.
 * 10. IC_DEVICE_ID (ADh) gives a real identity check:
 *     0x2020323935323620 ("  29526 ").
 * 11. Expected user CRC (B8h, page 0) is a PMBus block read (1B length + 4B
 *     data), not a plain 2-byte register. Confirmed against hardware
 *     readback: 04 c7 35 ae b3 -> CRC 0x35C7B3AE. See mp29526_get_fw_version().
 * =============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <logging/log.h>
#include "libutil.h"
#include "sensor.h"
#include "hal_i2c.h"
#include "pmbus.h"
#include "mp29526.h"
#include "util_pmbus.h"
#include "pldm_firmware_update.h"

LOG_MODULE_REGISTER(mp29526);

/* =============================================================================
 * Pages
 * ========================================================================== */

#define MP29526_PAGE_RAIL1 0x00 /* Rail 1 operating registers      */
#define MP29526_PAGE_RAIL2 0x01 /* Rail 2 operating registers      */
#define MP29526_PAGE_MULTICFG 0x02 /* multi-config, saved by STORE_USER */
#define MP29526_PAGE_MFR_R1 0x05 /* MPS registers affecting Rail 1  */
#define MP29526_PAGE_MFR_R2 0x06 /* MPS registers affecting Rail 2  */
#define MP29526_PAGE_MFR_ALL 0x09 /* MPS registers, all rails       */

/* =============================================================================
 * Register map
 * ========================================================================== */

/* ---- Page 0..3 : per-rail operating registers ---------------------------- */
#define MP29526_REG_OPERATION 0x01 /* 1B  */
#define MP29526_REG_ON_OFF_CONFIG 0x02 /* 1B  */
#define MP29526_REG_CLEAR_FAULT 0x03 /* 0B, write only */
#define MP29526_REG_PHASE 0x04 /* 1B  */
#define MP29526_REG_PASSKEY 0x0E /* 1B  */
#define MP29526_REG_WRITE_PROTECT 0x10 /* 1B  */
#define MP29526_REG_STORE_USER 0x15 /* 0B, write only */
#define MP29526_REG_RESTORE_USER 0x16 /* 0B, write only */
#define MP29526_REG_CAPABILITY 0x19 /* 1B  */
#define MP29526_REG_VOUT_MODE 0x20 /* 1B  */
#define MP29526_REG_VOUT_COMMAND 0x21 /* 2B, VID format */
#define MP29526_REG_VOUT_TRIM 0x22 /* 2B, 0.3906mV/LSB two's complement */
#define MP29526_REG_VOUT_CAL_OFFSET 0x23 /* 2B, VID format */
#define MP29526_REG_VOUT_MAX 0x24 /* 2B, VID format */
#define MP29526_REG_VOUT_MARGIN_HIGH 0x25 /* 2B, VID format */
#define MP29526_REG_VOUT_MARGIN_LOW 0x26 /* 2B, VID format */
#define MP29526_REG_VOUT_TRANSITION_RATE 0x27 /* 2B */
#define MP29526_REG_VOUT_MIN 0x2B /* 2B, VID format */
#define MP29526_REG_FREQUENCY_SWITCH 0x33 /* 2B, 1kHz/LSB bits[12:0] */
#define MP29526_REG_POWER_MODE 0x34 /* 1B  */
#define MP29526_REG_VOUT_OV_FAULT_LIMIT 0x40 /* 2B, VID format */
#define MP29526_REG_VOUT_OV_FAULT_RESPONSE 0x41 /* 1B */
#define MP29526_REG_VOUT_UV_FAULT_LIMIT 0x44 /* 2B, VID format */
#define MP29526_REG_VOUT_UV_FAULT_RESPONSE 0x45 /* 1B */
#define MP29526_REG_IOUT_OC_FAULT_LIMIT 0x46 /* 2B, linear11 */
#define MP29526_REG_IOUT_OC_FAULT_RESPONSE 0x47 /* 1B */
#define MP29526_REG_IOUT_OC_WARN_LIMIT 0x4A /* 2B, linear11 */
#define MP29526_REG_OT_FAULT_LIMIT 0x4F /* 2B, linear11 */
#define MP29526_REG_STATUS_BYTE 0x78 /* 1B */
#define MP29526_REG_STATUS_WORD 0x79 /* 2B */
#define MP29526_REG_STATUS_VOUT 0x7A /* 1B */
#define MP29526_REG_STATUS_IOUT 0x7B /* 1B */
#define MP29526_REG_STATUS_INPUT 0x7C /* 1B */
#define MP29526_REG_STATUS_TEMPERATURE 0x7D /* 1B */
#define MP29526_REG_STATUS_CML 0x7E /* 1B */
#define MP29526_REG_STATUS_OTHER 0x7F /* 1B */
#define MP29526_REG_STATUS_MFR_SPECIFIC 0x80 /* 1B, page 0..3 */
#define MP29526_REG_READ_VIN 0x88 /* 2B, linear11 */
#define MP29526_REG_READ_IIN 0x89 /* 2B, linear11 */
#define MP29526_REG_READ_VOUT 0x8B /* 2B, bits[11:0] VID format */
#define MP29526_REG_READ_IOUT 0x8C /* 2B, linear11 */
#define MP29526_REG_READ_TEMP 0x8D /* 2B, linear11 */
#define MP29526_REG_READ_DUTY_CYCLE 0x94 /* 2B, 1%/LSB */
#define MP29526_REG_READ_FREQUENCY 0x95 /* 2B, 1kHz/LSB */
#define MP29526_REG_READ_POUT 0x96 /* 2B, linear11 */
#define MP29526_REG_READ_PIN 0x97 /* 2B, linear11 */
#define MP29526_REG_MFR_ID 0x99 /* block, 4B */
#define MP29526_REG_MFR_MODEL 0x9A /* block, 2B */
#define MP29526_REG_IC_DEVICE_ID 0xAD /* block, 8B */
#define MP29526_REG_IC_DEVICE_REV 0xAE /* block, 1B */
#define MP29526_REG_USER_DATA_00 0xB0 /* block, 2B */
#define MP29526_REG_MFR_CODE_VERSION 0xEE /* 2B */
/*
 * Expected user CRC register. This is a PMBus block read, NOT a plain 2-byte
 * register: byte 0 is a length count (4), followed by 4 CRC data bytes. Confirmed
 * against actual hardware (page 0, `i2c read ... 0xB8 8` -> 04 c7 35 ae b3 ...).
 */
#define VR_REG_EXPECTED_USER_CRC 0xB8 /* block, 1B len + 4B data */
#define MP29526_EXPECTED_USER_CRC_LEN 4

/* ---- Page 5..8 : MPS registers, per rail --------------------------------- */
#define MP29526_REG_PHASE_NUM 0x01 /* 1B, bits[4:0], 0=off 1..16 */
#define MP29526_REG_VBOOT 0x02 /* 2B, VID format */
#define MP29526_REG_RSAMP_GAIN 0x03 /* 1B */
#define MP29526_REG_TSNS_FAULT_CFG 0x04 /* 1B */
#define MP29526_REG_PH_OCL_THR 0x05 /* 1B */
#define MP29526_REG_OV_VID_THR 0x06 /* 2B, 1mV/LSB */
#define MP29526_REG_UV_VID_THR 0x07 /* 2B, 1mV/LSB */
#define MP29526_REG_OV_CFG 0x08 /* 1B */
#define MP29526_REG_STATUS_MFR_PROTECT 0x80 /* 2B, read only */

/* ---- Page 9 : MPS registers, all rails ----------------------------------- */
#define MP29526_REG_MULTICFG_SEL 0x09 /* 1B, bit3 override, bits[2:0] addr */

/*
 * IC_DEVICE_ID (ADh) is a block read: byte 0 is the length, then 8 ID bytes
 * MSB-first. 0x2020323935323620 is ASCII "  29526 ".
 * (MP29426 is 0x2020323934323620, MP29529 is 0x2020323935323920.)
 */
#define MP29526_IC_DEVICE_ID_VALUE 0x2020323935323620ULL
#define MP29526_IC_DEVICE_ID_LEN 8

/* Field masks - all confirmed against the register map */
#define MP29526_READ_VOUT_MASK GENMASK(11, 0)
#define MP29526_PHASE_NUM_MASK GENMASK(4, 0)
#define MP29526_LINEAR11_EXP_MASK GENMASK(15, 11)
#define MP29526_LINEAR11_MANTISSA_MASK GENMASK(9, 0) /* bit10 is reserved */
#define MP29526_FAULT_RESPONSE_MODE_MASK GENMASK(7, 6)
#define MP29526_FAULT_RESPONSE_RETRY_MASK GENMASK(5, 3)
#define MP29526_FAULT_RESPONSE_DELAY_MASK GENMASK(2, 0)

/* VOUT_OV/UV_FAULT_RESPONSE bits[7:6] */
#define MP29526_FAULT_MODE_CONTINUE 0x0
#define MP29526_FAULT_MODE_DELAY 0x1
#define MP29526_FAULT_MODE_SHUTDOWN 0x2

/* VOUT_TRIM is a fixed-resolution register, independent of VOUT_MODE */
#define MP29526_VOUT_TRIM_LSB_MV 0.3906f

#define MP29526_MAX_PHASE 16
#define MP29526_MAX_READ_BYTES (8 + 1) /* IC_DEVICE_ID: 8 data + 1 length */
#define MAX_CFG_LEN 1024

/* wait 1s after STORE_USER, 100ms after RESTORE_USER before checking STATUS_CML */
#define MP29526_NVM_STORE_DELAY_MS 1000
#define MP29526_NVM_RESTORE_DELAY_MS 100

/* =============================================================================
 * Low level access
 * ========================================================================== */

bool mp29526_i2c_read(uint8_t bus, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	if (len > MP29526_MAX_READ_BYTES) {
		LOG_ERR("mp29526 read len %d is too long", len);
		return false;
	}

	memset(data, 0, len);

	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 1;
	i2c_msg.rx_len = len;
	i2c_msg.data[0] = reg;

	if (i2c_master_read(&i2c_msg, retry)) {
		LOG_ERR("Failed to read mp29526, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr, reg);
		return false;
	}

	memcpy(data, i2c_msg.data, len);
	return true;
}

bool mp29526_i2c_write(uint8_t bus, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = len + 1;
	i2c_msg.data[0] = reg;

	if (len > 0) {
		CHECK_NULL_ARG_WITH_RETURN(data, false);
		memcpy(&i2c_msg.data[1], data, len);
	}

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to write mp29526, bus: %d, addr: 0x%x, reg: 0x%x", bus, addr, reg);
		return false;
	}

	return true;
}

bool mp29526_set_page(uint8_t bus, uint8_t addr, uint8_t page)
{
	I2C_MSG i2c_msg = { 0 };
	uint8_t retry = 5;
	i2c_msg.bus = bus;
	i2c_msg.target_addr = addr;
	i2c_msg.tx_len = 2;
	i2c_msg.data[0] = PMBUS_PAGE;
	i2c_msg.data[1] = page;

	if (i2c_master_write(&i2c_msg, retry)) {
		LOG_ERR("Failed to set mp29526 page, bus: %d, addr: 0x%x, page: 0x%x", bus, addr,
			page);
		return false;
	}

	return true;
}

/* Rail 1 -> page 0, Rail 2 -> page 1 */
static bool mp29526_select_rail(sensor_cfg *cfg, uint8_t rail)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	if (rail >= MP29526_RAIL_MAX) {
		LOG_ERR("VR[0x%x] invalid rail %d (MP29526 has 2 loops)", cfg->num, rail);
		return false;
	}

	uint8_t page = (rail == MP29526_RAIL_1) ? MP29526_PAGE_RAIL1 : MP29526_PAGE_RAIL2;
	return mp29526_set_page(cfg->port, cfg->target_addr, page);
}

/* Rail 1 -> page 5, Rail 2 -> page 6 */
static bool mp29526_select_mfr_page(sensor_cfg *cfg, uint8_t rail)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	if (rail >= MP29526_RAIL_MAX) {
		LOG_ERR("VR[0x%x] invalid rail %d", cfg->num, rail);
		return false;
	}

	uint8_t page = (rail == MP29526_RAIL_1) ? MP29526_PAGE_MFR_R1 : MP29526_PAGE_MFR_R2;
	return mp29526_set_page(cfg->port, cfg->target_addr, page);
}

static uint16_t le16(const uint8_t *d)
{
	return (uint16_t)(d[0] | (d[1] << 8));
}

static void put_le16(uint8_t *d, uint16_t v)
{
	d[0] = v & 0xFF;
	d[1] = (v >> 8) & 0xFF;
}

/* =============================================================================
 * linear11 helpers
 *
 * util_pmbus.h supplies slinear11_to_float(). The encode direction is not
 * provided there, so it lives here. Format: 5-bit signed exponent in bits
 * [15:11], mantissa in bits[10:0]. On MP29526 the OC-limit registers reserve
 * bit[10], leaving a 10-bit positive mantissa - which is why the search below
 * caps the mantissa at MP29526_LINEAR11_MANTISSA_MASK rather than 0x3FF worth
 * of signed range.
 * ========================================================================== */

static bool mp29526_float_to_linear11(float value, uint16_t *out)
{
	CHECK_NULL_ARG_WITH_RETURN(out, false);

	if (value < 0) {
		LOG_ERR("mp29526 linear11 encode: negative value %d not supported here",
			(int)value);
		return false;
	}

	/*
	 * Pick the smallest exponent whose mantissa still fits, which maximises
	 * resolution. Exponent range for a 5-bit two's complement field is
	 * -16..+15. Scaling is done with shifts rather than powf() so the
	 * driver does not pull in libm.
	 */
	for (int exp = -16; exp <= 15; exp++) {
		float scaled;

		if (exp < 0)
			scaled = value * (float)(1UL << (-exp));
		else
			scaled = value / (float)(1UL << exp);

		if (scaled > (float)MP29526_LINEAR11_MANTISSA_MASK)
			continue;

		uint16_t mantissa = (uint16_t)(scaled + 0.5f);
		if (mantissa > MP29526_LINEAR11_MANTISSA_MASK)
			continue;

		*out = ((uint16_t)(exp & 0x1F) << 11) | (mantissa & MP29526_LINEAR11_MANTISSA_MASK);
		return true;
	}

	LOG_ERR("mp29526 linear11 encode: value %d out of representable range", (int)value);
	return false;
}

/* =============================================================================
 * VOUT_MODE / VID step
 * ========================================================================== */

bool mp29526_get_vout_mode(sensor_cfg *cfg, uint8_t rail, uint8_t *mode)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(mode, false);

	if (!mp29526_select_rail(cfg, rail))
		return false;

	uint8_t data[1] = { 0 };
	if (!mp29526_i2c_read(cfg->port, cfg->target_addr, MP29526_REG_VOUT_MODE, data,
			      sizeof(data))) {
		return false;
	}

	*mode = data[0];
	return true;
}

/*
 * VOUT_MODE (20h) is one byte with exactly four legal values. Anything else
 * means the read is bogus or the part is misconfigured - return 0 (= error)
 * rather than picking a plausible-looking step, because a wrong step silently
 * mis-scales every voltage the BMC reports and every voltage it writes.
 */
float mp29526_get_vid_step(sensor_cfg *cfg, uint8_t rail)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, 0);

	uint8_t mode = 0;
	if (!mp29526_get_vout_mode(cfg, rail, &mode))
		return 0;

	switch (mode) {
	case MP29526_VOUT_MODE_DIRECT_1MV:
		return 0.001f;
	case MP29526_VOUT_MODE_LINEAR_512:
		return 1.0f / 512.0f; /* 1.953125 mV */
	case MP29526_VOUT_MODE_VID_5MV:
		return 0.005f;
	case MP29526_VOUT_MODE_VID_10MV:
		return 0.010f;
	default:
		LOG_ERR("VR[0x%x] rail%d VOUT_MODE 0x%02x is not one of the four legal values "
			"(0x40/0x22/0x21/0x17)",
			cfg->num, rail + 1, mode);
		return 0;
	}
}

/* =============================================================================
 * VID-format register helpers
 *
 * VOUT_COMMAND, VOUT_MAX, VOUT_MIN, VOUT_OV/UV_FAULT_LIMIT and VBOOT all share
 * one representation: 16-bit unsigned, scaled by the VID step.
 * ========================================================================== */

static bool mp29526_get_vid_reg(sensor_cfg *cfg, uint8_t rail, uint8_t reg, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	float vid_step = mp29526_get_vid_step(cfg, rail);
	if (vid_step == 0)
		return false;

	if (!mp29526_select_rail(cfg, rail))
		return false;

	uint8_t data[2] = { 0 };
	if (!mp29526_i2c_read(cfg->port, cfg->target_addr, reg, data, sizeof(data)))
		return false;

	*millivolt = (uint16_t)((float)le16(data) * vid_step * 1000.0f);
	return true;
}

static bool mp29526_set_vid_reg(sensor_cfg *cfg, uint8_t rail, uint8_t reg, uint16_t millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	float vid_step = mp29526_get_vid_step(cfg, rail);
	if (vid_step == 0)
		return false;

	if (!mp29526_select_rail(cfg, rail))
		return false;

	float code_f = ((float)millivolt / 1000.0f) / vid_step + 0.5f;
	if (code_f > (float)UINT16_MAX) {
		LOG_ERR("VR[0x%x] rail%d %d mV overflows the 16-bit VID field at this VOUT_MODE",
			cfg->num, rail + 1, millivolt);
		return false;
	}

	uint8_t data[2] = { 0 };
	put_le16(data, (uint16_t)code_f);

	return mp29526_i2c_write(cfg->port, cfg->target_addr, reg, data, sizeof(data));
}

bool mp29526_get_vout_max(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	return mp29526_get_vid_reg(cfg, rail, MP29526_REG_VOUT_MAX, millivolt);
}

bool mp29526_get_vout_min(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	return mp29526_get_vid_reg(cfg, rail, MP29526_REG_VOUT_MIN, millivolt);
}

bool mp29526_set_vout_max(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);
	return mp29526_set_vid_reg(cfg, rail, MP29526_REG_VOUT_MAX, *millivolt);
}

bool mp29526_set_vout_min(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);
	return mp29526_set_vid_reg(cfg, rail, MP29526_REG_VOUT_MIN, *millivolt);
}

/* =============================================================================
 * VOUT command and trim
 * ========================================================================== */

/*
 * VOUT_TRIM (22h) fine-tunes the output. It is two's complement with a FIXED
 * 0.3906 mV/LSB resolution - do not scale it by the VID step. mp29816a treated
 * its trim register as a signed count of VID steps; copying that here would
 * mis-scale by up to 25x at the 10mV VID setting.
 */
static bool mp29526_read_trim_mv(sensor_cfg *cfg, float *trim_mv)
{
	uint8_t data[2] = { 0 };
	if (!mp29526_i2c_read(cfg->port, cfg->target_addr, MP29526_REG_VOUT_TRIM, data,
			      sizeof(data))) {
		return false;
	}

	*trim_mv = (float)((int16_t)le16(data)) * MP29526_VOUT_TRIM_LSB_MV;
	return true;
}

bool mp29526_get_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	float vid_step = mp29526_get_vid_step(cfg, rail);
	if (vid_step == 0)
		return false;

	if (!mp29526_select_rail(cfg, rail))
		return false;

	uint8_t data[2] = { 0 };
	if (!mp29526_i2c_read(cfg->port, cfg->target_addr, MP29526_REG_VOUT_COMMAND, data,
			      sizeof(data))) {
		return false;
	}

	float trim_mv = 0;
	if (!mp29526_read_trim_mv(cfg, &trim_mv))
		return false;

	float vout_mv = (float)le16(data) * vid_step * 1000.0f + trim_mv;
	if (vout_mv < 0)
		vout_mv = 0;

	*millivolt = (uint16_t)vout_mv;
	return true;
}

bool mp29526_set_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	float vid_step = mp29526_get_vid_step(cfg, rail);
	if (vid_step == 0)
		return false;

	if (!mp29526_select_rail(cfg, rail))
		return false;

	float trim_mv = 0;
	if (!mp29526_read_trim_mv(cfg, &trim_mv))
		return false;

	float target_mv = (float)(*millivolt) - trim_mv;
	if (target_mv < 0)
		target_mv = 0;

	float code_f = (target_mv / 1000.0f) / vid_step + 0.5f;
	if (code_f > (float)UINT16_MAX) {
		LOG_ERR("VR[0x%x] rail%d VOUT %d mV overflows the VID field", cfg->num, rail + 1,
			*millivolt);
		return false;
	}
	uint16_t code = (uint16_t)code_f;

	/*
	 * Clamp check against VOUT_MAX / VOUT_MIN. The silicon clamps silently
	 * (datasheet p.18), so without this the caller would see a successful
	 * write and a different rail voltage.
	 */
	uint16_t vmax_mv = 0, vmin_mv = 0;
	if (mp29526_get_vout_max(cfg, rail, &vmax_mv) &&
	    mp29526_get_vout_min(cfg, rail, &vmin_mv)) {
		if (*millivolt > vmax_mv || *millivolt < vmin_mv) {
			LOG_ERR("VR[0x%x] rail%d VOUT %d mV is outside VOUT_MIN %d / VOUT_MAX %d; "
				"the VR would clamp it silently",
				cfg->num, rail + 1, *millivolt, vmin_mv, vmax_mv);
			return false;
		}
		/* re-select: the limit reads above changed the page */
		if (!mp29526_select_rail(cfg, rail))
			return false;
	}

	uint8_t data[2] = { 0 };
	put_le16(data, code);

	if (!mp29526_i2c_write(cfg->port, cfg->target_addr, MP29526_REG_VOUT_COMMAND, data,
			       sizeof(data))) {
		return false;
	}

	/*
	 * WRITE_PROTECT (10h) can reject this write, and a rejected PMBus write
	 * is indistinguishable from an accepted one at the I2C layer. Read back
	 * so the failure surfaces here.
	 */
	uint8_t verify[2] = { 0 };
	if (!mp29526_i2c_read(cfg->port, cfg->target_addr, MP29526_REG_VOUT_COMMAND, verify,
			      sizeof(verify))) {
		return false;
	}

	if (le16(verify) != code) {
		LOG_ERR("VR[0x%x] rail%d VOUT_COMMAND write dropped: wrote 0x%04x, read 0x%04x "
			"(check WRITE_PROTECT 10h)",
			cfg->num, rail + 1, code, le16(verify));
		return false;
	}

	return true;
}

bool mp29526_get_vout_offset(sensor_cfg *cfg, uint16_t *vout_offset)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(vout_offset, false);

	if (!mp29526_select_rail(cfg, MP29526_RAIL_1))
		return false;

	float trim_mv = 0;
	if (!mp29526_read_trim_mv(cfg, &trim_mv))
		return false;

	*vout_offset = (uint16_t)((trim_mv < 0) ? 0 : trim_mv);
	return true;
}

bool mp29526_set_vout_offset(sensor_cfg *cfg, uint16_t *write_vout_offset)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(write_vout_offset, false);

	if (!mp29526_select_rail(cfg, MP29526_RAIL_1))
		return false;

	float code_f = (float)(*write_vout_offset) / MP29526_VOUT_TRIM_LSB_MV;
	if (code_f > (float)INT16_MAX) {
		LOG_ERR("VR[0x%x] vout trim %d mV overflows the signed 16-bit field", cfg->num,
			*write_vout_offset);
		return false;
	}

	uint8_t data[2] = { 0 };
	put_le16(data, (uint16_t)((int16_t)(code_f + 0.5f)));

	return mp29526_i2c_write(cfg->port, cfg->target_addr, MP29526_REG_VOUT_TRIM, data,
				 sizeof(data));
}

/* =============================================================================
 * IOUT limits - linear11, in amperes
 * ========================================================================== */

static bool mp29526_get_iout_limit(sensor_cfg *cfg, uint8_t reg, uint16_t *amps)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(amps, false);

	if (!mp29526_select_rail(cfg, MP29526_RAIL_1))
		return false;

	uint8_t data[2] = { 0 };
	if (!mp29526_i2c_read(cfg->port, cfg->target_addr, reg, data, sizeof(data)))
		return false;

	float val = slinear11_to_float(le16(data));
	if (val < 0) {
		LOG_ERR("VR[0x%x] reg 0x%02x decoded to a negative current limit", cfg->num, reg);
		return false;
	}

	*amps = (uint16_t)(val + 0.5f);
	return true;
}

static bool mp29526_set_iout_limit(sensor_cfg *cfg, uint8_t reg, uint16_t amps)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	uint16_t encoded = 0;
	if (!mp29526_float_to_linear11((float)amps, &encoded))
		return false;

	if (!mp29526_select_rail(cfg, MP29526_RAIL_1))
		return false;

	uint8_t data[2] = { 0 };
	put_le16(data, encoded);

	if (!mp29526_i2c_write(cfg->port, cfg->target_addr, reg, data, sizeof(data)))
		return false;

	/* linear11 is lossy; report what actually landed so the caller can see
	 * the quantisation rather than assume an exact set. */
	uint8_t verify[2] = { 0 };
	if (!mp29526_i2c_read(cfg->port, cfg->target_addr, reg, verify, sizeof(verify)))
		return false;

	if (le16(verify) != encoded) {
		LOG_ERR("VR[0x%x] reg 0x%02x write dropped: wrote 0x%04x, read 0x%04x "
			"(check WRITE_PROTECT 10h)",
			cfg->num, reg, encoded, le16(verify));
		return false;
	}

	float actual = slinear11_to_float(encoded);
	if ((uint16_t)(actual + 0.5f) != amps) {
		LOG_WRN("VR[0x%x] reg 0x%02x requested %dA, linear11 quantised to %dA", cfg->num,
			reg, amps, (uint16_t)(actual + 0.5f));
	}

	return true;
}

bool mp29526_get_iout_oc_warn_limit(sensor_cfg *cfg, uint16_t *value)
{
	return mp29526_get_iout_limit(cfg, MP29526_REG_IOUT_OC_WARN_LIMIT, value);
}

bool mp29526_set_iout_oc_warn_limit(sensor_cfg *cfg, uint16_t value)
{
	return mp29526_set_iout_limit(cfg, MP29526_REG_IOUT_OC_WARN_LIMIT, value);
}

bool mp29526_get_total_ocp(sensor_cfg *cfg, uint16_t *total_ocp)
{
	return mp29526_get_iout_limit(cfg, MP29526_REG_IOUT_OC_FAULT_LIMIT, total_ocp);
}

bool mp29526_set_total_ocp(sensor_cfg *cfg, uint16_t *write_total_ocp)
{
	CHECK_NULL_ARG_WITH_RETURN(write_total_ocp, false);
	return mp29526_set_iout_limit(cfg, MP29526_REG_IOUT_OC_FAULT_LIMIT, *write_total_ocp);
}

/* =============================================================================
 * OVP / UVP
 *
 * MP29526 has two independent OV mechanisms and two UV mechanisms:
 *   ABS : VOUT_OV_FAULT_LIMIT (40h) / VOUT_UV_FAULT_LIMIT (44h) on page 0/1,
 *         absolute thresholds in VOUT_MODE format.
 *   VID : OV_VID_THR (06h) / UV_VID_THR (07h) on page 5/6, 1 mV/LSB.
 * The existing API maps ovp_1 -> ABS and ovp_2 -> VID, matching mp29816a's
 * OVP1/OVP2 naming.
 * ========================================================================== */

bool mp29526_get_ovp_1(sensor_cfg *cfg, uint16_t *ovp_1)
{
	return mp29526_get_vid_reg(cfg, MP29526_RAIL_1, MP29526_REG_VOUT_OV_FAULT_LIMIT, ovp_1);
}

bool mp29526_set_ovp_1(sensor_cfg *cfg, uint16_t *write_ovp_1)
{
	CHECK_NULL_ARG_WITH_RETURN(write_ovp_1, false);
	return mp29526_set_vid_reg(cfg, MP29526_RAIL_1, MP29526_REG_VOUT_OV_FAULT_LIMIT,
				   *write_ovp_1);
}

bool mp29526_get_uvp(sensor_cfg *cfg, uint16_t *uvp_threshold)
{
	return mp29526_get_vid_reg(cfg, MP29526_RAIL_1, MP29526_REG_VOUT_UV_FAULT_LIMIT,
				   uvp_threshold);
}

bool mp29526_set_uvp_threshold(sensor_cfg *cfg, uint16_t *write_uvp_threshold)
{
	CHECK_NULL_ARG_WITH_RETURN(write_uvp_threshold, false);
	return mp29526_set_vid_reg(cfg, MP29526_RAIL_1, MP29526_REG_VOUT_UV_FAULT_LIMIT,
				   *write_uvp_threshold);
}

/* VID_OVP / VID_UVP, page 5/6, plain 1 mV/LSB */
static bool mp29526_get_vid_thr(sensor_cfg *cfg, uint8_t rail, uint8_t reg, uint16_t *millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(millivolt, false);

	if (!mp29526_select_mfr_page(cfg, rail))
		return false;

	uint8_t data[2] = { 0 };
	if (!mp29526_i2c_read(cfg->port, cfg->target_addr, reg, data, sizeof(data)))
		return false;

	*millivolt = le16(data);
	return true;
}

static bool mp29526_set_vid_thr(sensor_cfg *cfg, uint8_t rail, uint8_t reg, uint16_t millivolt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	if (!mp29526_select_mfr_page(cfg, rail))
		return false;

	uint8_t data[2] = { 0 };
	put_le16(data, millivolt);

	return mp29526_i2c_write(cfg->port, cfg->target_addr, reg, data, sizeof(data));
}

bool mp29526_get_ov_vid_threshold(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	return mp29526_get_vid_thr(cfg, rail, MP29526_REG_OV_VID_THR, millivolt);
}

bool mp29526_set_ov_vid_threshold(sensor_cfg *cfg, uint8_t rail, uint16_t millivolt)
{
	return mp29526_set_vid_thr(cfg, rail, MP29526_REG_OV_VID_THR, millivolt);
}

bool mp29526_get_uv_vid_threshold(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt)
{
	return mp29526_get_vid_thr(cfg, rail, MP29526_REG_UV_VID_THR, millivolt);
}

bool mp29526_set_uv_vid_threshold(sensor_cfg *cfg, uint8_t rail, uint16_t millivolt)
{
	return mp29526_set_vid_thr(cfg, rail, MP29526_REG_UV_VID_THR, millivolt);
}

bool mp29526_get_ovp_2(sensor_cfg *cfg, uint16_t *ovp_2)
{
	return mp29526_get_ov_vid_threshold(cfg, MP29526_RAIL_1, ovp_2);
}

/*
 * VOUT_OV_FAULT_RESPONSE (41h) bits[7:6]:
 *   00 continue without interruption
 *   01 continue for MFR_OVP_SET_DELAYTIME, then act per retry bits[5:3]
 *   10 shut down, then act per retry bits[5:3]
 *   11 unused
 * Note this is a different encoding from mp29816a, where 0/1 meant
 * "No action"/"Latch off". Bits[5:3] and [2:0] are preserved on write.
 */
bool mp29526_get_ovp_2_action(sensor_cfg *cfg, uint16_t *ovp_2_action)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(ovp_2_action, false);

	if (!mp29526_select_rail(cfg, MP29526_RAIL_1))
		return false;

	uint8_t data[1] = { 0 };
	if (!mp29526_i2c_read(cfg->port, cfg->target_addr, MP29526_REG_VOUT_OV_FAULT_RESPONSE, data,
			      sizeof(data))) {
		return false;
	}

	*ovp_2_action = (data[0] & MP29526_FAULT_RESPONSE_MODE_MASK) >> 6;
	return true;
}

bool mp29526_set_ovp_2_action(sensor_cfg *cfg, uint16_t *write_ovp_2_action)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(write_ovp_2_action, false);

	if (*write_ovp_2_action > MP29526_FAULT_MODE_SHUTDOWN) {
		LOG_ERR("VR[0x%x] OVP response %d invalid: 0=continue, 1=delay-then-retry, "
			"2=shutdown (3 is unused)",
			cfg->num, *write_ovp_2_action);
		return false;
	}

	if (!mp29526_select_rail(cfg, MP29526_RAIL_1))
		return false;

	uint8_t data[1] = { 0 };
	if (!mp29526_i2c_read(cfg->port, cfg->target_addr, MP29526_REG_VOUT_OV_FAULT_RESPONSE, data,
			      sizeof(data))) {
		return false;
	}

	/* preserve retry count and delay time */
	data[0] = (data[0] & ~MP29526_FAULT_RESPONSE_MODE_MASK) |
		  ((*write_ovp_2_action << 6) & MP29526_FAULT_RESPONSE_MODE_MASK);

	return mp29526_i2c_write(cfg->port, cfg->target_addr, MP29526_REG_VOUT_OV_FAULT_RESPONSE,
				 data, sizeof(data));
}

/* =============================================================================
 * Status
 * ========================================================================== */

bool mp29526_get_vr_status(sensor_cfg *cfg, uint8_t rail, uint8_t vr_status_rail,
			   uint16_t *vr_status)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(vr_status, false);

	if (!mp29526_select_rail(cfg, rail))
		return false;

	uint16_t val = 0;
	uint8_t data[2] = { 0 };

	switch (vr_status_rail) {
	case MP29526_REG_STATUS_WORD:
		if (!mp29526_i2c_read(cfg->port, cfg->target_addr, MP29526_REG_STATUS_WORD, data,
				      2))
			return false;
		val = le16(data);
		break;

	case MP29526_REG_STATUS_BYTE:
	case MP29526_REG_STATUS_VOUT:
	case MP29526_REG_STATUS_IOUT:
	case MP29526_REG_STATUS_INPUT:
	case MP29526_REG_STATUS_TEMPERATURE:
	case MP29526_REG_STATUS_CML:
	case MP29526_REG_STATUS_OTHER:
	case MP29526_REG_STATUS_MFR_SPECIFIC:
		if (!mp29526_i2c_read(cfg->port, cfg->target_addr, vr_status_rail, data, 1))
			return false;
		val = (uint16_t)data[0];
		break;

	default:
		LOG_ERR("VR[0x%x] does not support vr status: 0x%x", cfg->num, vr_status_rail);
		return false;
	}

	*vr_status = val;
	return true;
}

bool mp29526_clear_vr_status(sensor_cfg *cfg, uint8_t rail)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);

	if (!mp29526_select_rail(cfg, rail))
		return false;

	/* CLEAR_FAULT (03h) is send-byte: command code only, no data */
	if (!mp29526_i2c_write(cfg->port, cfg->target_addr, MP29526_REG_CLEAR_FAULT, NULL, 0)) {
		LOG_ERR("VR[0x%x] rail%d clear fault failed", cfg->num, rail + 1);
		return false;
	}

	return true;
}

bool mp29526_get_mfr_protect(sensor_cfg *cfg, uint8_t rail, uint16_t *status)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(status, false);

	if (!mp29526_select_mfr_page(cfg, rail))
		return false;

	uint8_t data[2] = { 0 };
	if (!mp29526_i2c_read(cfg->port, cfg->target_addr, MP29526_REG_STATUS_MFR_PROTECT, data,
			      sizeof(data))) {
		return false;
	}

	*status = le16(data);
	return true;
}

bool mp29526_get_line_float_fault(sensor_cfg *cfg, uint8_t rail, bool *is_faulted)
{
	CHECK_NULL_ARG_WITH_RETURN(is_faulted, false);

	uint16_t status = 0;
	if (!mp29526_get_mfr_protect(cfg, rail, &status))
		return false;

	*is_faulted = (status & MP29526_PROTECT_LINE_FLOAT) ? true : false;

	if (*is_faulted) {
		LOG_WRN("VR[0x%x] rail%d line-float fault latched - check VOSEN%d/VORTN%d "
			"routing. The rail is shut down and stays latched until CLEAR_FAULT.",
			cfg->num, rail + 1, rail + 1, rail + 1);
	}

	return true;
}

bool mp29526_get_phase_count(sensor_cfg *cfg, uint8_t rail, uint8_t *phase_cnt)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, false);
	CHECK_NULL_ARG_WITH_RETURN(phase_cnt, false);

	if (!mp29526_select_mfr_page(cfg, rail))
		return false;

	uint8_t data[1] = { 0 };
	if (!mp29526_i2c_read(cfg->port, cfg->target_addr, MP29526_REG_PHASE_NUM, data,
			      sizeof(data))) {
		return false;
	}

	uint8_t cnt = data[0] & MP29526_PHASE_NUM_MASK;
	if (cnt > MP29526_MAX_PHASE) {
		LOG_ERR("VR[0x%x] rail%d PHASE_NUM %d is out of range (0=off, 1..16)", cfg->num,
			rail + 1, cnt);
		return false;
	}

	*phase_cnt = cnt;
	return true;
}

/* =============================================================================
 * Identity and write protection
 * ========================================================================== */

bool mp29526_check_device_id(uint8_t bus, uint8_t addr)
{
	if (!mp29526_set_page(bus, addr, MP29526_PAGE_RAIL1))
		return false;

	/*
	 * Block read: byte 0 is the length, then 8 ID bytes - confirmed against
	 * hardware that they arrive LSB-first, i.e. reversed relative to
	 * MP29526_IC_DEVICE_ID_VALUE's natural MSB-first byte order. On the
	 * wire: 20 36 32 35 39 32 20 20 (reads as "  62592 " in transmission
	 * order); read back data[8]..data[1] and it's "  29526 ", matching
	 * MP29526_IC_DEVICE_ID_VALUE.
	 */
	uint8_t data[MP29526_IC_DEVICE_ID_LEN + 1] = { 0 };
	if (!mp29526_i2c_read(bus, addr, MP29526_REG_IC_DEVICE_ID, data, sizeof(data))) {
		LOG_ERR("Failed to read IC_DEVICE_ID (ADh)");
		return false;
	}

	if (data[0] != MP29526_IC_DEVICE_ID_LEN) {
		LOG_ERR("IC_DEVICE_ID block length %d, expected %d", data[0],
			MP29526_IC_DEVICE_ID_LEN);
		return false;
	}

	uint64_t id = 0;
	for (int i = MP29526_IC_DEVICE_ID_LEN; i >= 1; i--)
		id = (id << 8) | data[i];

	if (id != MP29526_IC_DEVICE_ID_VALUE) {
		LOG_ERR("IC_DEVICE_ID 0x%08x%08x is not MP29526 (expected 0x%08x%08x) - wrong "
			"part or wrong address",
			(uint32_t)(id >> 32), (uint32_t)id,
			(uint32_t)(MP29526_IC_DEVICE_ID_VALUE >> 32),
			(uint32_t)MP29526_IC_DEVICE_ID_VALUE);
		return false;
	}

	return true;
}

bool mp29526_get_write_protect(uint8_t bus, uint8_t addr, uint8_t *wp)
{
	CHECK_NULL_ARG_WITH_RETURN(wp, false);

	if (!mp29526_set_page(bus, addr, MP29526_PAGE_RAIL1))
		return false;

	uint8_t data[1] = { 0 };
	if (!mp29526_i2c_read(bus, addr, MP29526_REG_WRITE_PROTECT, data, sizeof(data)))
		return false;

	*wp = data[0];
	return true;
}

bool mp29526_set_write_protect(uint8_t bus, uint8_t addr, uint8_t wp)
{
	if (!mp29526_set_page(bus, addr, MP29526_PAGE_RAIL1))
		return false;

	if (wp != MP29526_WP_DISABLE && wp != MP29526_WP_ALL_EXCEPT_WP &&
	    wp != MP29526_WP_EXCEPT_WP_OP_PAGE && wp != MP29526_WP_EXCEPT_WP_OP_PAGE_ONOFF_VOUT &&
	    wp != 0x01) {
		LOG_ERR("WRITE_PROTECT value 0x%02x is not accepted by the device", wp);
		return false;
	}

	uint8_t data[1] = { wp };
	if (!mp29526_i2c_write(bus, addr, MP29526_REG_WRITE_PROTECT, data, sizeof(data)))
		return false;

	uint8_t readback = 0;
	if (!mp29526_get_write_protect(bus, addr, &readback))
		return false;

	if (readback != wp) {
		LOG_ERR("WRITE_PROTECT did not take: wrote 0x%02x, read 0x%02x", wp, readback);
		return false;
	}

	return true;
}

/*
 * Config revision, reported as the expected user CRC (B8h, page 0) - the
 * value the device compares against the CRC it generates each time NVM is
 * copied to configuration registers. Same role as mp29816a's checksum at
 * EDh and mp2971's MFR checksum, so callers can treat it interchangeably.
 *
 * B8h is a PMBus block read: byte 0 is a length count (4), then 4 CRC data
 * bytes. The two halves are each little-endian but concatenate in
 * transmission order (first half -> high 16 bits, second half -> low 16
 * bits) - confirmed against hardware: 04 c7 35 ae b3 -> CRC 0x35C7B3AE.
 * It is NOT one 32-bit little-endian word.
 */
bool mp29526_get_fw_version(uint8_t bus, uint8_t addr, uint32_t *rev)
{
	CHECK_NULL_ARG_WITH_RETURN(rev, false);

	if (!mp29526_set_page(bus, addr, MP29526_PAGE_RAIL1)) {
		LOG_ERR("Failed to set page before reading expected user CRC");
		return false;
	}

	uint8_t data[MP29526_EXPECTED_USER_CRC_LEN + 1] = { 0 };
	if (!mp29526_i2c_read(bus, addr, VR_REG_EXPECTED_USER_CRC, data, sizeof(data))) {
		LOG_ERR("Failed to read expected user CRC (B8h)");
		return false;
	}

	if (data[0] != MP29526_EXPECTED_USER_CRC_LEN) {
		LOG_ERR("Expected user CRC block length %d, expected %d", data[0],
			MP29526_EXPECTED_USER_CRC_LEN);
		return false;
	}

	*rev = ((uint32_t)le16(&data[1]) << 16) | le16(&data[3]);
	return true;
}

/* =============================================================================
 * Config update
 *
 * Flow, all of it now documented:
 *   1. IC_DEVICE_ID check - refuse to program the wrong part.
 *   2. WRITE_PROTECT = 0x00.
 *   3. Walk the parsed config, switching PAGE as required.
 *   4. STORE_USER (15h) on page 0 - copies pages 0/1/2 to NVM with two CRC
 *      sets. Write-only, no data byte.
 *   5. Wait, then check STATUS_CML (7Eh) bit[4] NVM_CRC_ERROR and bit[5]
 *      PEC_ERROR.
 *   6. Restore WRITE_PROTECT.
 *
 * RESTORE_USER (16h) is deliberately NOT issued: the register map states it
 * cannot be sent while the device is outputting power, and this driver has no
 * way to know the rail state. The stored config takes effect at the next POR.
 *
 * The parser handles the tab-separated MPS GUI export used by mp29816a. If the
 * MP29526 GUI emits a different column order, parsing fails loudly at the
 * cfg_cnt == 0 check rather than writing garbage.
 * ========================================================================== */

enum CFG_PARAM_IDX {
	CFG_PARAM_IDX_PAGE = 1,
	CFG_PARAM_IDX_REG_ADDR = 2,
	CFG_PARAM_IDX_REG_NAME = 4,
	CFG_PARAM_IDX_REG_VAL = 5,
	CFG_PARAM_IDX_REG_LEN = 7
};

struct cfg_data {
	uint8_t cfg_page;
	uint8_t cfg_idx;
	uint8_t reg_addr;
	uint16_t reg_val;
	uint8_t reg_len;
};

static int cnt_char(char *s, char c)
{
	int cnt = 0;
	while (*s) {
		if (*s == c)
			cnt++;
		s++;
	}
	return cnt;
}

static uint8_t parsing_line(char *str, uint16_t len, struct cfg_data *cfg_data)
{
	if (!str || !cfg_data)
		return 1;

	char *save_ptr;
	char *k = strtok_r(str, "\t", &save_ptr);
	uint8_t idx = 0;

	do {
		switch (idx++) {
		case CFG_PARAM_IDX_PAGE:
			cfg_data->cfg_page = strtol(k, NULL, 16) & 0x0F;
			cfg_data->cfg_idx = (strtol(k, NULL, 16) & 0xF0) >> 4;
			break;
		case CFG_PARAM_IDX_REG_ADDR:
			cfg_data->reg_addr = strtol(k, NULL, 16);
			break;
		case CFG_PARAM_IDX_REG_NAME:
			if (!strncmp(k, "CRC", strlen("CRC"))) {
				LOG_INF("CRC: %s", log_strdup(k));
				return 1;
			} else if (!strncmp(k, "TRIM", strlen("TRIM"))) {
				LOG_INF("TRIM: %s", log_strdup(k));
				return 1;
			}
			break;
		case CFG_PARAM_IDX_REG_VAL:
			cfg_data->reg_val = strtol(k, NULL, 16);
			break;
		case CFG_PARAM_IDX_REG_LEN:
			cfg_data->reg_len = strtol(k, NULL, 10);
			break;
		default:
			break;
		}
	} while ((k = strtok_r(NULL, "\t", &save_ptr)));

	return 0;
}

static uint32_t parsing_image(const uint8_t *img_buff, uint32_t img_size,
			      struct cfg_data *cfg_data_list, uint32_t max_cfg_len)
{
	if (!img_buff || !cfg_data_list)
		return 0;

	uint32_t i = 0;
	uint32_t cfg_cnt = 0;

	while (i < img_size) {
		/* line length; only Windows line endings (\r\n) are supported */
		int len = 0;
		while (((i + len) < img_size) &&
		       ((img_buff[i + len] != '\r') && (img_buff[i + len] != '\n')))
			len++;

		if (!len) {
			i = i + 2;
			continue;
		}

		char tmp[128] = { 0 };
		if (len >= (int)sizeof(tmp)) {
			LOG_ERR("Config line %d bytes exceeds parse buffer", len);
			return 0;
		}
		strncpy(tmp, (const char *)img_buff + i, len);

		/* "END" marks the end of the real register writes; everything
		 * after it (CRC_CHECK_START/STOP, CUSTOM_START/STOP) is
		 * reference/readback data, not writes - see mp29526_stream's
		 * past_end comment for the full rationale. */
		if (len == (int)strlen("END") && !strncmp(tmp, "END", len))
			break;

		if (cnt_char(tmp, '\t') == 7) {
			if (!parsing_line(tmp, len, cfg_data_list + cfg_cnt))
				cfg_cnt++;
		}

		if (cfg_cnt >= max_cfg_len) {
			LOG_ERR("Too many config data");
			return 0;
		}

		i = i + len + 2;
	}

	return cfg_cnt;
}

static bool mp29526_check_nvm_status(uint8_t bus, uint8_t addr)
{
	if (!mp29526_set_page(bus, addr, MP29526_PAGE_RAIL1))
		return false;

	uint8_t cml[1] = { 0 };
	if (!mp29526_i2c_read(bus, addr, MP29526_REG_STATUS_CML, cml, sizeof(cml))) {
		LOG_ERR("Failed to read STATUS_CML after STORE_USER");
		return false;
	}

	if (cml[0] & MP29526_CML_NVM_CRC_ERROR) {
		LOG_ERR("NVM CRC error after STORE_USER (STATUS_CML 0x%02x). The stored config is "
			"invalid and the VR will shut down on restore.",
			cml[0]);
		return false;
	}

	if (cml[0] & MP29526_CML_PEC_ERROR) {
		LOG_ERR("PEC error flagged during config write (STATUS_CML 0x%02x) - at least one "
			"register write was rejected",
			cml[0]);
		return false;
	}

	return true;
}

static uint8_t mp29526_do_update(struct cfg_data *cfg_data_list, uint32_t cfg_cnt, uint8_t bus,
				 uint8_t addr)
{
	if (!cfg_data_list || !cfg_cnt)
		return 1;

	if (!mp29526_check_device_id(bus, addr))
		return 1;

	uint8_t saved_wp = 0;
	if (!mp29526_get_write_protect(bus, addr, &saved_wp))
		return 1;

	if (saved_wp != MP29526_WP_DISABLE) {
		LOG_INF("WRITE_PROTECT was 0x%02x, disabling for the update", saved_wp);
		if (!mp29526_set_write_protect(bus, addr, MP29526_WP_DISABLE))
			return 1;
	}

	uint8_t ret = 1;
	uint8_t page = 0xFF;
	uint8_t data[MP29526_MAX_READ_BYTES] = { 0 };

	/* clear any stale CML bits so the post-store check is meaningful */
	mp29526_i2c_write(bus, addr, MP29526_REG_CLEAR_FAULT, NULL, 0);

	for (uint32_t i = 0; i < cfg_cnt; i++) {
		const struct cfg_data *p = cfg_data_list + i;

		if (p->reg_len > 2) {
			LOG_ERR("Config entry %d has unsupported length %d", i, p->reg_len);
			goto restore_wp;
		}

		if (p->cfg_page != page) {
			page = p->cfg_page;
			if (!mp29526_set_page(bus, addr, page))
				goto restore_wp;
		}

		memcpy(data, &p->reg_val, p->reg_len);
		if (!mp29526_i2c_write(bus, addr, p->reg_addr, data, p->reg_len)) {
			LOG_ERR("Config write failed at entry %d (page 0x%x reg 0x%02x)", i,
				p->cfg_page, p->reg_addr);
			goto restore_wp;
		}
	}

	/* STORE_USER (15h) on page 0: send byte, no data */
	if (!mp29526_set_page(bus, addr, MP29526_PAGE_RAIL1))
		goto restore_wp;

	if (!mp29526_i2c_write(bus, addr, MP29526_REG_STORE_USER, NULL, 0)) {
		LOG_ERR("STORE_USER (15h) failed");
		goto restore_wp;
	}

	k_msleep(MP29526_NVM_STORE_DELAY_MS);

	if (!mp29526_check_nvm_status(bus, addr))
		goto restore_wp;

	LOG_INF("MP29526 config stored to NVM (%d registers). Takes effect at the next POR; "
		"RESTORE_USER is not issued because it is illegal while the rail is powered.",
		cfg_cnt);
	ret = 0;

restore_wp:
	if (saved_wp != MP29526_WP_DISABLE) {
		if (!mp29526_set_write_protect(bus, addr, saved_wp))
			LOG_ERR("Failed to restore WRITE_PROTECT to 0x%02x - the device is left "
				"unprotected",
				saved_wp);
	}

	return ret;
}

bool mp29526_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size)
{
	CHECK_NULL_ARG_WITH_RETURN(img_buff, false);

	bool ret = false;

	struct cfg_data *cfg_data_list =
		(struct cfg_data *)malloc(MAX_CFG_LEN * sizeof(struct cfg_data));
	if (!cfg_data_list) {
		LOG_ERR("malloc fail");
		return false;
	}

	uint32_t cfg_cnt = parsing_image(img_buff, img_size, cfg_data_list, MAX_CFG_LEN);
	LOG_INF("cfg_cnt %d", cfg_cnt);

	if (!cfg_cnt) {
		LOG_ERR("parsing image fail");
		goto exit;
	}

	/* mp29816a discarded this return value; a failed update must surface */
	if (mp29526_do_update(cfg_data_list, cfg_cnt, bus, addr)) {
		LOG_ERR("mp29526 config update failed");
		goto exit;
	}

	ret = true;
exit:
	SAFE_FREE(cfg_data_list);
	return ret;
}

/* =============================================================================
 * Config update - streaming variant
 *
 * mp29526 images are ~200KB of the same tab-separated ATE-format text lines
 * that mp29526_fwupdate() above buffers whole and parses in one pass. That
 * doesn't fit this platform's heap, so this
 * variant is fed one PLDM data chunk at a time (in offset order) and applies
 * each complete "\r\n"-terminated line to the device as soon as it is
 * assembled, instead of malloc'ing the full image.
 *
 * Column layout (tab-separated):
 *   col0  configuration ID (constant per file, informational only here)
 *   col1  page, or "<group><page>" for a multi-config register (group is
 *         the leading digit 1-5 when the field is 3 digits long; a 1-2
 *         digit field is a plain page number with no group)
 *   col2  register address, hex. For "P1"/"P2"/"BP1" rows this is
 *         <command_code_hi><data_byte_lo> packed together, not a plain
 *         address.
 *   col3  register address, decimal (redundant, ignored)
 *   col4  register name ("CRC_USER_Multi_N"/"TRIM*" rows are reference
 *         values, not configuration writes)
 *   col5  value, hex
 *   col6  value, decimal (redundant, ignored)
 *   col7  row type: "1"/"2" = byte/word write; "Bn" = block write of n
 *         bytes (n<=4: the value is already in this one row; n>4: one
 *         byte per row across n consecutive rows, concatenated into a
 *         single block write); "P1"/"P2"/"BP1" = process-call write-word
 *         to the command code encoded in col2's high byte
 *
 * Rows before "END" are real writes. CRC_CHECK_START/STOP brackets the
 * per-group reference CRC, read back from B8h after STORE_USER and
 * compared here as a log-only sanity check. CUSTOM_START/STOP and
 * anything after CRC_CHECK_STOP is readback verification data, not
 * writes, and is ignored entirely.
 * ========================================================================== */

#define MP29526_STREAM_LINE_BUF_LEN 128
#define MP29526_STREAM_BLOCK_MAX_LEN 32
#define MP29526_MULTICFG_SEL_BASE 0x08
#define MP29526_MULTICFG_GROUP_MAX 5

struct mp29526_row {
	uint8_t group; /* 0 = shared/user register, 1-5 = multi-config set */
	uint8_t page;
	uint16_t addr_field;
	char name[40];
	char value_hex[16];
	char tag[8];
};

static struct {
	uint8_t bus;
	uint8_t addr;
	uint8_t page;
	uint8_t group; /* multi-config set currently selected on the device */
	uint8_t saved_wp;
	uint32_t applied_cnt;
	uint16_t line_len;
	bool aborted;
	bool config_id_logged;
	/* "\r\n"-terminated line reassembly, spanning PLDM chunk boundaries */
	char line_buf[MP29526_STREAM_LINE_BUF_LEN];
	/*
	 * "Bn" rows with n>4 spread one byte per row across n consecutive
	 * lines (MSByte first) instead of putting the whole value in one
	 * row; these fields reassemble that run into a single block write.
	 */
	uint8_t block_cmd;
	uint8_t block_want;
	uint8_t block_have;
	uint8_t block_data[MP29526_STREAM_BLOCK_MAX_LEN];
	/*
	 * state machine for the file's tail: real writes, then "END", then
	 * CRC_CHECK_START..CRC_CHECK_STOP (reference CRC capture only), then
	 * CUSTOM_START..CUSTOM_STOP plus trailing comments (ignored).
	 */
	bool past_end;
	bool in_crc_block;
	uint32_t crc_ref[MP29526_MULTICFG_GROUP_MAX + 1];
	bool crc_ref_valid[MP29526_MULTICFG_GROUP_MAX + 1];
} mp29526_stream;

/* col1: "<group><page>" (3 digits) or "<page>" (1-2 digits). */
static bool mp29526_parse_group_page(const char *s, uint8_t *group, uint8_t *page)
{
	size_t len = strlen(s);
	if (len == 0 || len > 3)
		return false;

	if (len <= 2) {
		*group = 0;
		*page = (uint8_t)strtol(s, NULL, 16);
	} else {
		if (s[0] < '0' || s[0] > '9')
			return false;
		*group = (uint8_t)(s[0] - '0');
		*page = (uint8_t)strtol(s + 1, NULL, 16);
	}
	return true;
}

static bool mp29526_hex_to_bytes_msb_first(const char *hex, uint8_t *out, uint8_t n)
{
	if (strlen(hex) != (size_t)(n * 2))
		return false;

	for (uint8_t i = 0; i < n; i++) {
		char byte_str[3] = { hex[i * 2], hex[i * 2 + 1], '\0' };
		out[i] = (uint8_t)strtol(byte_str, NULL, 16);
	}
	return true;
}

static bool mp29526_parse_row(char *line, struct mp29526_row *row)
{
	memset(row, 0, sizeof(*row));

	char *save_ptr;
	char *tok = strtok_r(line, "\t", &save_ptr);
	int idx = 0;

	while (tok) {
		switch (idx) {
		case 1:
			if (!mp29526_parse_group_page(tok, &row->group, &row->page))
				return false;
			break;
		case 2:
			row->addr_field = (uint16_t)strtol(tok, NULL, 16);
			break;
		case 4:
			strncpy(row->name, tok, sizeof(row->name) - 1);
			break;
		case 5:
			strncpy(row->value_hex, tok, sizeof(row->value_hex) - 1);
			break;
		case 7:
			strncpy(row->tag, tok, sizeof(row->tag) - 1);
			break;
		default:
			break;
		}
		idx++;
		tok = strtok_r(NULL, "\t", &save_ptr);
	}

	return idx == 8;
}

static void mp29526_stream_restore_wp(void)
{
	if (mp29526_stream.saved_wp != MP29526_WP_DISABLE) {
		if (!mp29526_set_write_protect(mp29526_stream.bus, mp29526_stream.addr,
					       mp29526_stream.saved_wp))
			LOG_ERR("Failed to restore WRITE_PROTECT to 0x%02x - the device is left "
				"unprotected",
				mp29526_stream.saved_wp);
	}
}

/* STORE_USER + wait + B8h CRC readback (log-only compare) + RESTORE_USER +
 * wait + STATUS_CML check, for whichever group's registers were just written. */
static bool mp29526_stream_store_and_verify(uint8_t group)
{
	uint8_t bus = mp29526_stream.bus;
	uint8_t addr = mp29526_stream.addr;

	if (!mp29526_set_page(bus, addr, MP29526_PAGE_RAIL1))
		return false;
	mp29526_stream.page = MP29526_PAGE_RAIL1;

	if (!mp29526_i2c_write(bus, addr, MP29526_REG_STORE_USER, NULL, 0)) {
		LOG_ERR("STORE_USER failed for group %d", group);
		return false;
	}
	k_msleep(MP29526_NVM_STORE_DELAY_MS);

	uint32_t rev = 0;
	if (mp29526_get_fw_version(bus, addr, &rev)) {
		if (group <= MP29526_MULTICFG_GROUP_MAX && mp29526_stream.crc_ref_valid[group]) {
			if (rev != mp29526_stream.crc_ref[group])
				LOG_WRN("Group %d CRC mismatch: B8h readback 0x%08x, file expects 0x%08x",
					group, rev, mp29526_stream.crc_ref[group]);
			else
				LOG_INF("Group %d CRC matches file (0x%08x)", group, rev);
		} else {
			LOG_INF("Group %d B8h readback 0x%08x (no reference captured to compare)",
				group, rev);
		}
	} else {
		LOG_WRN("Failed to read back B8h for group %d verification", group);
	}

	if (!mp29526_i2c_write(bus, addr, MP29526_REG_RESTORE_USER, NULL, 0)) {
		LOG_ERR("RESTORE_USER failed for group %d", group);
		return false;
	}
	k_msleep(MP29526_NVM_RESTORE_DELAY_MS);

	if (!mp29526_check_nvm_status(bus, addr)) {
		LOG_ERR("NVM status check failed after group %d store/restore", group);
		return false;
	}

	mp29526_stream.page = MP29526_PAGE_RAIL1;
	return true;
}

/* Finishes the group being left (store+verify, unless it's the initial
 * "no group yet" state) and selects the new group on the device. */
static bool mp29526_stream_switch_group(uint8_t new_group)
{
	if (new_group == mp29526_stream.group)
		return true;

	/* group 0 (shared registers) has no store of its own - it gets
	 * committed together with whichever group is written first */
	if (mp29526_stream.group != 0) {
		if (!mp29526_stream_store_and_verify(mp29526_stream.group))
			return false;
	}

	if (new_group != 0) {
		if (!mp29526_set_page(mp29526_stream.bus, mp29526_stream.addr,
				      MP29526_PAGE_MFR_ALL))
			return false;
		mp29526_stream.page = MP29526_PAGE_MFR_ALL;

		uint8_t sel = MP29526_MULTICFG_SEL_BASE + (new_group - 1);
		uint8_t d[1] = { sel };
		if (!mp29526_i2c_write(mp29526_stream.bus, mp29526_stream.addr,
				       MP29526_REG_MULTICFG_SEL, d, 1)) {
			LOG_ERR("Failed to select multi-config set %d", new_group);
			return false;
		}
		LOG_INF("Selected multi-config set %d (09h=0x%02x)", new_group, sel);
	}

	mp29526_stream.group = new_group;
	return true;
}

static bool mp29526_stream_flush_block(void)
{
	if (!mp29526_stream.block_want)
		return true;

	if (mp29526_stream.block_have != mp29526_stream.block_want) {
		LOG_ERR("Block write for reg 0x%02x truncated (%d of %d bytes)",
			mp29526_stream.block_cmd, mp29526_stream.block_have,
			mp29526_stream.block_want);
		return false;
	}

	uint8_t payload[MP29526_STREAM_BLOCK_MAX_LEN + 1];
	payload[0] = mp29526_stream.block_want;
	/* wire order is the file's row order reversed */
	for (uint8_t i = 0; i < mp29526_stream.block_want; i++)
		payload[1 + i] = mp29526_stream.block_data[mp29526_stream.block_want - 1 - i];

	bool ok = mp29526_i2c_write(mp29526_stream.bus, mp29526_stream.addr,
				    mp29526_stream.block_cmd, payload,
				    mp29526_stream.block_want + 1);
	if (!ok)
		LOG_ERR("Block write failed (reg 0x%02x, %d bytes)", mp29526_stream.block_cmd,
			mp29526_stream.block_want);
	else
		mp29526_stream.applied_cnt++;

	mp29526_stream.block_want = 0;
	mp29526_stream.block_have = 0;
	return ok;
}

static bool mp29526_stream_apply_row(const struct mp29526_row *row)
{
	if (!strncmp(row->name, "CRC", strlen("CRC")) || !strncmp(row->name, "TRIM", strlen("TRIM")))
		return true;

	if (row->group != mp29526_stream.group) {
		if (!mp29526_stream_flush_block())
			return false;
		if (!mp29526_stream_switch_group(row->group))
			return false;
	}

	if (row->page != mp29526_stream.page) {
		if (!mp29526_set_page(mp29526_stream.bus, mp29526_stream.addr, row->page))
			return false;
		mp29526_stream.page = row->page;
	}

	if (!strcmp(row->tag, "1") || !strcmp(row->tag, "2")) {
		uint8_t len = (uint8_t)(row->tag[0] - '0');
		uint8_t cmd = (uint8_t)row->addr_field;
		uint16_t val = (uint16_t)strtol(row->value_hex, NULL, 16);
		uint8_t data[2];
		memcpy(data, &val, len);
		if (!mp29526_i2c_write(mp29526_stream.bus, mp29526_stream.addr, cmd, data, len)) {
			LOG_ERR("Config write failed (page 0x%x reg 0x%02x)", row->page, cmd);
			return false;
		}
		mp29526_stream.applied_cnt++;
	} else if (!strcmp(row->tag, "P1") || !strcmp(row->tag, "P2") || !strcmp(row->tag, "BP1")) {
		if (!mp29526_stream_flush_block())
			return false;
		uint8_t cmd = (uint8_t)(row->addr_field >> 8);
		uint8_t addr_low = (uint8_t)(row->addr_field & 0xFF);
		uint8_t value_byte;
		if (!mp29526_hex_to_bytes_msb_first(row->value_hex, &value_byte, 1)) {
			LOG_ERR("Malformed process-call value for %s", log_strdup(row->name));
			return false;
		}
		/* PMBus write-word sends the low byte first on the wire */
		uint8_t data[2] = { addr_low, value_byte };
		if (!mp29526_i2c_write(mp29526_stream.bus, mp29526_stream.addr, cmd, data, 2)) {
			LOG_ERR("Process-call write failed (page 0x%x reg 0x%02x)", row->page,
				cmd);
			return false;
		}
		mp29526_stream.applied_cnt++;
	} else if (row->tag[0] == 'B') {
		/* checked after P1/P2/BP1 above, since "BP1" also starts with 'B' */
		uint8_t n = (uint8_t)atoi(row->tag + 1);
		uint8_t cmd = (uint8_t)row->addr_field;
		if (!n || n > MP29526_STREAM_BLOCK_MAX_LEN) {
			LOG_ERR("Unsupported block length in tag '%s' for %s",
				log_strdup(row->tag), log_strdup(row->name));
			return false;
		}

		if (n <= 4) {
			if (!mp29526_stream_flush_block())
				return false;
			uint8_t bytes[4];
			if (!mp29526_hex_to_bytes_msb_first(row->value_hex, bytes, n)) {
				LOG_ERR("Malformed block value for %s", log_strdup(row->name));
				return false;
			}
			uint8_t payload[5];
			payload[0] = n;
			/* wire order is the hex string's byte-pairs reversed */
			for (uint8_t i = 0; i < n; i++)
				payload[1 + i] = bytes[n - 1 - i];
			if (!mp29526_i2c_write(mp29526_stream.bus, mp29526_stream.addr, cmd,
					       payload, n + 1)) {
				LOG_ERR("Block write failed (page 0x%x reg 0x%02x)", row->page,
					cmd);
				return false;
			}
			mp29526_stream.applied_cnt++;
		} else {
			if (mp29526_stream.block_want && (mp29526_stream.block_cmd != cmd ||
							  mp29526_stream.block_want != n)) {
				LOG_WRN("Block run for reg 0x%02x changed mid-sequence, restarting",
					cmd);
				mp29526_stream.block_want = 0;
				mp29526_stream.block_have = 0;
			}
			if (!mp29526_stream.block_want) {
				mp29526_stream.block_cmd = cmd;
				mp29526_stream.block_want = n;
				mp29526_stream.block_have = 0;
			}
			uint8_t byte_val;
			if (!mp29526_hex_to_bytes_msb_first(row->value_hex, &byte_val, 1)) {
				LOG_ERR("Malformed block byte for %s", log_strdup(row->name));
				return false;
			}
			mp29526_stream.block_data[mp29526_stream.block_have++] = byte_val;
			if (mp29526_stream.block_have == mp29526_stream.block_want) {
				if (!mp29526_stream_flush_block())
					return false;
			}
		}
	} else {
		LOG_WRN("Unknown row type tag '%s' for %s, skipping", log_strdup(row->tag),
			log_strdup(row->name));
	}

	return true;
}

/* CRC_CHECK_START..STOP rows only capture a reference value, keyed by
 * group; they're never written. col1 is always "0" for these, so the
 * group comes from the REG_NAME suffix ("CRC_USER_Multi_<N>") instead. */
static void mp29526_stream_capture_crc_ref(const struct mp29526_row *row)
{
	uint8_t group = row->group;
	const char *last_us = strrchr(row->name, '_');
	if (last_us && last_us[1] >= '1' && last_us[1] <= '9' && last_us[2] == '\0')
		group = (uint8_t)(last_us[1] - '0');

	if (group < 1 || group > MP29526_MULTICFG_GROUP_MAX) {
		LOG_WRN("Could not determine group for CRC reference row '%s'",
			log_strdup(row->name));
		return;
	}

	uint32_t val = (uint32_t)strtoul(row->value_hex, NULL, 16);
	mp29526_stream.crc_ref[group] = val;
	mp29526_stream.crc_ref_valid[group] = true;
}

static bool mp29526_stream_apply_line(void)
{
	if (!mp29526_stream.line_len)
		return true;

	char *line = mp29526_stream.line_buf;
	uint16_t len = mp29526_stream.line_len;
	if (line[len - 1] == '\r')
		line[--len] = '\0';
	else
		line[len] = '\0';
	mp29526_stream.line_len = 0;

	if (!len)
		return true;

	if (mp29526_stream.past_end) {
		if (!strcmp(line, "CRC_CHECK_START")) {
			mp29526_stream.in_crc_block = true;
		} else if (!strcmp(line, "CRC_CHECK_STOP")) {
			mp29526_stream.in_crc_block = false;
		} else if (mp29526_stream.in_crc_block && cnt_char(line, '\t') == 7) {
			struct mp29526_row row;
			if (mp29526_parse_row(line, &row))
				mp29526_stream_capture_crc_ref(&row);
		}
		return true;
	}

	if (!strcmp(line, "END")) {
		LOG_INF("Reached END marker; finishing group %d and ignoring anything after it",
			mp29526_stream.group);
		bool ok = mp29526_stream_flush_block() &&
			  (mp29526_stream.group == 0 && !mp29526_stream.applied_cnt ?
				   true :
				   mp29526_stream_store_and_verify(mp29526_stream.group));
		mp29526_stream.past_end = true;
		return ok;
	}

	if (cnt_char(line, '\t') != 7)
		return true;

	struct mp29526_row row;
	if (!mp29526_parse_row(line, &row)) {
		LOG_ERR("Malformed config line (wrong column count)");
		return false;
	}

	return mp29526_stream_apply_row(&row);
}

static bool mp29526_stream_consume(const uint8_t *data, uint32_t data_len)
{
	for (uint32_t i = 0; i < data_len; i++) {
		char c = (char)data[i];

		if (c == '\n') {
			if (!mp29526_stream_apply_line())
				return false;
			continue;
		}

		if (mp29526_stream.line_len >= MP29526_STREAM_LINE_BUF_LEN - 1) {
			LOG_ERR("Config line exceeds parse buffer (%d bytes)",
				MP29526_STREAM_LINE_BUF_LEN);
			return false;
		}
		mp29526_stream.line_buf[mp29526_stream.line_len++] = c;
	}
	return true;
}

bool mp29526_fwupdate_stream(uint8_t bus, uint8_t addr, const uint8_t *data, uint32_t data_len,
			     bool is_first, bool is_last)
{
	CHECK_NULL_ARG_WITH_RETURN(data, false);

	if (is_first) {
		memset(&mp29526_stream, 0, sizeof(mp29526_stream));
		mp29526_stream.bus = bus;
		mp29526_stream.addr = addr;
		mp29526_stream.page = 0xFF;
		mp29526_stream.group = 0;

		if (!mp29526_check_device_id(bus, addr)) {
			mp29526_stream.aborted = true;
			return false;
		}

		/* informational only - logged, does not gate the update */
		uint8_t pw[2] = { 0 };
		if (mp29526_i2c_read(bus, addr, 0xE0, pw, sizeof(pw)))
			LOG_INF("E0h readback: 0x%02x 0x%02x", pw[0], pw[1]);

		if (!mp29526_get_write_protect(bus, addr, &mp29526_stream.saved_wp)) {
			mp29526_stream.aborted = true;
			return false;
		}

		if (mp29526_stream.saved_wp != MP29526_WP_DISABLE) {
			LOG_INF("WRITE_PROTECT was 0x%02x, disabling for the update",
				mp29526_stream.saved_wp);
			if (!mp29526_set_write_protect(bus, addr, MP29526_WP_DISABLE)) {
				mp29526_stream.aborted = true;
				return false;
			}
		}

		/* clear any stale CML bits so the post-store check is meaningful */
		mp29526_i2c_write(bus, addr, MP29526_REG_CLEAR_FAULT, NULL, 0);
	}

	if (mp29526_stream.aborted) {
		LOG_ERR("mp29526 stream update already aborted, dropping chunk");
		return false;
	}

	if (!mp29526_stream_consume(data, data_len)) {
		mp29526_stream.aborted = true;
		mp29526_stream_restore_wp();
		return false;
	}

	if (!is_last)
		return true;

	if (mp29526_stream.line_len) {
		LOG_ERR("Incomplete trailing config line (%d bytes) at end of image",
			mp29526_stream.line_len);
		mp29526_stream.aborted = true;
		mp29526_stream_restore_wp();
		return false;
	}

	/* well-formed files hit "END" and finish their last group there; this
	 * only fires for a truncated/malformed file that never saw "END" */
	if (!mp29526_stream.past_end) {
		if (!mp29526_stream_flush_block() ||
		    !mp29526_stream_store_and_verify(mp29526_stream.group)) {
			mp29526_stream.aborted = true;
			mp29526_stream_restore_wp();
			return false;
		}
	}

	if (!mp29526_stream.applied_cnt) {
		LOG_ERR("parsing image fail");
		mp29526_stream.aborted = true;
		mp29526_stream_restore_wp();
		return false;
	}

	LOG_INF("MP29526 config stored to NVM (%d registers)", mp29526_stream.applied_cnt);
	mp29526_stream_restore_wp();
	return true;
}

/*
 * PLDM-facing entry point: works out the chunk-boundary bookkeeping
 * (next_ofs/next_len, first/last chunk) that pldm_vr_update() would
 * otherwise do inline for a hex_buff-based VR, then feeds the chunk to
 * mp29526_fwupdate_stream() above. Kept here (rather than in
 * pldm_firmware_update.c) so all the mp29526 update logic lives in one
 * place; see pldm_cpld_update()'s lattice.c equivalent for the same pattern.
 */
uint8_t mp29526_pldm_update(pldm_fw_update_param_t *p)
{
	bool is_first = (p->data_ofs == 0);

	p->next_ofs = p->data_ofs + p->data_len;
	p->next_len = fw_update_cfg.max_buff_size;

	bool is_last = (p->next_ofs >= fw_update_cfg.image_size);
	if (!is_last) {
		if (p->next_ofs + p->next_len > fw_update_cfg.image_size)
			p->next_len = fw_update_cfg.image_size - p->next_ofs;
	} else {
		p->next_len = 0;
	}

	if (is_first)
		LOG_INF("MP29526 update start");

	if (!mp29526_fwupdate_stream(p->bus, p->addr, p->data, p->data_len, is_first, is_last)) {
		LOG_ERR("MP29526 streaming update failed");
		return 1;
	}

	return 0;
}

/* =============================================================================
 * Sensor read
 * ========================================================================== */

uint8_t mp29526_read(sensor_cfg *cfg, int *reading)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_UNSPECIFIED_ERROR);
	CHECK_NULL_ARG_WITH_RETURN(reading, SENSOR_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX) {
		LOG_ERR("sensor num: 0x%x is invalid", cfg->num);
		return SENSOR_UNSPECIFIED_ERROR;
	}

	/* rail index travels in arg0 so one driver serves both loops */
	uint8_t rail = cfg->arg0;
	if (rail >= MP29526_RAIL_MAX) {
		LOG_ERR("sensor num: 0x%x has invalid rail arg0=%d", cfg->num, rail);
		return SENSOR_PARAMETER_NOT_VALID;
	}

	float val;

	/*
	 * READ_VOUT needs the VID step, and reading VOUT_MODE leaves the page
	 * already selected on the rail, so fetch it before the telemetry read
	 * and do not re-page in between.
	 */
	float vid_step = 0;
	if (cfg->offset == MP29526_REG_READ_VOUT) {
		vid_step = mp29526_get_vid_step(cfg, rail);
		if (vid_step == 0)
			return SENSOR_FAIL_TO_ACCESS;
	}

	if (!mp29526_select_rail(cfg, rail))
		return SENSOR_FAIL_TO_ACCESS;

	uint8_t retry = 5;
	sensor_val *sval = (sensor_val *)reading;
	I2C_MSG msg;
	memset(sval, 0, sizeof(sensor_val));

	msg.bus = cfg->port;
	msg.target_addr = cfg->target_addr;
	msg.tx_len = 1;
	msg.rx_len = 2;
	msg.data[0] = cfg->offset;

	if (i2c_master_read(&msg, retry)) {
		LOG_WRN("I2C read failed");
		return SENSOR_FAIL_TO_ACCESS;
	}

	uint16_t raw = (msg.data[1] << 8) | msg.data[0];

	switch (cfg->offset) {
	case MP29526_REG_READ_VOUT:
		/* bits[11:0], scaled by VOUT_MODE */
		val = (float)(raw & MP29526_READ_VOUT_MASK) * vid_step;
		break;

	case MP29526_REG_READ_VIN:
	case MP29526_REG_READ_IIN:
	case MP29526_REG_READ_IOUT:
	case MP29526_REG_READ_POUT:
	case MP29526_REG_READ_PIN:
		val = slinear11_to_float(raw);
		break;

	case MP29526_REG_READ_TEMP:
		/*
		 * linear11 with the exponent fixed at 0, so the mantissa is a
		 * plain 11-bit two's complement count of degrees C. Sign-extend
		 * bit[10] by hand: passing this through the generic linear11
		 * helper would work, but doing it explicitly documents why the
		 * negative range (-1C = 0x7FF, -40C = 0x7D8) decodes correctly.
		 */
		{
			int16_t t = (int16_t)(raw & GENMASK(10, 0));
			if (t & BIT(10))
				t -= (1 << 11);
			val = (float)t;
		}
		break;

	case MP29526_REG_READ_DUTY_CYCLE:
		val = (float)raw; /* 1 %/LSB */
		break;

	case MP29526_REG_READ_FREQUENCY:
		val = (float)raw; /* 1 kHz/LSB */
		break;

	default:
		LOG_ERR("Sensor num 0x%x, offset 0x%x not supported.", cfg->num, cfg->offset);
		return SENSOR_FAIL_TO_ACCESS;
	}

	sval->integer = (int16_t)val;
	sval->fraction = (int16_t)((val - sval->integer) * 1000);

	return SENSOR_READ_SUCCESS;
}

uint8_t mp29526_init(sensor_cfg *cfg)
{
	CHECK_NULL_ARG_WITH_RETURN(cfg, SENSOR_INIT_UNSPECIFIED_ERROR);

	if (cfg->num > SENSOR_NUM_MAX)
		return SENSOR_INIT_UNSPECIFIED_ERROR;

	if (cfg->arg0 >= MP29526_RAIL_MAX) {
		LOG_ERR("sensor num 0x%x: arg0 must be the rail index (0 or 1), got %d", cfg->num,
			cfg->arg0);
		return SENSOR_INIT_UNSPECIFIED_ERROR;
	}

	cfg->read = mp29526_read;
	return SENSOR_INIT_SUCCESS;
}
