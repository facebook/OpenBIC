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
 * MP29526 - Dual Loop Digital Multi-Phase Controller (MPS)
 *
 * Datasheet    : MP29526 Rev. 0.1, 2/26/2026 (preliminary)
 * Register map : MP29526 Register Map Rev 1.0.0.196 / 2.0.0.233, 2026-07-28
 * Reference    : mp29816a.c / mp29816a.h
 *
 * The register map resolves everything the preliminary datasheet left TBD.
 * All command codes, widths, and bit fields below are taken from it.
 *
 * PAGE MAP (PAGE 00h bits[5:0]):
 *   0 = Rail 1 operating registers      5 = MPS registers affecting Rail 1
 *   1 = Rail 2 operating registers      6 = MPS registers affecting Rail 2
 *   2 = Rail 3 (unused on MP29526)      7 = MPS Rail 3 (unused)
 *   3 = Rail 4 (unused on MP29526)      8 = MPS Rail 4 (unused)
 *   4 = SVID psys                       9,10 = MPS all-rail, 11 = debug
 *
 * MP29526 is dual loop, so only rails 1 and 2 exist: pages 0/5 and 1/6.
 */

#ifndef MP29526_H
#define MP29526_H

#include "stdint.h"
#include "sensor.h"

/* Avoids pulling pldm_firmware_update.h into every mp29526.h consumer just
 * for the pointer type below; the full struct is only needed in mp29526.c. */
struct pldm_fw_update_param;

enum mp29526_rail {
	MP29526_RAIL_1 = 0,
	MP29526_RAIL_2 = 1,
	MP29526_RAIL_MAX,
};

/*
 * VOUT_MODE (20h) encodings. Every VID-format register - VOUT_COMMAND,
 * VOUT_MAX/MIN, VOUT_OV/UV_FAULT_LIMIT, VBOOT, READ_VOUT - is scaled by
 * whichever of these the rail is configured for.
 */
#define MP29526_VOUT_MODE_DIRECT_1MV 0x40 /* 1 mV/LSB                    */
#define MP29526_VOUT_MODE_VID_10MV 0x22 /* VID table, 10 mV/LSB          */
#define MP29526_VOUT_MODE_VID_5MV 0x21 /* VID table, 5 mV/LSB            */
#define MP29526_VOUT_MODE_LINEAR_512 0x17 /* linear, 1/512 V = 1.953125mV */

/* WRITE_PROTECT (10h) */
#define MP29526_WP_DISABLE 0x00
#define MP29526_WP_ALL_EXCEPT_WP 0x80
#define MP29526_WP_EXCEPT_WP_OP_PAGE 0x40
#define MP29526_WP_EXCEPT_WP_OP_PAGE_ONOFF_VOUT 0x20

/* STATUS_MFR_PROTECT (80h on page 5/6), 2 bytes, read only */
#define MP29526_PROTECT_LINE_FLOAT BIT(0)
#define MP29526_PROTECT_PWM_SELF_CHECK BIT(1)
#define MP29526_PROTECT_VAUX_FAULT BIT(2)
#define MP29526_PROTECT_CHIP_OTP BIT(3)
#define MP29526_PROTECT_LIMP_MODE BIT(4)
#define MP29526_PROTECT_DRMOS_FAULT BIT(8)
#define MP29526_PROTECT_ITR_FLT BIT(9)
#define MP29526_PROTECT_PHASE_REDUNDANCY BIT(11)

/* STATUS_CML (7Eh) */
#define MP29526_CML_NVM_CRC_ERROR BIT(4)
#define MP29526_CML_PEC_ERROR BIT(5)

/* ---------------- low level helpers -------------------------------------- */
bool mp29526_i2c_read(uint8_t bus, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);
bool mp29526_i2c_write(uint8_t bus, uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);
bool mp29526_set_page(uint8_t bus, uint8_t addr, uint8_t page);

/* ---------------- API mirroring mp29816a.h ------------------------------- */
bool mp29526_get_fw_version(uint8_t bus, uint8_t addr, uint32_t *rev);
bool mp29526_get_vout_max(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool mp29526_get_vout_min(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool mp29526_set_vout_max(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool mp29526_set_vout_min(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool mp29526_get_iout_oc_warn_limit(sensor_cfg *cfg, uint16_t *value);
bool mp29526_set_iout_oc_warn_limit(sensor_cfg *cfg, uint16_t value);
bool mp29526_fwupdate(uint8_t bus, uint8_t addr, uint8_t *img_buff, uint32_t img_size);
/*
 * Line-streaming variant of mp29526_fwupdate(): applies the tab-separated
 * config lines to the device as each chunk arrives instead of buffering the
 * whole (~200KB) image in RAM first. Call once per PLDM firmware data chunk,
 * in offset order, with is_first true on the very first chunk (offset 0) and
 * is_last true on the chunk that completes the image.
 */
bool mp29526_fwupdate_stream(uint8_t bus, uint8_t addr, const uint8_t *data, uint32_t data_len,
			     bool is_first, bool is_last);
/* PLDM-facing wrapper around mp29526_fwupdate_stream(): call once per
 * RequestFirmwareData chunk from pldm_vr_update(). */
uint8_t mp29526_pldm_update(struct pldm_fw_update_param *p);
bool mp29526_get_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool mp29526_set_vout_command(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool mp29526_get_vr_status(sensor_cfg *cfg, uint8_t rail, uint8_t vr_status_rail,
			   uint16_t *vr_status);
bool mp29526_clear_vr_status(sensor_cfg *cfg, uint8_t rail);
bool mp29526_get_uvp(sensor_cfg *cfg, uint16_t *uvp_threshold);
bool mp29526_set_uvp_threshold(sensor_cfg *cfg, uint16_t *write_uvp_threshold);
bool mp29526_get_vout_offset(sensor_cfg *cfg, uint16_t *vout_offset);
bool mp29526_set_vout_offset(sensor_cfg *cfg, uint16_t *write_vout_offset);
bool mp29526_get_total_ocp(sensor_cfg *cfg, uint16_t *total_ocp);
bool mp29526_set_total_ocp(sensor_cfg *cfg, uint16_t *write_total_ocp);
bool mp29526_get_ovp_1(sensor_cfg *cfg, uint16_t *ovp_1);
bool mp29526_set_ovp_1(sensor_cfg *cfg, uint16_t *write_ovp_1);
bool mp29526_get_ovp_2(sensor_cfg *cfg, uint16_t *ovp_2);
bool mp29526_get_ovp_2_action(sensor_cfg *cfg, uint16_t *ovp_2_action);
bool mp29526_set_ovp_2_action(sensor_cfg *cfg, uint16_t *write_ovp_2_action);

/* ---------------- MP29526 specific --------------------------------------- */

/* VID step in volts, decoded from VOUT_MODE (20h). Returns 0.0f on failure. */
float mp29526_get_vid_step(sensor_cfg *cfg, uint8_t rail);

/* Raw VOUT_MODE byte for the rail. */
bool mp29526_get_vout_mode(sensor_cfg *cfg, uint8_t rail, uint8_t *mode);

/* VID_OVP / VID_UVP thresholds, page 5/6 06h and 07h, 1 mV/LSB. */
bool mp29526_get_ov_vid_threshold(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool mp29526_set_ov_vid_threshold(sensor_cfg *cfg, uint8_t rail, uint16_t millivolt);
bool mp29526_get_uv_vid_threshold(sensor_cfg *cfg, uint8_t rail, uint16_t *millivolt);
bool mp29526_set_uv_vid_threshold(sensor_cfg *cfg, uint8_t rail, uint16_t millivolt);

/* STATUS_MFR_PROTECT (80h page 5/6). Mask with MP29526_PROTECT_* above. */
bool mp29526_get_mfr_protect(sensor_cfg *cfg, uint8_t rail, uint16_t *status);
bool mp29526_get_line_float_fault(sensor_cfg *cfg, uint8_t rail, bool *is_faulted);

/* PHASE_NUM (01h page 5/6): 0 = off, 1..16 = active phase count. */
bool mp29526_get_phase_count(sensor_cfg *cfg, uint8_t rail, uint8_t *phase_cnt);

/* IC_DEVICE_ID (ADh) check - confirms the part really is an MP29526. */
bool mp29526_check_device_id(uint8_t bus, uint8_t addr);

/* WRITE_PROTECT (10h) */
bool mp29526_get_write_protect(uint8_t bus, uint8_t addr, uint8_t *wp);
bool mp29526_set_write_protect(uint8_t bus, uint8_t addr, uint8_t wp);

#endif /* MP29526_H */
