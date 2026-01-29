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

#ifndef _PLAT_VR_TEST_MODE_H_
#define _PLAT_VR_TEST_MODE_H_

#define VR_FAST_OCP_REG 0x46
#define VR_SLOW_OCP_REG 0xEA
#define VR_UVP_REG 0x44
#define VR_OVP_REG 0x40
#define VR_VOUT_MAX_REG 0x24
#define VR_VOUT_REG 0x21
#define VR_LOOPCFG_REG 0xF0

#define VR_UVP_0_DMA_ADDR 0xEA3D
#define VR_UVP_1_DMA_ADDR 0xEABD
#define VR_OVP_0_DMA_ADDR 0xEA3E
#define VR_OVP_1_DMA_ADDR 0xEABE

#define NO_USE 0xFF //VR_UVP_THRESHOLD_GAIN_NO_USE
#define NO_ACTION 0 // ovp2 threshold no action
#define LATCH_OFF 1 // ovp2 threshold latch off
#define ENABLE 1 // divider enable
#define DISABLE 0 // divider disable

typedef struct {
	uint8_t vr_rail;
	uint16_t fast_ocp;
	uint16_t slow_ocp;
	uint16_t uvp;
	uint16_t ovp;
	uint16_t vout_max;
	uint16_t lcr;
	uint16_t ucr;
	uint16_t vout_default;
} vr_test_mode_setting_t;

typedef struct {
	uint8_t vr_rail;
	uint16_t total_ocp;
	uint16_t mp2971_uvp_threshold_gain;
	uint16_t uvp_threshold;
	uint16_t lcr;
	uint16_t ucr;
	uint16_t ovp2_threshold;
	uint16_t ovp1_threshold;
	uint16_t ovp2;
	uint16_t ovp1;
	uint16_t vout_default;
	uint16_t vout_max;
	uint16_t uvp;
} mps_vr_test_mode_setting_t;

extern const vr_test_mode_setting_t vr_test_mode_table[];
extern const vr_test_mode_setting_t vr_test_mode_table_default[];
extern const uint8_t vr_test_mode_table_size;
extern const uint8_t vr_test_mode_table_dafault_size;

extern const mps_vr_test_mode_setting_t vr_mps_test_mode_table[];
extern const mps_vr_test_mode_setting_t vr_mps_normal_mode_table[];
extern const uint8_t vr_mps_test_mode_table_size;
extern const uint8_t vr_mps_normal_mode_table_size;

bool get_vr_test_mode_flag(void);
void vr_test_mode_enable(bool onoff);
void init_vr_test_mode_polling(void);
bool dma_write_vr(uint8_t rail, uint16_t reg, uint8_t *data, uint8_t len);
bool dma_read_vr(uint8_t rail, uint16_t reg, uint8_t *data, uint8_t len);
bool get_vr_offset_uvp_ovp(uint8_t rail, uint16_t *uvp, uint16_t *ovp);
bool use_offset_uvp_ovp(uint8_t rail);
bool get_vr_fixed_uvp_ovp_enable(uint8_t rail);
bool set_vr_fixed_uvp_ovp_enable(uint8_t rail, uint8_t enable);
#endif /* _PLAT_VR_TEST_MODE_H_ */
