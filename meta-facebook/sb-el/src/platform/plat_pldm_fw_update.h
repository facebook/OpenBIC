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

#ifndef _PLAT_FWUPDATE_H_
#define _PLAT_FWUPDATE_H_

#define PLAT_CRC32_READ_SIZE 128
#define PLAT_FLASH_BOOT0_VER_OFFSET 0x1FFFF8

#define RESET_CPLD_ON 0x3F
#define RESET_CPLD_OFF 0x00

enum ASIC_BOOT0_COMPONENT {
	BOOT0_HAMSA,
	BOOT0_NUWA0,
	BOOT0_NUWA1,
	BOOT0_MAX,
};

enum ASIC_IMG_NUM {
	IMG_BOOT1_HAMSA,
	IMG_BOOT0_HAMSA,
	IMG_BOOT0_NUWA0,
	IMG_BOOT0_NUWA1,
	IMG_UNKNOWN,
};

enum FLASH_VER_CRC {
	VERSION,
	CRC32,
	MAX_VER_CRC,
};

enum FIRMWARE_COMPONENT {
	COMPNT_BIC,
	COMPNT_VR_1,
	COMPNT_VR_2,
	COMPNT_VR_3,
	COMPNT_VR_4,
	COMPNT_VR_5,
	COMPNT_VR_6,
	COMPNT_VR_7,
	COMPNT_VR_8,
	COMPNT_VR_9,
	COMPNT_VR_10,
	COMPNT_VR_11,
	COMPNT_VR_12,
	COMPNT_VR_13,
	COMPNT_HAMSA,
	COMPNT_NUWA0,
	COMPNT_NUWA1,
	COMPNT_VR_3V3,
	COMPNT_HAMSA_BOOT1,
};

void plat_set_cpld_reset_reg(uint8_t value);
bool find_sensor_id_and_name_by_firmware_comp_id(uint8_t comp_identifier, uint8_t *sensor_id,
						 char *sensor_name);
uint32_t plat_get_image_crc_checksum(uint8_t index);
int sb_read_block(uint8_t slv_id, uint8_t cmd, uint8_t *data, uint32_t len);
bool plat_get_image_crc_checksum_from_flash(uint8_t index, uint32_t *data_ver, uint32_t *data_crc);
uint32_t plat_get_image_version(uint8_t index);
void update_temp_boot0_version(uint32_t version, uint8_t index);
#endif /* _PLAT_FWUPDATE_H_ */
