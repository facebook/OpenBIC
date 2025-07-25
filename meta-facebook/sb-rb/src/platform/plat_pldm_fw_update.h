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
};

bool find_sensor_id_and_name_by_firmware_comp_id(uint8_t comp_identifier, uint8_t *sensor_id,
						 char *sensor_name);

#endif /* _PLAT_FWUPDATE_H_ */