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

#ifndef TMP75_H
#define TMP75_H

enum TMP75_REIGSTER_MAP {
	TMP75_LOCAL_HIGH_LIMIT_REG = 0x03,
	TMP75_LOCAL_LOW_LIMIT_REG = 0x02,
};

bool tmp75_get_temp_threshold(sensor_cfg *cfg, uint8_t temp_threshold_index,
			      uint32_t *millidegree_celsius);
bool tmp75_set_temp_threshold(sensor_cfg *cfg, uint8_t temp_threshold_index,
			      uint32_t *millidegree_celsius);

#endif
