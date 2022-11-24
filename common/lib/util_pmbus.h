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

#ifndef UTIL_PMBUS_H
#define UTIL_PMBUS_H

float slinear11_to_float(uint16_t);
bool get_exponent_from_vout_mode(uint8_t, float *);
int pmbus_read_command(uint8_t sensor_num, uint8_t command, uint8_t *result, uint8_t read_len);

#endif
