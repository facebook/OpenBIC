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
bool modbus_i2c_master_write_read(const uint16_t *modbus_data, uint8_t data_len);
void modbus_i2c_master_write_read_response(uint16_t *modbus_data);
void regs_reverse(uint16_t reg_len, uint16_t *data);
void plat_enable_sensor_poll();
void plat_disable_sensor_poll();
void set_rpu_ready();
uint8_t get_rpu_ready_pin_status();
float pow_of_10(int8_t exp);