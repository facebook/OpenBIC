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

#ifndef PLAT_IOEXP_H
#define PLAT_IOEXP_H

#define PCA6414A_INPUT_PORT_0 0x00
#define PCA6414A_INPUT_PORT_1 0x01
#define PCA6414A_OUTPUT_PORT_0 0x02
#define PCA6414A_OUTPUT_PORT_1 0x03
#define PCA6414A_CONFIG_0 0x06
#define PCA6414A_CONFIG_1 0x07

#define TCA6424A_INPUT_PORT_0 0x00
#define TCA6424A_INPUT_PORT_1 0x01
#define TCA6424A_INPUT_PORT_2 0x02
#define TCA6424A_OUTPUT_PORT_0 0x04
#define TCA6424A_OUTPUT_PORT_1 0x05
#define TCA6424A_OUTPUT_PORT_2 0x06
#define TCA6424A_CONFIG_0 0x0C
#define TCA6424A_CONFIG_1 0x0D
#define TCA6424A_CONFIG_2 0x0E
#define U200052_IO_ADDR 0x3A
#define U200053_IO_ADDR 0x3B
#define U200070_IO_ADDR 0x3C
#define U200051_IO_I2C_BUS I2C_BUS1
#define U200052_IO_I2C_BUS I2C_BUS1
#define U200053_IO_I2C_BUS I2C_BUS1
#define U200070_IO_I2C_BUS I2C_BUS1

#define U200052_IO_INIT_VAL 0xC0
#define U200053_IO_INIT_VAL 0x40
#define U200070_IO_INIT_VAL 0x07

#define HAMSA_MFIO19 1
#define HAMSA_MFIO6 6
#define HAMSA_MFIO8 7
#define HAMSA_MFIO10 0
#define MEDHA0_MFIO6 7
#define MEDHA0_MFIO8 6
#define MEDHA0_MFIO10 5
#define MEDHA1_MFIO6 4
#define MEDHA1_MFIO8 3
#define MEDHA1_MFIO10 2

bool pca6416a_i2c_read(uint8_t offset, uint8_t *data, uint8_t len);
bool pca6416a_i2c_write(uint8_t offset, uint8_t *data, uint8_t len);
bool is_tca6424a_accessible();
bool tca6424a_i2c_read(uint8_t offset, uint8_t *data, uint8_t len);
bool tca6424a_i2c_read_drive_value(uint8_t group, uint8_t bit, uint8_t *data);
bool tca6424a_i2c_write(uint8_t offset, uint8_t *data, uint8_t len);
bool tca6424a_i2c_write_bit(uint8_t offset, uint8_t bit, uint8_t val);
void set_hamsa_mfio_6_8_10_input();
void set_medha0_mfio_6_8_10_input();
void set_medha1_mfio_6_8_10_input();
void set_hamsa_mfio_6_8_10_output();
void set_medha0_mfio_6_8_10_output();
void set_medha1_mfio_6_8_10_output();
void ioexp_init(void);
void init_U200052_IO();
void init_U200053_IO();
void init_U200070_IO();
void set_pca6554apw_ioe_value(uint8_t ioe_bus, uint8_t ioe_addr, uint8_t ioe_reg, uint8_t value);
int get_pca6554apw_ioe_value(uint8_t ioe_bus, uint8_t ioe_addr, uint8_t ioe_reg, uint8_t *value);

#endif
