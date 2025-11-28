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

#ifndef PLAT_IRIS_SMBUS_H
#define PLAT_IRIS_SMBUS_H

/*
 * smbus sideband operating modes
 */
#define FASTBOOT_MODE BIT(0)
#define CMRT_SIC_MODE BIT(1)
#define RECOVERY_MODE BIT(2)

#define BYTE_MASK 0xff

// Hamsa boot1 binary at 0x94008000
#define HAMSA_BOOT1_ASIC_MEM_ADDR 0x94008000
#define HAMSA_BOOT1_ADDR 0x32 //0x32(7-bit)

// smbus2.0 spec restrict max bytes in an atomic transfer to 32
#define SMBUS_MAX_PKT_LEN 32
// maximum data payload excluding metadata(cmd, len and addr) overheads
#define SMBUS_MAX_PAYLOAD_LEN 24

#define MSG_PKT_LEN_OFFSET 0
#define MSG_ADDR_OFFSET 1
#define ADDRESS_BYTE_SZ 4
#define DATA_LEN_BYTE_SZ 1
#define MSG_DATA_LEN_OFFSET 5
#define MSG_RDWR_DATA_START 6
#define BYTES_PER_WORD 4
#define PKT_LEN_BYTE_SZ 1

#define ADDR_DATA_LEN_SZ (ADDRESS_BYTE_SZ + DATA_LEN_BYTE_SZ)
// FW download
#define MAX_RETRIES 3
// fw download status/control bytes
#define FW_DL_START BIT(7)
#define FW_DL_SLV_RDY BIT(6)
#define FW_DL_HST_ABRT BIT(5)
#define FW_DL_SLV_ABRTD BIT(4)
#define FW_DL_FINISH BIT(3)
#define FW_DL_SLV_DONE BIT(2)
#define FW_DL_SLV_PROG BIT(1)
#define FW_SB_EXIT_CMD BIT(0)

#define MEDHA0_I2C_ADDR 0x33
#define MEDHA1_I2C_ADDR 0x34
#define OWLW_I2C_ADDR 0x6E
#define OWLE_I2C_ADDR 0x6E

#define MAX_DATA_PKT_SIZE 24
#define STATUS_RETRY_CNT 2
#define ERROR_CODE_LEN 8

// smbus error code
#define SMBUS_ERROR 122
// asic_error_code
#define ASIC_ERROR 123
// module_error_code
#define MODULE_ERROR 124
// smbus/firmware protocol error
#define FW_SMBUS_ERROR 130
// custom device error report
#define SB_MODE_QUERY 135
// fw download control write
#define FW_CTRL_WRITE 136
// fw download control read
#define FW_CTRL_READ 137
// firmware block write
#define FW_DATA_WRITE 138

int iris_smbus_fast_boot(uint8_t *image_buff, uint32_t img_dest_addr, uint32_t img_size);
#endif