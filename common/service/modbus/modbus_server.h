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

/** @name Modbus exception codes
 *  @{
 */
/* Modbus exception codes */
#define MODBUS_EXC_NONE				0
#define MODBUS_EXC_ILLEGAL_FC			1
#define MODBUS_EXC_ILLEGAL_DATA_ADDR		2
#define MODBUS_EXC_ILLEGAL_DATA_VAL		3
#define MODBUS_EXC_SERVER_DEVICE_FAILURE	4
#define MODBUS_EXC_ACK				5
#define MODBUS_EXC_SERVER_DEVICE_BUSY		6
#define MODBUS_EXC_MEM_PARITY_ERROR		8
#define MODBUS_EXC_GW_PATH_UNAVAILABLE		10
#define MODBUS_EXC_GW_TARGET_FAILED_TO_RESP	11
/** @} */
